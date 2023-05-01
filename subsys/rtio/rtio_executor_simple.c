/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtio_executor_common.h"
#include <zephyr/rtio/rtio_executor_simple.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rtio_executor_simple, CONFIG_RTIO_LOG_LEVEL);


/**
 * @brief Submit submissions to simple executor
 *
 * The simple executor provides no concurrency instead
 * execution each submission chain one after the next.
 *
 * @param r RTIO context
 *
 * @retval 0 Always succeeds
 */
int rtio_simple_submit(struct rtio *r)
{
	struct rtio_simple_executor *exc = (struct rtio_simple_executor *)r->executor;

	/* Task is already running */
	if (exc->task.sqe != NULL) {
		return 0;
	}

	LOG_DBG("Consuming from SQ");
	struct rtio_sqe *sqe = rtio_spsc_consume(r->sq);

	if (sqe == NULL) {
		LOG_DBG("Nothing to consume, waiting");
		return 0;
	}

	/* Some validation on the sqe to ensure no programming errors were
	 * made so assumptions in ok and err are valid.
	 */
#ifdef CONFIG_ASSERT
	__ASSERT(sqe != NULL, "Expected a valid sqe on submit call");

	bool transaction = sqe->flags & RTIO_SQE_TRANSACTION;
	bool chained = sqe->flags & RTIO_SQE_CHAINED;

	if (transaction || chained) {
		struct rtio_sqe *next = rtio_spsc_next(r->sq, sqe);

		__ASSERT(next != NULL,
			"sqe %p flagged as transaction (%d) or chained (%d) without subsequent sqe in queue",
			sqe, transaction, chained);
	}
	__ASSERT(!(chained && transaction),
		"sqe %p flagged as both being transaction and chained, only one is allowed",
		sqe);
#endif

	exc->task.sqe = sqe;
	exc->task.r = r;

	if (FIELD_GET(RTIO_SQE_CANCELED, sqe->flags) != 0) {
		rtio_iodev_sqe_err(&exc->task, -ECANCELED);
	} else {
		rtio_executor_submit(&exc->task);
	}

	return 0;
}

static inline void rtio_simple_spsc_release(struct rtio *r, struct rtio_sqe *sqe)
{
	if (FIELD_GET(RTIO_SQE_MULTISHOT, sqe->flags) == 0) {
		LOG_DBG("Release SQEs");
		rtio_spsc_release(r->sq);
	} else if (FIELD_GET(RTIO_SQE_MEMPOOL_BUFFER, sqe->flags) == 1 && sqe->op == RTIO_OP_RX) {
		/* Multi-shot read with mempool buffer, reset the memory for the next read */
		LOG_DBG("Multi-shot with mempool detected, clearing buffers for %p", (void *)sqe);
		sqe->buf = NULL;
		sqe->buf_len = 0;
	}
}

/**
 * @brief Callback from an iodev describing success
 */
void rtio_simple_ok(struct rtio_iodev_sqe *iodev_sqe, int result)
{
	struct rtio *r = iodev_sqe->r;
	struct rtio_sqe *sqe = &r->sq->buffer[iodev_sqe->sqe - r->sq->buffer];
	bool transaction;

#ifdef CONFIG_ASSERT
	struct rtio_simple_executor *exc =
		(struct rtio_simple_executor *)r->executor;

	__ASSERT_NO_MSG(iodev_sqe == &exc->task);
#endif

	do {
		/* Capture the sqe information */
		void *userdata = sqe->userdata;
		uint32_t cqe_flags = rtio_cqe_compute_flags(iodev_sqe);

		transaction = sqe->flags & RTIO_SQE_TRANSACTION;

		/* Release the sqe */
		rtio_simple_spsc_release(r, sqe);

		/* Submit the completion event */
		rtio_cqe_submit(r, result, userdata, cqe_flags);

		if (transaction) {
			/* sqe was a transaction, get the next one */
			sqe = rtio_spsc_consume(r->sq);
			__ASSERT_NO_MSG(sqe != NULL);
		}

	} while (transaction);

	iodev_sqe->sqe = NULL;
	rtio_simple_submit(r);
}

/**
 * @brief Callback from an iodev describing error
 *
 * Some assumptions are made and should have been validated on rtio_submit
 * - a sqe marked as chained or transaction has a next sqe
 * - a sqe is marked either chained or transaction but not both
 */
void rtio_simple_err(struct rtio_iodev_sqe *iodev_sqe, int result)
{
	struct rtio *r = iodev_sqe->r;
	struct rtio_sqe *sqe = &r->sq->buffer[iodev_sqe->sqe - r->sq->buffer];
	void *userdata = sqe->userdata;
	uint32_t flags = rtio_cqe_compute_flags(iodev_sqe);
	bool chained = sqe->flags & RTIO_SQE_CHAINED;
	bool transaction = sqe->flags & RTIO_SQE_TRANSACTION;

#ifdef CONFIG_ASSERT
	struct rtio_simple_executor *exc =
		(struct rtio_simple_executor *)r->executor;

	__ASSERT_NO_MSG(iodev_sqe == &exc->task);
#endif

	rtio_simple_spsc_release(r, sqe);
	iodev_sqe->sqe = NULL;
	if (!transaction) {
		rtio_cqe_submit(r, result, userdata, flags);
	}
	while (chained | transaction) {
		sqe = rtio_spsc_consume(r->sq);
		chained = sqe->flags & RTIO_SQE_CHAINED;
		transaction = sqe->flags & RTIO_SQE_TRANSACTION;
		userdata = sqe->userdata;
		rtio_simple_spsc_release(r, sqe);

		if (!transaction) {
			rtio_cqe_submit(r, result, userdata, flags);
		} else {
			rtio_cqe_submit(r, -ECANCELED, userdata, flags);
		}
	}

	iodev_sqe->sqe = rtio_spsc_consume(r->sq);
	if (iodev_sqe->sqe != NULL) {
		rtio_executor_submit(iodev_sqe);
	}
}
