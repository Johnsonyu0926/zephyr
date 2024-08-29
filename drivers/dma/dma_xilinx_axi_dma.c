/** @file
 *@brief Driver for Xilinx AXI DMA.
 */
/*
 * Copyright (c) 2024 CISPA Helmholtz Center for Information Security gGmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#include "dma_xilinx_axi_dma.h"

#define XILINX_AXI_DMA_SG_DESCRIPTOR_ADDRESS_MASK 0x3f

LOG_MODULE_REGISTER(dma_xilinx_axi_dma, CONFIG_DMA_LOG_LEVEL);

/* masks for control field in SG descriptor */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_CTRL_RESERVED_MASK 0xF0000000
/* descriptor is for start of transfer */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_CTRL_SOF_MASK      0x08000000
/* descriptor is for end of transfer */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_CTRL_EOF_MASK      0x04000000
/* length of the associated buffer in main memory */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_CTRL_LENGTH_MASK   0x03FFFFFF
#define XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_LENGTH_MASK 0x03FFFFFF

/* masks for status field in SG descriptor */
/* transfer completed */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_COMPLETE_MASK    0x80000000
/* decode error, i.e., DECERR on AXI bus from memory */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_DEC_ERR_MASK     0x40000000
/* slave error, i.e., SLVERR on AXI bus from memory */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_SLV_ERR_MASK     0x20000000
/* internal DMA error, e.g., 0-length transfer */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_INT_ERR_MASK     0x10000000
/* reserved */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_INT_RES_MASK     0x0C000000
/* number of transferred bytes */
#define XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_TRANSFERRED_MASK 0x03FFFFFF

#define XILINX_AXI_DMA_SG_DESCRIPTOR_APP0_CHECKSUM_OFFLOAD_FULL 0x00000002
#define XILINX_AXI_DMA_SG_DESCRIPTOR_APP0_CHECKSUM_OFFLOAD_NONE 0x00000000
#define XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_FCS_ERR_MASK          0x00000100
#define XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_IP_ERR_MASK           0x00000028
#define XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_UDP_ERR_MASK          0x00000030
#define XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_TCP_ERR_MASK          0x00000038

/* masks for DMA registers */

#define XILINX_AXI_DMA_REGS_DMACR_IRQTHRESH_SHIFT_BITS 16
#define XILINX_AXI_DMA_REGS_DMACR_IRQDELAY_SHIFT_BITS  24
/* masks for DMACR register */
/* interrupt timeout - trigger interrupt after X cycles when no transfer. Unit is 125 * */
/* clock_period. */
#define XILINX_AXI_DMA_REGS_DMACR_IRQDELAY             0xFF000000
/* irqthreshold - this can be used to generate interrupts after X completed packets */
/* instead of after every packet */
#define XILINX_AXI_DMA_REGS_DMACR_IRQTHRESH            0x00FF0000
#define XILINX_AXI_DMA_REGS_DMACR_RESERVED1            0x00008000
/* interrupt on error enable */
#define XILINX_AXI_DMA_REGS_DMACR_ERR_IRQEN            0x00004000
/* interrupt on delay timer interrupt enable */
#define XILINX_AXI_DMA_REGS_DMACR_DLY_IRQEN            0x00002000
/* interrupt on complete enable */
#define XILINX_AXI_DMA_REGS_DMACR_IOC_IRQEN            0x00001000
#define XILINX_AXI_DMA_REGS_DMACR_ALL_IRQEN                                                        \
	(XILINX_AXI_DMA_REGS_DMACR_ERR_IRQEN | XILINX_AXI_DMA_REGS_DMACR_DLY_IRQEN |               \
	 XILINX_AXI_DMA_REGS_DMACR_IOC_IRQEN)
#define XILINX_AXI_DMA_REGS_DMACR_RESERVED2 0x00000FE0
/* DMA ignores completed bit in SG descriptor and overwrites descriptors */
#define XILINX_AXI_DMA_REGS_DMACR_CYC_BD_EN 0x00000010
/* use AXI fixed burst instead of incrementing burst for TX transfers, e.g., useful for reading a */
/* FIFO */
#define XILINX_AXI_DMA_REGS_DMACR_KEYHOLE   0x00000008
/* soft reset */
#define XILINX_AXI_DMA_REGS_DMACR_RESET     0x00000004
#define XILINX_AXI_DMA_REGS_DMACR_RESERVED3 0x00000002
/* run-stop */
#define XILINX_AXI_DMA_REGS_DMACR_RS        0x00000001

/* masks for DMASR register */
/* interrupt delay time status */
#define XILINX_AXI_DMA_REGS_DMASR_IRQDELAYSTS  0xFF000000
/* interrupt threshold status */
#define XILINX_AXI_DMA_REGS_DMASR_IRQTHRESHSTS 0x00FF0000
#define XILINX_AXI_DMA_REGS_DMASR_RESERVED1    0x00008000
/* current interrupt was generated on error */
#define XILINX_AXI_DMA_REGS_DMASR_ERR_IRQ      0x00004000
/* current interrupt was generated by timoeout */
#define XILINX_AXI_DMA_REGS_DMASR_DLY_IRQ      0x00002000
/* current interrupt was generated by completion of a transfer */
#define XILINX_AXI_DMA_REGS_DMASR_IOC_IRQ      0x00001000
#define XILINX_AXI_DMA_REGS_DMASR_RESERVED2    0x00000800
/* scatter gather decode error */
#define XILINX_AXI_DMA_REGS_DMASR_SGDECERR     0x00000400
/* scatter gather slave error */
#define XILINX_AXI_DMA_REGS_DMASR_SGSLVERR     0x00000200
/* scatter gather internal error, i.e., fetched a descriptor with complete bit already set */
#define XILINX_AXI_DMA_REGS_DMASR_SGINTERR     0x00000100
#define XILINX_AXI_DMA_REGS_DMASR_RESERVED3    0x00000080
/* DMA decode error */
#define XILINX_AXI_DMA_REGS_DMASR_DMADECERR    0x00000040
/* DMA slave error */
#define XILINX_AXI_DMA_REGS_DMASR_SLVERR       0x00000020
/* DMA internal error */
#define XILINX_AXI_DMA_REGS_DMASR_INTERR       0x00000010
/* scatter/gather support enabled at build time */
#define XILINX_AXI_DMA_REGS_DMASR_SGINCL       0x00000008
#define XILINX_AXI_DMA_REGS_DMASR_RESERVED4    0x00000004
/* DMA channel is idle, i.e., DMA operations completed; writing tail restarts operation */
#define XILINX_AXI_DMA_REGS_DMASR_IDLE         0x00000002
/* RS (run-stop) in DMACR is 0 and operations completed; writing tail does nothing */
#define XILINX_AXI_DMA_REGS_DMASR_HALTED       0x00000001

#define XILINX_AXI_DMA_REGS_SG_CTRL_CACHE_MASK 0x0000000F
#define XILINX_AXI_DMA_REGS_SG_CTRL_RES1_MASK  0x000000F0
#define XILINX_AXI_DMA_REGS_SG_CTRL_USER_MASK  0x00000F00
#define XILINX_AXI_DMA_REGS_SG_CTRL_RES2_MASK  0xFFFFF000

#ifdef CONFIG_DMA_XILINX_AXI_DMA_DISABLE_CACHE_WHEN_ACCESSING_SG_DESCRIPTORS
#include <zephyr/arch/cache.h>
static inline void dma_xilinx_axi_dma_disable_cache(void)
{
	cache_data_disable();
}
static inline void dma_xilinx_axi_dma_enable_cache(void)
{
	cache_data_enable();
}
#else
static inline void dma_xilinx_axi_dma_disable_cache(void)
{
	/* do nothing */
}
static inline void dma_xilinx_axi_dma_enable_cache(void)
{
	/* do nothing */
}
#endif

/* in-memory descriptor, read by the DMA, that instructs it how many bits to transfer from which */
/* buffer */
struct __attribute__((__packed__)) dma_xilinx_axi_dma_sg_descriptor {
	/* next descriptor[31:6], bits 5-0 reserved */
	uint32_t nxtdesc;
	/* next descriptor[63:32] */
	uint32_t nxtdesc_msb;
	/* address of buffer to transfer[31:0] */
	uint32_t buffer_address;
	/* address of buffer to transfer[63:32] */
	uint32_t buffer_address_msb;
	uint32_t reserved1;
	uint32_t reserved2;

	/* bitfield, masks for access defined above */
	uint32_t control;
	/* bitfield, masks for access defined above */
	uint32_t status;

	/* application-specific fields used, e.g., to enable checksum offloading */
	/* for the Ethernet Subsystem */
	uint32_t app0;
	uint32_t app1;
	uint32_t app2;
	uint32_t app3;
	uint32_t app4;
} __aligned(64);

__aligned(64) static struct dma_xilinx_axi_dma_sg_descriptor
	descriptors_tx[CONFIG_DMA_XILINX_AXI_DMA_SG_DESCRIPTOR_NUM_TX] = {0};
__aligned(64) static struct dma_xilinx_axi_dma_sg_descriptor
	descriptors_rx[CONFIG_DMA_XILINX_AXI_DMA_SG_DESCRIPTOR_NUM_RX] = {0};
/* registers are the same with different name */
struct __attribute__((__packed__)) dma_xilinx_axi_dma_mm2s_s2mm_registers {
	/* DMA control register */
	/* bitfield, masks defined above */
	uint32_t dmacr;
	/* DMA status register */
	/* bitfield, masks defined above */
	uint32_t dmasr;
	/* current descriptor address[31:0] */
	uint32_t curdesc;
	/* current descriptor address[63:0] */
	uint32_t curdesc_msb;
	/* current descriptor address[31:0] */
	uint32_t taildesc;
	/* current descriptor address[63:0] */
	uint32_t taildesc_msb;
	/* transfer source address for "direct register mode"[31:0] */
	uint32_t sa;
	/* transfer source address for "direct register mode"[63:32] */
	uint32_t sa_msb;
	uint32_t reserved1;
	uint32_t reserved2;
	/* transfer length for "direct register mode" */
	uint32_t length;
};

struct __attribute__((__packed__)) dma_xilinx_axi_dma_register_space {
	struct dma_xilinx_axi_dma_mm2s_s2mm_registers mm2s_registers;
	/* scatter/gather user and cache register or reserved */
	/* controls arcache/awcache and aruser/awuser of generated transactions */
	uint32_t sg_ctl;
	struct dma_xilinx_axi_dma_mm2s_s2mm_registers s2mm_registers;
};

/* global configuration per DMA device */
struct dma_xilinx_axi_dma_config {
	void *reg;
	/* this should always be 2 - one for TX, one for RX */
	uint32_t channels;
	void (*irq_configure)();
	uint32_t *irq0_channels;
	size_t irq0_channels_size;
};

/* per-channel state */
struct dma_xilinx_axi_dma_channel {
	volatile struct dma_xilinx_axi_dma_sg_descriptor *descriptors;
	size_t num_descriptors;

	size_t current_transfer_start_index, current_transfer_end_index;

	volatile struct dma_xilinx_axi_dma_mm2s_s2mm_registers *channel_regs;

	enum dma_channel_direction last_transfer_direction;

	/* call this when the transfer is complete */
	dma_callback_t completion_callback;
	void *completion_callback_user_data;

	uint32_t last_rx_size;

	uint32_t sg_desc_app0;
	bool check_csum_in_isr;
};

/* global state for device and array of per-channel states */
struct dma_xilinx_axi_dma_data {
	struct dma_context ctx;
	struct dma_xilinx_axi_dma_channel *channels;
	bool device_has_been_reset;
};

#ifdef CONFIG_DMA_XILINX_AXI_DMA_LOCK_ALL_IRQS
static inline int dma_xilinx_axi_dma_lock_irq(const struct dma_xilinx_axi_dma_config *cfg,
					      const uint32_t channel_num)
{
	(void)cfg;
	(void)channel_num;
	return irq_lock();
}

static inline void dma_xilinx_axi_dma_unlock_irq(const struct dma_xilinx_axi_dma_config *cfg,
						 const uint32_t channel_num, int key)
{
	(void)cfg;
	(void)channel_num;
	return irq_unlock(key);
}
#elif defined(CONFIG_DMA_XILINX_AXI_DMA_LOCK_DMA_IRQS)
static inline int dma_xilinx_axi_dma_lock_irq(const struct dma_xilinx_axi_dma_config *cfg,
					      const uint32_t channel_num)
{
	int ret;
	(void)channel_num;

	/* TX is 0, RX is 1 */
	ret = irq_is_enabled(cfg->irq0_channels[0]) ? 1 : 0;
	ret |= (irq_is_enabled(cfg->irq0_channels[1]) ? 1 : 0) << 1;

	LOG_DBG("DMA IRQ state: %x TX IRQN: %" PRIu32 " RX IRQN: %" PRIu32, ret,
		cfg->irq0_channels[0], cfg->irq0_channels[1]);

	irq_disable(cfg->irq0_channels[0]);
	irq_disable(cfg->irq0_channels[1]);

	return ret;
}

static inline void dma_xilinx_axi_dma_unlock_irq(const struct dma_xilinx_axi_dma_config *cfg,
						 const uint32_t channel_num, int key)
{
	(void)channel_num;

	if (key & 0x1) {
		/* TX was enabled */
		irq_enable(cfg->irq0_channels[0]);
	}
	if (key & 0x2) {
		/* RX was enabled */
		irq_enable(cfg->irq0_channels[1]);
	}
}
#elif defined(CONFIG_DMA_XILINX_AXI_DMA_LOCK_CHANNEL_IRQ)
static inline int dma_xilinx_axi_dma_lock_irq(const struct dma_xilinx_axi_dma_config *cfg,
					      const uint32_t channel_num)
{
	int ret;

	ret = irq_is_enabled(cfg->irq0_channels[channel_num]);

	LOG_DBG("DMA IRQ state: %x ", ret);

	irq_disable(cfg->irq0_channels[channel_num]);

	return ret;
}

static inline void dma_xilinx_axi_dma_unlock_irq(const struct dma_xilinx_axi_dma_config *cfg,
						 const uint32_t channel_num, int key)
{
	if (key) {
		/* was enabled */
		irq_enable(cfg->irq0_channels[channel_num]);
	}
}
#else
#error "No IRQ strategy selected in Kconfig!"
#endif

static inline void dma_xilinx_axi_dma_write_reg(volatile uint32_t *reg, uint32_t val)
{
	sys_write32(val, (mem_addr_t)(uintptr_t)reg);
}

static inline uint32_t dma_xilinx_axi_dma_read_reg(volatile uint32_t *reg)
{
	return sys_read32((mem_addr_t)(uintptr_t)reg);
}

uint32_t dma_xilinx_axi_dma_last_received_frame_length(const struct device *dev)
{
	const struct dma_xilinx_axi_dma_data *data = dev->data;

	return data->channels[XILINX_AXI_DMA_RX_CHANNEL_NUM].last_rx_size;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
static inline void
dma_xilinx_axi_dma_acknowledge_interrupt(struct dma_xilinx_axi_dma_channel *channel_data)
{
	/* interrupt handler might have called dma_start */
	/* this overwrites the DMA control register */
	/* so we cannot just write the old value back */
	uint32_t dmacr = dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmacr);

	dma_xilinx_axi_dma_write_reg(&channel_data->channel_regs->dmacr, dmacr);
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
static bool dma_xilinx_axi_dma_channel_has_error(
	const struct dma_xilinx_axi_dma_channel *channel_data,
	volatile const struct dma_xilinx_axi_dma_sg_descriptor *descriptor)
{
	bool error = false;

	/* check register errors first, as the SG descriptor might not be valid */
	if (dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
	    XILINX_AXI_DMA_REGS_DMASR_INTERR) {
		LOG_ERR("DMA has internal error, DMASR = %" PRIx32,
			dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr));
		error = true;
	}

	if (dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
	    XILINX_AXI_DMA_REGS_DMASR_SLVERR) {
		LOG_ERR("DMA has slave error, DMASR = %" PRIx32,
			dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr));
		error = true;
	}

	if (dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
	    XILINX_AXI_DMA_REGS_DMASR_DMADECERR) {
		LOG_ERR("DMA has decode error, DMASR = %" PRIx32,
			dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr));
		error = true;
	}

	if (dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
	    XILINX_AXI_DMA_REGS_DMASR_SGINTERR) {
		LOG_ERR("DMA has SG internal error, DMASR = %" PRIx32,
			dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr));
		error = true;
	}

	if (dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
	    XILINX_AXI_DMA_REGS_DMASR_SGSLVERR) {
		LOG_ERR("DMA has SG slave error, DMASR = %" PRIx32,
			dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr));
		error = true;
	}

	if (dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
	    XILINX_AXI_DMA_REGS_DMASR_SGDECERR) {
		LOG_ERR("DMA has SG decode error, DMASR = %" PRIx32,
			dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr));
		error = true;
	}

	if (descriptor->status & XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_DEC_ERR_MASK) {
		LOG_ERR("Descriptor has SG decode error, status=%" PRIx32, descriptor->status);
		error = true;
	}

	if (descriptor->status & XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_SLV_ERR_MASK) {
		LOG_ERR("Descriptor has SG slave error, status=%" PRIx32, descriptor->status);
		error = true;
	}

	if (descriptor->status & XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_INT_ERR_MASK) {
		LOG_ERR("Descriptor has SG internal error, status=%" PRIx32, descriptor->status);
		error = true;
	}

	return error;
}
#pragma GCC diagnostic pop

static int dma_xilinx_axi_dma_clean_up_sg_descriptors(
	struct device *dev, struct dma_xilinx_axi_dma_channel *channel_data, const char *chan_name)
{
	volatile struct dma_xilinx_axi_dma_sg_descriptor *current_descriptor =
		&channel_data->descriptors[channel_data->current_transfer_end_index];
	unsigned int processed_packets = 0;

	while (current_descriptor->status & XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_COMPLETE_MASK ||
	       current_descriptor->status & ~XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_TRANSFERRED_MASK) {
		/* descriptor completed or errored out - need to call callback */
		int retval = DMA_STATUS_COMPLETE;

		/* this is meaningless / ignored for TX channel */
		channel_data->last_rx_size = current_descriptor->status &
					     XILINX_AXI_DMA_SG_DESCRIPTOR_STATUS_LENGTH_MASK;

		if (dma_xilinx_axi_dma_channel_has_error(channel_data, current_descriptor)) {
			LOG_ERR("Channel / descriptor error on %s chan!", chan_name);
			retval = -EFAULT;
		}

		if (channel_data->check_csum_in_isr) {
			uint32_t checksum_status = current_descriptor->app2;

			if (checksum_status & XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_FCS_ERR_MASK) {
				LOG_ERR("Checksum offloading has FCS error status %" PRIx32 "!",
					checksum_status);
				retval = -EFAULT;
			}

			if ((checksum_status & XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_IP_ERR_MASK) ==
			    XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_IP_ERR_MASK) {
				LOG_ERR("Checksum offloading has IP error status %" PRIx32 "!",
					checksum_status);
				retval = -EFAULT;
			}

			if ((checksum_status & XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_UDP_ERR_MASK) ==
			    XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_UDP_ERR_MASK) {
				LOG_ERR("Checksum offloading has UDP error status %" PRIx32 "!",
					checksum_status);
				retval = -EFAULT;
			}

			if ((checksum_status & XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_TCP_ERR_MASK) ==
			    XILINX_AXI_DMA_SG_DESCRIPTOR_APP2_TCP_ERR_MASK) {
				LOG_ERR("Checksum offloading has TCP error status %" PRIx32 "!",
					checksum_status);
				retval = -EFAULT;
			}
			/* FIXME in some corner cases, the hardware cannot check the checksum */
			/* in this case, we cannot let the Zephyr network stack know, */
			/* as we do not have per-skb flags for checksum status */
		}

		/* clears the flags such that the DMA does not transfer it twice or errors */
		current_descriptor->control = current_descriptor->status = 0;

		/* callback might start new transfer */
		/* hence, the transfer end needs to be updated */
		channel_data->current_transfer_end_index++;
		if (channel_data->current_transfer_end_index >= channel_data->num_descriptors) {
			channel_data->current_transfer_end_index = 0;
		}

		if (channel_data->completion_callback) {
			LOG_DBG("Received packet with %u bytes!", channel_data->last_rx_size);
			channel_data->completion_callback(
				dev, channel_data->completion_callback_user_data,
				XILINX_AXI_DMA_TX_CHANNEL_NUM, retval);
		}

		current_descriptor =
			&channel_data->descriptors[channel_data->current_transfer_end_index];
		processed_packets++;
	}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
	/* this clears the IRQ */
	/* FIXME write the same value back... */
	dma_xilinx_axi_dma_write_reg(&channel_data->channel_regs->dmasr, 0xffffffff);
#pragma GCC diagnostic pop

	/* writes must commit before returning from ISR */
	barrier_dmem_fence_full();

	return processed_packets;
}

static void dma_xilinx_axi_dma_tx_isr(struct device *dev)
{
	struct dma_xilinx_axi_dma_data *data = dev->data;
	struct dma_xilinx_axi_dma_channel *channel_data =
		&data->channels[XILINX_AXI_DMA_TX_CHANNEL_NUM];
	int processed_packets;

	dma_xilinx_axi_dma_disable_cache();

	processed_packets = dma_xilinx_axi_dma_clean_up_sg_descriptors(dev, channel_data, "TX");

	dma_xilinx_axi_dma_enable_cache();

	LOG_DBG("Received %u RX packets in this ISR!\n", processed_packets);

	dma_xilinx_axi_dma_acknowledge_interrupt(channel_data);
}

static void dma_xilinx_axi_dma_rx_isr(struct device *dev)
{
	struct dma_xilinx_axi_dma_data *data = dev->data;
	struct dma_xilinx_axi_dma_channel *channel_data =
		&data->channels[XILINX_AXI_DMA_RX_CHANNEL_NUM];
	int processed_packets;

	dma_xilinx_axi_dma_disable_cache();

	processed_packets = dma_xilinx_axi_dma_clean_up_sg_descriptors(dev, channel_data, "RX");

	dma_xilinx_axi_dma_enable_cache();

	LOG_DBG("Cleaned up %u TX packets in this ISR!\n", processed_packets);

	dma_xilinx_axi_dma_acknowledge_interrupt(channel_data);
}

#ifdef CONFIG_DMA_64BIT
typedef uint64_t dma_addr_t;
#else
typedef uint32_t dma_addr_t;
#endif

static int dma_xilinx_axi_dma_start(const struct device *dev, uint32_t channel)
{
	const struct dma_xilinx_axi_dma_config *cfg = dev->config;
	struct dma_xilinx_axi_dma_data *data = dev->data;
	struct dma_xilinx_axi_dma_channel *channel_data = &data->channels[channel];
	volatile struct dma_xilinx_axi_dma_sg_descriptor *current_descriptor;
	volatile struct dma_xilinx_axi_dma_sg_descriptor *first_unprocessed_descriptor;
	size_t tail_descriptor;

	bool halted = false;

	/* running ISR in parallel could cause issues with the metadata */
	const int irq_key = dma_xilinx_axi_dma_lock_irq(cfg, channel);

	if (channel >= cfg->channels) {
		LOG_ERR("Invalid channel %" PRIu32 " - must be < %" PRIu32 "!", channel,
			cfg->channels);
		dma_xilinx_axi_dma_unlock_irq(cfg, channel, irq_key);
		return -EINVAL;
	}

	tail_descriptor = channel_data->current_transfer_start_index++;

	if (channel_data->current_transfer_start_index >= channel_data->num_descriptors) {
		LOG_DBG("Wrapping tail descriptor on %s chan!",
			channel == XILINX_AXI_DMA_TX_CHANNEL_NUM ? "TX" : "RX");
		channel_data->current_transfer_start_index = 0;
	}

	dma_xilinx_axi_dma_disable_cache();
	current_descriptor = &channel_data->descriptors[tail_descriptor];
	first_unprocessed_descriptor =
		&channel_data->descriptors[channel_data->current_transfer_end_index];

	LOG_DBG("Starting DMA on %s channel with tail ptr %zu start ptr %zu",
		channel == XILINX_AXI_DMA_TX_CHANNEL_NUM ? "TX" : "RX", tail_descriptor,
		channel_data->current_transfer_end_index);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
	if (dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
	    XILINX_AXI_DMA_REGS_DMASR_HALTED) {

		halted = true;

		LOG_DBG("AXI DMA is halted - restart operation!");

#ifdef CONFIG_DMA_64BIT
		dma_xilinx_axi_dma_write_reg(
			&channel_data->channel_regs->curdesc,
			(uint32_t)(((uintptr_t)first_unprocessed_descriptor) & 0xffffffff));
		dma_xilinx_axi_dma_write_reg(
			&channel_data->channel_regs->curdesc_msb,
			(uint32_t)(((uintptr_t)first_unprocessed_descriptor) >> 32));
#else
		dma_xilinx_axi_dma_write_reg(&channel_data->channel_regs->curdesc,
					     (uint32_t)(uintptr_t)first_unprocessed_descriptor);
#endif
	}
#pragma GCC diagnostic pop

	/* current descriptor MUST be set before tail descriptor */
	barrier_dmem_fence_full();

	if (halted) {
		uint32_t new_control = 0;

		new_control |= XILINX_AXI_DMA_REGS_DMACR_RS;
		/* no reset */
		new_control &= ~XILINX_AXI_DMA_REGS_DMACR_RESET;
		/* FIXME make this a DT parameter */
		/* for Eth DMA, this should never be used */
		new_control &= ~XILINX_AXI_DMA_REGS_DMACR_KEYHOLE;
		/* no cyclic mode - we use completed bit to control which */
		/* transfers where completed */
		new_control &= ~XILINX_AXI_DMA_REGS_DMACR_CYC_BD_EN;
		/* we want interrupts on complete */
		new_control |= XILINX_AXI_DMA_REGS_DMACR_IOC_IRQEN;
		/* we do NOT want timeout IRQs */
		new_control &= ~XILINX_AXI_DMA_REGS_DMACR_DLY_IRQEN;
		/* we want IRQs on error */
		new_control |= XILINX_AXI_DMA_REGS_DMACR_ERR_IRQEN;
		/* interrupt after every completed transfer */
		new_control |= 0x01 << XILINX_AXI_DMA_REGS_DMACR_IRQTHRESH_SHIFT_BITS;
		/* no timeout */
		new_control |= 0x0 << XILINX_AXI_DMA_REGS_DMACR_IRQDELAY_SHIFT_BITS;

		LOG_DBG("New DMACR value: %" PRIx32, new_control);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
		dma_xilinx_axi_dma_write_reg(&channel_data->channel_regs->dmacr, new_control);
		/* need to make sure start was committed before writing tail */
		barrier_dmem_fence_full();
	}

#ifdef CONFIG_DMA_64BIT
	dma_xilinx_axi_dma_write_reg(&channel_data->channel_regs->taildesc,
				     (uint32_t)(((uintptr_t)current_descriptor) & 0xffffffff));
	dma_xilinx_axi_dma_write_reg(&channel_data->channel_regs->taildesc_msb,
				     (uint32_t)(((uintptr_t)current_descriptor) >> 32));
#else
	dma_xilinx_axi_dma_write_reg(&channel_data->channel_regs->taildesc,
				     (uint32_t)(uintptr_t)current_descriptor);
#endif
#pragma GCC diagnostic pop

	dma_xilinx_axi_dma_enable_cache();

	dma_xilinx_axi_dma_unlock_irq(cfg, channel, irq_key);

	/* commit stores before returning to caller */
	barrier_dmem_fence_full();

	return 0;
}

static int dma_xilinx_axi_dma_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_xilinx_axi_dma_config *cfg = dev->config;
	struct dma_xilinx_axi_dma_data *data = dev->data;
	struct dma_xilinx_axi_dma_channel *channel_data = &data->channels[channel];

	uint32_t new_control;

	if (channel >= cfg->channels) {
		LOG_ERR("Invalid channel %" PRIu32 " - must be < %" PRIu32 "!", channel,
			cfg->channels);
		return -EINVAL;
	}

	new_control = channel_data->channel_regs->dmacr;
	/* RS = 0 --> DMA will complete ongoing transactions and then go into hold */
	new_control = new_control & ~XILINX_AXI_DMA_REGS_DMACR_RS;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
	dma_xilinx_axi_dma_write_reg(&channel_data->channel_regs->dmacr, new_control);
#pragma GCC diagnostic pop

	/* commit before returning to caller */
	barrier_dmem_fence_full();

	return 0;
}

static int dma_xilinx_axi_dma_get_status(const struct device *dev, uint32_t channel,
					 struct dma_status *stat)
{
	const struct dma_xilinx_axi_dma_config *cfg = dev->config;
	struct dma_xilinx_axi_dma_data *data = dev->data;
	struct dma_xilinx_axi_dma_channel *channel_data = &data->channels[channel];

	if (channel >= cfg->channels) {
		LOG_ERR("Invalid channel %" PRIu32 " - must be < %" PRIu32 "!", channel,
			cfg->channels);
		return -EINVAL;
	}

	memset(stat, 0, sizeof(*stat));

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
	stat->busy = !(dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
		       XILINX_AXI_DMA_REGS_DMASR_IDLE) &&
		     !(dma_xilinx_axi_dma_read_reg(&channel_data->channel_regs->dmasr) &
		       XILINX_AXI_DMA_REGS_DMASR_HALTED);
#pragma GCC diagnostic pop
	stat->dir = channel_data->last_transfer_direction;

	/* FIXME fill hardware-specific fields */

	return 0;
}
/**
 * Transfers a single buffer through the DMA
 * If is_first or is_last are NOT set, the buffer is considered part of a SG transfer consisting of
 * multiple blocks. Otherwise, the block is one transfer.
 */
static inline int dma_xilinx_axi_dma_transfer_block(const struct dma_xilinx_axi_dma_config *cfg,
						    uint32_t channel,
						    struct dma_xilinx_axi_dma_channel *channel_data,
						    dma_addr_t buffer_addr, size_t block_size,
						    bool is_first, bool is_last)
{
	volatile struct dma_xilinx_axi_dma_sg_descriptor *current_descriptor;

	/* running ISR in parallel could cause issues with the metadata */
	const int irq_key = dma_xilinx_axi_dma_lock_irq(cfg, channel);

	current_descriptor = &channel_data->descriptors[channel_data->current_transfer_start_index];

	dma_xilinx_axi_dma_disable_cache();

#ifdef CONFIG_DMA_64BIT
	current_descriptor->buffer_address = (uint32_t)buffer_addr & 0xffffffff;
	current_descriptor->buffer_address_msb = (uint32_t)(buffer_addr >> 32);
#else
	current_descriptor->buffer_address = buffer_addr;
#endif
	current_descriptor->app0 = channel_data->sg_desc_app0;

	if (block_size > UINT32_MAX) {
		LOG_ERR("Too large block: %zu bytes!", block_size);

		dma_xilinx_axi_dma_enable_cache();
		dma_xilinx_axi_dma_unlock_irq(cfg, channel, irq_key);

		return -EINVAL;
	}
	/* clears the start of frame / end of frame flags as well */
	current_descriptor->control = (uint32_t)block_size;

	if (is_first) {
		current_descriptor->control =
			current_descriptor->control | XILINX_AXI_DMA_SG_DESCRIPTOR_CTRL_SOF_MASK;
	}
	if (is_last) {
		current_descriptor->control =
			current_descriptor->control | XILINX_AXI_DMA_SG_DESCRIPTOR_CTRL_EOF_MASK;
	}

	/* SG descriptor must be completed BEFORE hardware is made aware of it */
	barrier_dmem_fence_full();

	dma_xilinx_axi_dma_enable_cache();

	dma_xilinx_axi_dma_unlock_irq(cfg, channel, irq_key);

	return 0;
}

#ifdef CONFIG_DMA_64BIT
static inline int dma_xilinx_axi_dma_config_reload(const struct device *dev, uint32_t channel,
						   uint64_t src, uint64_t dst, size_t size)
#else
static inline int dma_xilinx_axi_dma_config_reload(const struct device *dev, uint32_t channel,
						   uint32_t src, uint32_t dst, size_t size)
#endif
{
	const struct dma_xilinx_axi_dma_config *cfg = dev->config;
	struct dma_xilinx_axi_dma_data *data = dev->data;
	struct dma_xilinx_axi_dma_channel *channel_data = &data->channels[channel];

	if (channel >= cfg->channels) {
		LOG_ERR("Invalid channel %" PRIu32 " - must be < %" PRIu32 "!", channel,
			cfg->channels);
		return -EINVAL;
	}
	/* one-block-at-a-time transfer */
	return dma_xilinx_axi_dma_transfer_block(
		cfg, channel, channel_data, channel == XILINX_AXI_DMA_TX_CHANNEL_NUM ? src : dst,
		size, true, true);
}

static int dma_xilinx_axi_dma_configure(const struct device *dev, uint32_t channel,
					struct dma_config *dma_cfg)
{
	const struct dma_xilinx_axi_dma_config *cfg = dev->config;
	struct dma_xilinx_axi_dma_data *data = dev->data;
	struct dma_block_config *current_block = dma_cfg->head_block;
	int ret = 0;
	int block_count = 0;

	struct dma_xilinx_axi_dma_register_space *regs =
		(struct dma_xilinx_axi_dma_register_space *)cfg->reg;

	if (channel >= cfg->channels) {
		LOG_ERR("Invalid channel %" PRIu32 " - must be < %" PRIu32 "!", channel,
			cfg->channels);
		return -EINVAL;
	}

	if (cfg->channels != XILINX_AXI_DMA_NUM_CHANNELS) {
		LOG_ERR("Invalid number of configured channels (%" PRIu32
			") - Xilinx AXI DMA must have %" PRIu32 " channels!",
			cfg->channels, XILINX_AXI_DMA_NUM_CHANNELS);
		return -EINVAL;
	}

	if (dma_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		LOG_ERR("Xilinx AXI DMA only supports incrementing addresses!");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		LOG_ERR("Xilinx AXI DMA only supports incrementing addresses!");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid source_addr_adj %" PRIu16, dma_cfg->head_block->source_addr_adj);
		return -ENOTSUP;
	}
	if (dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid dest_addr_adj %" PRIu16, dma_cfg->head_block->dest_addr_adj);
		return -ENOTSUP;
	}

	if (channel == XILINX_AXI_DMA_TX_CHANNEL_NUM &&
	    dma_cfg->channel_direction != MEMORY_TO_PERIPHERAL) {
		LOG_ERR("TX channel must be used with MEMORY_TO_PERIPHERAL!");
		return -ENOTSUP;
	}

	if (channel == XILINX_AXI_DMA_RX_CHANNEL_NUM &&
	    dma_cfg->channel_direction != PERIPHERAL_TO_MEMORY) {
		LOG_ERR("RX channel must be used with PERIPHERAL_TO_MEMORY!");
		return -ENOTSUP;
	}

	data->channels[channel].last_transfer_direction = dma_cfg->channel_direction;

	dma_xilinx_axi_dma_disable_cache();

	if (channel == XILINX_AXI_DMA_TX_CHANNEL_NUM) {
		data->channels[channel].descriptors = descriptors_tx;
		data->channels[channel].num_descriptors = ARRAY_SIZE(descriptors_tx);

		data->channels[channel].channel_regs = &regs->mm2s_registers;
	} else {
		data->channels[channel].descriptors = descriptors_rx;
		data->channels[channel].num_descriptors = ARRAY_SIZE(descriptors_rx);

		data->channels[channel].channel_regs = &regs->s2mm_registers;
	}

	LOG_DBG("Resetting DMA channel!");

	if (!data->device_has_been_reset) {
		LOG_INF("Soft-resetting the DMA core!");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
		/* this resets BOTH RX and TX channels, although it is triggered in per-channel
		 * DMACR
		 */
		dma_xilinx_axi_dma_write_reg(&data->channels[channel].channel_regs->dmacr,
					     XILINX_AXI_DMA_REGS_DMACR_RESET);
#pragma GCC diagnostic pop
		data->device_has_been_reset = true;
	}

	LOG_DBG("Configuring %zu DMA descriptors for %s", data->channels[channel].num_descriptors,
		channel == XILINX_AXI_DMA_TX_CHANNEL_NUM ? "TX" : "RX");

	/* only configures fields whos default is not 0, as descriptors are in zero-initialized */
	/* segment */
	data->channels[channel].current_transfer_start_index =
		data->channels[channel].current_transfer_end_index = 0;
	for (int i = 0; i < data->channels[channel].num_descriptors; i++) {
		uintptr_t nextdesc;
		uint32_t low_bytes;
#ifdef CONFIG_DMA_64BIT
		uint32_t high_bytes;
#endif
		if (i + 1 < data->channels[channel].num_descriptors) {
			nextdesc = (uintptr_t)&data->channels[channel].descriptors[i + 1];
		} else {
			nextdesc = (uintptr_t)&data->channels[channel].descriptors[0];
		}
		/* SG descriptors have 64-byte alignment requirements */
		/* we check this here, for each descriptor */
		__ASSERT(
			nextdesc & XILINX_AXI_DMA_SG_DESCRIPTOR_ADDRESS_MASK == 0,
			"SG descriptor address %p (offset %u) was not aligned to 64-byte boundary!",
			(void *)nextdesc, i);

		low_bytes = (uint32_t)(((uint64_t)nextdesc) & 0xffffffff);
		data->channels[channel].descriptors[i].nxtdesc = low_bytes;

#ifdef CONFIG_DMA_64BIT
		high_bytes = (uint32_t)(((uint64_t)nextdesc >> 32) & 0xffffffff);
		data->channels[channel].descriptors[i].nxtdesc_msb = high_bytes;
#endif
	}

	dma_xilinx_axi_dma_enable_cache();

	data->channels[channel].check_csum_in_isr = false;

	/* the DMA passes the app fields through to the AXIStream-connected device */
	/* whether the connected device understands these flags needs to be determined by the */
	/* caller! */
	switch (dma_cfg->linked_channel) {
	case XILINX_AXI_DMA_LINKED_CHANNEL_FULL_CSUM_OFFLOAD:
		if (channel == XILINX_AXI_DMA_TX_CHANNEL_NUM) {
			/* for the TX channel, we need to indicate that we would like to use */
			/* checksum offloading */
			data->channels[channel].sg_desc_app0 =
				XILINX_AXI_DMA_SG_DESCRIPTOR_APP0_CHECKSUM_OFFLOAD_FULL;
		} else {
			/* for the RX channel, the Ethernet core will indicate to us that it has */
			/* computed a checksum and whether it is valid we need to check this in */
			/* the ISR and report it upstream */
			data->channels[channel].check_csum_in_isr = true;
		}
		break;
	case XILINX_AXI_DMA_LINKED_CHANNEL_NO_CSUM_OFFLOAD:
		data->channels[channel].sg_desc_app0 =
			XILINX_AXI_DMA_SG_DESCRIPTOR_APP0_CHECKSUM_OFFLOAD_NONE;
		break;
	default:
		LOG_ERR("Linked channel invalid! Valid values: %u for full ethernt checksum "
			"offloading %u for no checksum offloading!",
			XILINX_AXI_DMA_LINKED_CHANNEL_FULL_CSUM_OFFLOAD,
			XILINX_AXI_DMA_LINKED_CHANNEL_NO_CSUM_OFFLOAD);
		return -EINVAL;
	}

	data->channels[channel].completion_callback = dma_cfg->dma_callback;
	data->channels[channel].completion_callback_user_data = dma_cfg->user_data;

	LOG_INF("Completed configuration of AXI DMA - Starting transfer!");

	do {
		ret = ret ||
		      dma_xilinx_axi_dma_transfer_block(cfg, channel, &data->channels[channel],
							channel == XILINX_AXI_DMA_TX_CHANNEL_NUM
								? current_block->source_address
								: current_block->dest_address,
							current_block->block_size, block_count == 0,
							current_block->next_block == NULL);
		block_count++;
	} while ((current_block = current_block->next_block) && ret == 0);

	return ret;
}

static bool dma_xilinx_axi_dma_chan_filter(const struct device *dev, int channel,
					   void *filter_param)
{
	const char *filter_str = (const char *)filter_param;

	if (strcmp(filter_str, "tx") == 0) {
		return channel == XILINX_AXI_DMA_TX_CHANNEL_NUM;
	}
	if (strcmp(filter_str, "rx") == 0) {
		return channel == XILINX_AXI_DMA_RX_CHANNEL_NUM;
	}

	return false;
}

/* DMA API callbacks */
static const struct dma_driver_api dma_xilinx_axi_dma_driver_api = {
	.config = dma_xilinx_axi_dma_configure,
	.reload = dma_xilinx_axi_dma_config_reload,
	.start = dma_xilinx_axi_dma_start,
	.stop = dma_xilinx_axi_dma_stop,
	.suspend = NULL,
	.resume = NULL,
	.get_status = dma_xilinx_axi_dma_get_status,
	.chan_filter = dma_xilinx_axi_dma_chan_filter,
};

static int dma_xilinx_axi_dma_init(const struct device *dev)
{
	const struct dma_xilinx_axi_dma_config *cfg = dev->config;

	cfg->irq_configure();
	return 0;
}

/* first IRQ is TX */
#define TX_IRQ_CONFIGURE(inst)                                                                     \
	IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, 0), DT_INST_IRQ_BY_IDX(inst, 0, priority),           \
		    dma_xilinx_axi_dma_tx_isr, DEVICE_DT_INST_GET(inst), 0);                       \
	irq_enable(DT_INST_IRQN_BY_IDX(inst, 0));
/* second IRQ is RX */
#define RX_IRQ_CONFIGURE(inst)                                                                     \
	IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, 1), DT_INST_IRQ_BY_IDX(inst, 1, priority),           \
		    dma_xilinx_axi_dma_rx_isr, DEVICE_DT_INST_GET(inst), 0);                       \
	irq_enable(DT_INST_IRQN_BY_IDX(inst, 1));

#define CONFIGURE_ALL_IRQS(inst)                                                                   \
	TX_IRQ_CONFIGURE(inst);                                                                    \
	RX_IRQ_CONFIGURE(inst);

#define XILINX_AXI_DMA_INIT(inst)                                                                  \
	static void dma_xilinx_axi_dma##inst##_irq_configure(void)                                 \
	{                                                                                          \
		CONFIGURE_ALL_IRQS(inst);                                                          \
	}                                                                                          \
	static uint32_t dma_xilinx_axi_dma##inst##_irq0_channels[] =                               \
		DT_INST_PROP_OR(inst, interrupts, {0});                                            \
	static const struct dma_xilinx_axi_dma_config dma_xilinx_axi_dma##inst##_config = {        \
		.reg = (void *)(uintptr_t)DT_INST_REG_ADDR(inst),                                  \
		.channels = DT_INST_PROP(inst, dma_channels),                                      \
		.irq_configure = dma_xilinx_axi_dma##inst##_irq_configure,                         \
		.irq0_channels = dma_xilinx_axi_dma##inst##_irq0_channels,                         \
		.irq0_channels_size = ARRAY_SIZE(dma_xilinx_axi_dma##inst##_irq0_channels),        \
	};                                                                                         \
	static struct dma_xilinx_axi_dma_channel                                                   \
		dma_xilinx_axi_dma##inst##_channels[DT_INST_PROP(inst, dma_channels)];             \
	ATOMIC_DEFINE(dma_xilinx_axi_dma_atomic##inst, DT_INST_PROP(inst, dma_channels));          \
	static struct dma_xilinx_axi_dma_data dma_xilinx_axi_dma##inst##_data = {                  \
		.ctx = {.magic = DMA_MAGIC, .atomic = NULL},                                       \
		.channels = dma_xilinx_axi_dma##inst##_channels,                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &dma_xilinx_axi_dma_init, NULL,                                \
			      &dma_xilinx_axi_dma##inst##_data,                                    \
			      &dma_xilinx_axi_dma##inst##_config, POST_KERNEL,                     \
			      CONFIG_DMA_INIT_PRIORITY, &dma_xilinx_axi_dma_driver_api);

/* two different compatibles match the very same Xilinx AXI DMA, */
/* depending on if it is used in the AXI Ethernet subsystem or not */
#define DT_DRV_COMPAT xlnx_eth_dma
DT_INST_FOREACH_STATUS_OKAY(XILINX_AXI_DMA_INIT)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT xlnx_axi_dma_1_00_a
DT_INST_FOREACH_STATUS_OKAY(XILINX_AXI_DMA_INIT)
