/* NOTE: Definitions used between ULL and LLL implementations */

struct tmp_tx {
	u16_t handle;
	void *node;
};

struct tmp_node_tx {
	union {
		void        *next;
		void        *pool;
		memq_link_t *link;
	};

	u8_t data[];
};

struct lll_tmp {
	struct lll_hdr hdr;

	MEMQ_DECLARE(tx);
	memq_link_t _link;      /* Dedicated thread allocatable */
	memq_link_t *link_free; /* Thread allocatable reference */
};

int lll_tmp_init(void);
void lll_tmp_prepare(void *param);

u8_t lll_tmp_ack_last_idx_get(void);
memq_link_t *lll_tmp_ack_peek(u16_t *handle, struct tmp_node_tx **node_tx);
memq_link_t *lll_tmp_ack_by_last_peek(u8_t last, u16_t *handle,
				      struct tmp_node_tx **node_tx);
void *lll_tmp_ack_dequeue(void);

extern u16_t ull_tmp_handle_get(struct lll_tmp *tmp);
