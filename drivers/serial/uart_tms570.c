
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tms570_uart);

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#endif

#define DT_DRV_COMPAT tms570_uart

#define CGR0_OFFSET     (0x00)
#define CGR1_OFFSET     (0x04)
#define SETINT_OFFSET   (0x0c)
#define CLEARINT_OFFSET (0x10)
#define FLAGS_OFFSET    (0x1c)
#define BRS_OFFSET      (0x2c)
#define FORMAT_OFFSET   (0x28)
#define RDBUF_OFFSET    (0x34)
#define TDBUF_OFFSET    (0x38)
#define PIO0_OFFSET     (0x3c)

/* SCIGCR0 bits */
#define RESET_BIT (1 << 0)

/* SCIGCR1 bits */
#define CLOCK_BIT    (1 << 5)
#define SWnRST_BIT   (1 << 7)
#define LOOPBACK_BIT (1 << 16)
#define CONT_BIT     (1 << 17)
#define RXENA_BIT    (1 << 24)
#define TXENA_BIT    (1 << 25)
#define ASYNC_BIT    (1 << 1)

/* SCIPIO0 bits */
#define RXFUNC_BIT (1 << 1)
#define TXFUNC_BIT (1 << 2)

/* Flags/FLR bits */
#define TXRDY_BIT   (1 << 8)
#define RXRDY_BIT   (1 << 9)
#define TXEMPTY_BIT (1 << 11)

/* SETINT/CLEARINT bits */
#define TXINT_BIT   (1 << 8)
#define RXINT_BIT   (1 << 9)
#define TXDMA_BIT   (1 << 16)
#define RXDMA_BITS  (1 << 17 | 1 << 18)
/* Framing error interrupt, overrun error interrupt and parity error interrupt */
#define ERRINT_BITS ((1 << 26) | (1 << 25) | (1 << 24))

#define FRAME7_BITS (0x7)

struct uart_tms570_cfg {
        DEVICE_MMIO_ROM;
        uint32_t baud;

        const struct pinctrl_dev_config *pincfg;
        const struct device *clk_ctrl;
        unsigned int clk_domain;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        void (*irq_connect)(const struct device *);
#endif

#ifdef CONFIG_UART_ASYNC_API
        const struct device *dma_dev;
        int dma_request_rx;
        int dma_request_tx;
        int dma_channel_rx;
        int dma_channel_tx;
#endif
};

struct uart_tms570_data {
        DEVICE_MMIO_RAM;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        uart_irq_callback_user_data_t cb;
        void *cb_data;
#endif

#ifdef CONFIG_UART_ASYNC_API
        uart_callback_t async_cb;
        void *async_data;

        const struct device *dev;

        struct {
                uint8_t *buf;
                size_t buf_size;
                size_t offset;

                uint8_t *next_buf;
                size_t next_buf_size;

                int32_t timeout_usec;
                struct k_work_delayable timeout_work;
                struct k_work_delayable done_work;

                struct dma_block_config dma_blk;

                struct dma_config dma_cfg;
        } dma_rx, dma_tx;
#endif
};

/**
 * @brief Calculate the value for the BAUD register, based on the nominal baud
 * rate @p cfg->baud
 *
 * @param cfg
 * @return uint32_t Converted value, which can be set in the BAUD register
 */
static uint32_t calc_baud_reg(const struct uart_tms570_cfg *cfg)
{
        int ret;
        uint32_t rate;
        uint32_t prescaler;
        uint32_t div;

        ret = clock_control_get_rate(cfg->clk_ctrl, (clock_control_subsys_t)&cfg->clk_domain,
                                     &rate);
        if (ret < 0) {
                __ASSERT(0, "clock control get rate failed");
                return 0;
        }

        prescaler = rate / (cfg->baud * 16) - 1;
        div = rate / (cfg->baud) - 16 * (1 + prescaler);

        return ((div & 0xf) << 24) | prescaler;
}

static inline int is_ready(uintptr_t reg_base, uint32_t mask)
{
        return sys_read32(reg_base + FLAGS_OFFSET) & mask;
}

/**
 * @brief Receive data from the UART peripheral, using the polling method
 *
 * @param dev
 * @param c Buffer where incoming character is stored
 * @return int Always 0
 */
static int uart_tms570_poll_in(const struct device *dev, unsigned char *c)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);

        while (!is_ready(reg_base, RXRDY_BIT)) {
        }

        *c = (unsigned char)sys_read32(reg_base + RDBUF_OFFSET);

        return 0;
}

/**
 * @brief Send data to the UART peripheral, using the polling method
 *
 * @param dev
 * @param c Character to send
 */
static void uart_tms570_poll_out(const struct device *dev, unsigned char c)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);

        while (!is_ready(reg_base, TXRDY_BIT)) {
        }

        sys_write32(c, reg_base + TDBUF_OFFSET);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_tms570_fifo_fill(const struct device *dev, const uint8_t *data, int size)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);
        int i = 0;

        for (; i < size && is_ready(reg_base, TXRDY_BIT); i++) {
                sys_write32(data[i], reg_base + TDBUF_OFFSET);
        }

        return i;
}

static int uart_tms570_fifo_read(const struct device *dev, uint8_t *buf, const int size)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);
        int i = 0;

        for (; i < size && is_ready(reg_base, RXRDY_BIT); i++) {
                buf[i] = sys_read32(reg_base + RDBUF_OFFSET);
        }

        return i;
}

static void uart_tms570_irq_tx_enable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + SETINT_OFFSET, TXINT_BIT);
}

static void uart_tms570_irq_tx_disable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + CLEARINT_OFFSET, TXINT_BIT);
}

static int uart_tms570_irq_tx_ready(const struct device *dev)
{
        return is_ready(DEVICE_MMIO_GET(dev), TXRDY_BIT);
}

static int uart_tms570_irq_tx_complete(const struct device *dev)
{
        return sys_read32(DEVICE_MMIO_GET(dev) + FLAGS_OFFSET) & TXEMPTY_BIT;
}

static void uart_tms570_irq_rx_enable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + SETINT_OFFSET, RXINT_BIT);
}

static void uart_tms570_irq_rx_disable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + CLEARINT_OFFSET, RXINT_BIT);
}

static int uart_tms570_irq_rx_ready(const struct device *dev)
{
        return is_ready(DEVICE_MMIO_GET(dev), RXRDY_BIT);
}

static void uart_tms570_irq_err_enable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + SETINT_OFFSET, ERRINT_BITS);
}

static void uart_tms570_irq_err_disable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + CLEARINT_OFFSET, ERRINT_BITS);
}

static int uart_tms570_irq_is_pending(const struct device *dev)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);

        return is_ready(reg_base, TXRDY_BIT) || is_ready(reg_base, RXRDY_BIT);
}

static int uart_tms570_irq_update(const struct device *dev)
{
        ARG_UNUSED(dev);
        return 1;
}

static void uart_tms570_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
                                         void *user_data)
{
        struct uart_tms570_data *data = dev->data;

        data->cb = cb;
        data->cb_data = user_data;
}

static void uart_tms570_isr(const struct device *dev)
{
        struct uart_tms570_data *data = dev->data;

        if (data->cb != NULL) {
                data->cb(dev, data->cb_data);
        }
}
#endif

#ifdef CONFIG_UART_ASYNC_API
static void async_event(const struct device *dev, struct uart_event *event)
{
        struct uart_tms570_data *data = dev->data;

        if (data->async_cb != NULL) {
                data->async_cb(dev, event, data->async_data);
        }
}

static void async_timer_restart(struct k_work_delayable *dwork, int32_t timeout)
{
        if (timeout == SYS_FOREVER_US || timeout == 0) {
                (void)k_work_reschedule(dwork, K_TICKS(1));
                return;
        }

        (void)k_work_reschedule(dwork, K_USEC(timeout));
}

static int uart_tms570_callback_set(const struct device *dev, uart_callback_t cb, void *user_data)
{
        struct uart_tms570_data *data = dev->data;

        data->async_cb = cb;
        data->async_data = user_data;

        return 0;
}

static int uart_tms570_rx_enable(const struct device *dev, uint8_t *buf, size_t size,
                                 int32_t timeout_usec)
{
        int status;
        struct uart_event evt;
        const struct uart_tms570_cfg *cfg = dev->config;
        struct uart_tms570_data *data = dev->data;

        if (cfg->dma_request_rx == -1 || cfg->dma_channel_rx == -1) {
                LOG_ERR("uart not configured for async use");
                return -EINVAL;
        }

        data->dma_rx.buf = buf;
        data->dma_rx.buf_size = size;
        data->dma_rx.timeout_usec = timeout_usec;

        data->dma_rx.dma_blk.dest_address = (uint32_t)buf;
        data->dma_rx.dma_blk.block_size = size;

        status = dma_config(cfg->dma_dev, cfg->dma_channel_rx, &data->dma_rx.dma_cfg);
        if (status != 0) {
                return status;
        }

        status = dma_start(cfg->dma_dev, cfg->dma_channel_rx);
        if (status != 0) {
                return status;
        }

        evt.type = UART_RX_BUF_REQUEST;
        if (data->async_cb != NULL) {
                data->async_cb(dev, &evt, data->async_data);
        }

        return status;
}

static int uart_tms570_rx_disable(const struct device *dev)
{
        int status;
        const struct uart_tms570_cfg *cfg = dev->config;
        struct uart_tms570_data *data = dev->data;
        struct dma_status dma_stat;
        struct uart_event evt;
        size_t total_len;

        status = dma_get_status(cfg->dma_dev, cfg->dma_channel_rx, &dma_stat);
        if (status != 0) {
                LOG_ERR("unable to get dma status: %i", status);
                return status;
        }

        total_len = data->dma_rx.buf_size - dma_stat.pending_length;

        evt = (struct uart_event){
                .type = UART_RX_RDY,
                .data.rx.buf = data->dma_rx.buf,
                .data.rx.offset = data->dma_rx.offset,
                .data.rx.len = total_len,
        };
        async_event(dev, &evt);

        data->dma_rx.offset = 0;

        evt = (struct uart_event){
                .type = UART_RX_BUF_RELEASED,
                .data.rx_buf.buf = data->dma_rx.buf,
        };
        async_event(dev, &evt);

        if (data->dma_rx.next_buf == NULL) {
                evt = (struct uart_event){
                        .type = UART_RX_BUF_RELEASED,
                        .data.rx_buf.buf = data->dma_rx.next_buf,
                };
                async_event(dev, &evt);
        }

        data->dma_rx.buf = NULL;
        data->dma_rx.buf_size = 0;
        data->dma_rx.next_buf = NULL;
        data->dma_rx.next_buf_size = 0;

        (void)dma_stop(cfg->dma_dev, cfg->dma_channel_rx);

        evt.type = UART_RX_DISABLED;
        async_event(dev, &evt);

        (void)k_work_cancel_delayable(&data->dma_rx.timeout_work);

        return 0;
}

static int uart_tms570_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t size)
{
        struct uart_tms570_data *data = dev->data;

        data->dma_rx.next_buf = buf;
        data->dma_rx.next_buf_size = size;

        return 0;
}

static void async_rx_timeout(struct k_work *work)
{
        int status;
        size_t total_len;
        struct dma_status dma_stat;
        struct k_work_delayable *dwork;
        const struct device *dev;
        const struct uart_tms570_cfg *cfg;
        struct uart_tms570_data *data;
        struct uart_event evt;
        unsigned int irq_key;

        dwork = k_work_delayable_from_work(work);
        data = CONTAINER_OF(dwork, struct uart_tms570_data, dma_rx.timeout_work);
        dev = data->dev;
        cfg = dev->config;

        irq_key = irq_lock();

        (void)k_work_cancel_delayable(&data->dma_rx.timeout_work);

        status = dma_get_status(cfg->dma_dev, cfg->dma_channel_rx, &dma_stat);
        if (status != 0) {
                LOG_ERR("unable to get dma status: %i", status);
                return;
        }

        total_len = data->dma_rx.buf_size - dma_stat.pending_length;

        evt = (struct uart_event){
                .type = UART_RX_RDY,
                .data.rx.buf = data->dma_rx.buf,
                .data.rx.offset = data->dma_rx.offset,
                .data.rx.len = total_len,
        };
        async_event(dev, &evt);

        data->dma_rx.offset = 0;

        evt = (struct uart_event){
                .type = UART_RX_BUF_RELEASED,
                .data.rx_buf.buf = data->dma_rx.buf,
        };
        async_event(dev, &evt);

        data->dma_rx.buf = data->dma_rx.next_buf;
        data->dma_rx.buf_size = data->dma_rx.next_buf_size;

        data->dma_rx.next_buf = NULL;
        data->dma_rx.next_buf_size = 0;

        evt.type = UART_RX_BUF_REQUEST;
        async_event(dev, &evt);

        if (data->dma_rx.buf == NULL) {
                (void)dma_stop(cfg->dma_dev, cfg->dma_channel_rx);

                evt.type = UART_RX_DISABLED;
                async_event(dev, &evt);
        } else {
                (void)dma_reload(cfg->dma_dev, cfg->dma_channel_rx, 0, (uint32_t)data->dma_rx.buf,
                                 data->dma_rx.buf_size);

                (void)dma_start(cfg->dma_dev, cfg->dma_channel_rx);
        }

        irq_unlock(irq_key);
}

static void uart_tms570_async_rx_isr(const struct device *dma, void *user_data, uint32_t channel,
                                     int status)
{
        const struct device *dev = user_data;
        struct uart_tms570_data *data = dev->data;

        /* In circular mode `DMA_STATUS_COMPLETE` indicates buffer full, whereas
         * `DMA_STATUS_BLOCK` indicates a "water mark", in this case a byte */
        if (status == DMA_STATUS_COMPLETE) {
                async_timer_restart(&data->dma_rx.timeout_work, 0);
        } else {
                async_timer_restart(&data->dma_rx.timeout_work, data->dma_rx.timeout_usec);
        }
}

static int uart_tms570_tx(const struct device *dev, const uint8_t *bytes, size_t size,
                          int32_t timeout)
{
        int status;
        uintptr_t reg_base;
        const struct uart_tms570_cfg *cfg = dev->config;
        struct uart_tms570_data *data = dev->data;
        int irq_key;

        reg_base = DEVICE_MMIO_GET(dev);

        if (cfg->dma_request_tx == -1 || cfg->dma_channel_tx == -1) {
                LOG_ERR("uart not configured for async use");
                return -EINVAL;
        }

        data->dma_tx.buf = (uint8_t *)bytes;
        data->dma_tx.buf_size = size;
        data->dma_tx.timeout_usec = timeout;
        data->dma_tx.offset = 0;
        data->dma_tx.dma_blk.block_size = data->dma_tx.buf_size;
        data->dma_tx.dma_blk.source_address = (uint32_t)data->dma_tx.buf;

        irq_key = irq_lock();

        status = dma_config(cfg->dma_dev, cfg->dma_channel_tx, &data->dma_tx.dma_cfg);
        if (status != 0) {
                LOG_ERR("unable to configure dma");
                irq_unlock(irq_key);
                return status;
        }

        async_timer_restart(&data->dma_tx.timeout_work, data->dma_tx.timeout_usec);

        status = dma_start(cfg->dma_dev, cfg->dma_channel_tx);
        if (status != 0) {
                LOG_ERR("unable to start dma");
                irq_unlock(irq_key);
                return status;
        }

        sys_clear_bits(reg_base + CGR1_OFFSET, TXENA_BIT);
        sys_set_bits(reg_base + CGR1_OFFSET, TXENA_BIT);

        irq_unlock(irq_key);
        return 0;
}

static int uart_tms570_tx_abort(const struct device *dev)
{
        int status;
        struct uart_event evt;
        struct dma_status dma_stat;
        const struct uart_tms570_cfg *cfg;
        struct uart_tms570_data *data;
        int irq_key;

        cfg = dev->config;
        data = dev->data;

        irq_key = irq_lock();

        (void)k_work_cancel_delayable(&data->dma_tx.done_work);
        (void)k_work_cancel_delayable(&data->dma_tx.timeout_work);

        status = dma_get_status(cfg->dma_dev, cfg->dma_channel_tx, &dma_stat);
        if (status != 0) {
                LOG_ERR("unable to get dma status: %i", status);
                irq_unlock(irq_key);
                return status;
        }

        (void)dma_stop(cfg->dma_dev, cfg->dma_channel_tx);

        evt = (struct uart_event){
                .type = UART_TX_ABORTED,
                .data.tx.buf = data->dma_tx.buf,
                .data.tx.len = data->dma_tx.buf_size - dma_stat.pending_length,
        };
        async_event(dev, &evt);

        irq_unlock(irq_key);
        return 0;
}

static void async_tx_timeout(struct k_work *work)
{
        struct k_work_delayable *dwork;
        struct uart_tms570_data *data;

        dwork = k_work_delayable_from_work(work);
        data = CONTAINER_OF(dwork, struct uart_tms570_data, dma_tx.timeout_work);

        (void)uart_tms570_tx_abort(data->dev);
}

static void async_tx_done(struct k_work *work)
{
        int status;
        struct k_work_delayable *dwork;
        struct uart_tms570_data *data;
        struct uart_event evt;
        struct dma_status dma_stat;
        const struct uart_tms570_cfg *cfg;
        const struct device *dev;

        dwork = k_work_delayable_from_work(work);
        data = CONTAINER_OF(dwork, struct uart_tms570_data, dma_tx.done_work);
        dev = data->dev;
        cfg = dev->config;

        status = dma_get_status(cfg->dma_dev, cfg->dma_channel_tx, &dma_stat);
        if (status != 0) {
                LOG_ERR("unable to get dma status %i", status);
                return;
        }

        (void)k_work_cancel_delayable(&data->dma_tx.done_work);
        (void)k_work_cancel_delayable(&data->dma_tx.timeout_work);

        (void)dma_stop(cfg->dma_dev, cfg->dma_channel_tx);

        evt = (struct uart_event){
                .type = UART_TX_DONE,
                .data.tx.buf = data->dma_tx.buf,
                .data.tx.len = data->dma_tx.buf_size - dma_stat.pending_length,
        };
        async_event(dev, &evt);
}

static void uart_tms570_async_tx_isr(const struct device *dma, void *user_data, uint32_t channel,
                                     int status)
{
        const struct device *dev = user_data;
        struct uart_tms570_data *data = dev->data;

        if (status != DMA_STATUS_BLOCK) {
                return;
        }

        async_timer_restart(&data->dma_tx.done_work, 0);
}

#endif

static const struct uart_driver_api uart_tms570_driver_api = {
        .poll_in = uart_tms570_poll_in,
        .poll_out = uart_tms570_poll_out,

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        .fifo_fill = uart_tms570_fifo_fill,
        .fifo_read = uart_tms570_fifo_read,
        .irq_tx_enable = uart_tms570_irq_tx_enable,
        .irq_tx_disable = uart_tms570_irq_tx_disable,
        .irq_tx_ready = uart_tms570_irq_tx_ready,
        .irq_tx_complete = uart_tms570_irq_tx_complete,
        .irq_rx_enable = uart_tms570_irq_rx_enable,
        .irq_rx_disable = uart_tms570_irq_rx_disable,
        .irq_rx_ready = uart_tms570_irq_rx_ready,
        .irq_err_enable = uart_tms570_irq_err_enable,
        .irq_err_disable = uart_tms570_irq_err_disable,
        .irq_is_pending = uart_tms570_irq_is_pending,
        .irq_update = uart_tms570_irq_update,
        .irq_callback_set = uart_tms570_irq_callback_set,
#endif

#ifdef CONFIG_UART_ASYNC_API
        .rx_enable = uart_tms570_rx_enable,
        .rx_buf_rsp = uart_tms570_rx_buf_rsp,
        .rx_disable = uart_tms570_rx_disable,
        .tx = uart_tms570_tx,
        .tx_abort = uart_tms570_tx_abort,
        .callback_set = uart_tms570_callback_set,
#endif
};

#ifdef CONFIG_UART_ASYNC_API
static void uart_tms570_async_init(const struct device *dev)
{
        uintptr_t reg_base;
        const struct uart_tms570_cfg *cfg = dev->config;
        struct uart_tms570_data *data = dev->data;

        reg_base = DEVICE_MMIO_GET(dev);

        data->dev = dev;

        data->dma_rx.dma_blk = (struct dma_block_config){
                .source_address = reg_base + RDBUF_OFFSET + 3,
                .source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
                .dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
        };

        data->dma_rx.dma_cfg = (struct dma_config){
                .source_data_size = 1,
                .dest_data_size = 1,
                .channel_direction = PERIPHERAL_TO_MEMORY,
                .dma_slot = cfg->dma_request_rx,
                .dma_callback = uart_tms570_async_rx_isr,
                .user_data = (void *)dev,
                .head_block = &data->dma_rx.dma_blk,
                .block_count = 1,
                .cyclic = 1,
                .channel_priority = 1,
        };

        data->dma_tx.dma_blk = (struct dma_block_config){
                .dest_address = reg_base + TDBUF_OFFSET + 3,
                .dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
                .source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
        };

        data->dma_tx.dma_cfg = (struct dma_config){
                .source_data_size = 1,
                .dest_data_size = 1,
                .dma_slot = cfg->dma_request_tx,
                .user_data = (void *)dev,
                .channel_direction = MEMORY_TO_PERIPHERAL,
                .head_block = &data->dma_tx.dma_blk,
                .block_count = 1,
                .dma_callback = uart_tms570_async_tx_isr,
                .channel_priority = 1,
        };

        k_work_init_delayable(&data->dma_rx.timeout_work, async_rx_timeout);
        k_work_init_delayable(&data->dma_tx.timeout_work, async_tx_timeout);
        k_work_init_delayable(&data->dma_tx.done_work, async_tx_done);

        sys_set_bits(reg_base + SETINT_OFFSET, RXDMA_BITS | TXDMA_BIT);
}

#endif

/**
 * @brief Initialize UART device @p dev
 *
 * This implements the steps outlined in TMS570LS1224 technical reference
 * manual, section 30.5.
 *
 * @param dev
 * @return int Always 0
 */
static int uart_tms570_init(const struct device *dev)
{
        int status;
        uint32_t tmp;
        uintptr_t reg_base;
        const struct uart_tms570_cfg *cfg = dev->config;

        DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
        reg_base = DEVICE_MMIO_GET(dev);

        status = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
        /* -ENOENT means that pinctrl was not specified, which is fine in this case
         * as it is optional. */
        if (status < 0 && status != -ENOENT) {
                return status;
        }

        /* Set RESET bit to 1 */
        sys_set_bits(reg_base + CGR0_OFFSET, RESET_BIT);

        /* Set SWnRST bit to 0. Only when this bit is 0 can the SCI registers
         * be configured. This is set back to 1 at the end of the function */
        sys_clear_bits(reg_base + CGR1_OFFSET, SWnRST_BIT);

        sys_write32(FRAME7_BITS, reg_base + FORMAT_OFFSET);

        /* Configure TX and RX pins */
        sys_set_bits(reg_base + PIO0_OFFSET, TXFUNC_BIT | RXFUNC_BIT);

        /* Set the BAUD register */
        sys_write32(calc_baud_reg(cfg), reg_base + BRS_OFFSET);

        /* Enable clock. Enable CONT bit for emulation environment.
         * Additionally, enable LOOPBACK bit for use in a self test. */
        tmp = CLOCK_BIT | CONT_BIT | ASYNC_BIT;
        tmp |= TXENA_BIT | RXENA_BIT;

        sys_write32(tmp, reg_base + CGR1_OFFSET);

#ifdef CONFIG_UART_ASYNC_API
        uart_tms570_async_init(dev);
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        cfg->irq_connect(dev);
#endif

        /* Set the SWnRST bit back to 1. This prevents further configuration
         * of the peripheral. */
        sys_set_bits(reg_base + CGR1_OFFSET, SWnRST_BIT);

        return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_TMS570_IRQ_ENABLE_FN(n)                                                               \
        static void irq_config_func_##n(const struct device *dev)                                  \
        {                                                                                          \
                IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_tms570_isr,            \
                            DEVICE_DT_INST_GET(n), 0);                                             \
                irq_enable(DT_INST_IRQN(n));                                                       \
        }
#define UART_TMS570_IRQ_CFG(n) .irq_connect = irq_config_func_##n,
#else
#define UART_TMS570_IRQ_ENABLE_FN(n)
#define UART_TMS570_IRQ_CFG(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
#define UART_TMS570_ASYNC_CFG(n)                                                                   \
        .dma_dev = DEVICE_DT_GET(DT_INST(0, tms570_dma)),                                          \
        .dma_request_rx = DT_INST_PROP_OR(n, dma_request_rx, -1),                                  \
        .dma_request_tx = DT_INST_PROP_OR(n, dma_request_tx, -1),                                  \
        .dma_channel_rx = DT_INST_PROP_OR(n, dma_channel_rx, -1),                                  \
        .dma_channel_tx = DT_INST_PROP_OR(n, dma_channel_tx, -1),

#else
#define UART_TMS570_ASYNC_CFG(n)
#endif

#define UART_TMS570_INIT(node)                                                                     \
        PINCTRL_DT_INST_DEFINE(node);                                                              \
        UART_TMS570_IRQ_ENABLE_FN(node);                                                           \
        static const struct uart_tms570_cfg uart_tms570_##node##_config = {                        \
                DEVICE_MMIO_ROM_INIT(DT_DRV_INST(node)),                                           \
                .baud = DT_INST_PROP(node, current_speed),                                         \
                .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(node),                                    \
                .clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(node)),                              \
                .clk_domain = DT_CLOCKS_CELL(DT_DRV_INST(node), clk_id),                           \
                UART_TMS570_ASYNC_CFG(node) UART_TMS570_IRQ_CFG(node)};                            \
        static struct uart_tms570_data uart_tms570_##node##_data;                                  \
                                                                                                   \
        DEVICE_DT_INST_DEFINE(node, &uart_tms570_init, NULL, &uart_tms570_##node##_data,           \
                              &uart_tms570_##node##_config, PRE_KERNEL_1,                          \
                              CONFIG_SERIAL_INIT_PRIORITY, &uart_tms570_driver_api)

DT_INST_FOREACH_STATUS_OKAY(UART_TMS570_INIT);
