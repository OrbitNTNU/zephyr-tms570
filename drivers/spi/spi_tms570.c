
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/dma.h>

LOG_MODULE_REGISTER(spi_tms570);

/* in ${ZEPHYR_BASE}/drivers/spi */
#include "spi_context.h"
#include "zephyr/dt-bindings/spi/spi.h"

#define DT_DRV_COMPAT ti_tms570_spi

#define MAX_FMT    (4)
#define MAX_SLAVES (8)

#define DMA_TIMEOUT (K_USEC(CONFIG_SPI_TMS570_DMA_TIMEOUT_USEC))

#define CGR0_OFFSET      (0x00)
#define CGR0_NRST_OFFSET (0)

#define CGR1_OFFSET          (0x04)
#define CGR1_EN_OFFSET       (24)
#define CGR1_LOOPBACK_OFFSET (16)
#define CGR1_CLKMOD_OFFSET   (1)
#define CGR1_MASTER_OFFSET   (0)

#define INT0_OFFSET          (0x08)
#define INT0_DMAREQEN_OFFSET (16)
#define INT0_TXINTENA_OFFSET (9)
#define INT0_RXINTENA_OFFSET (8)

#define FLG_OFFSET       (0x10)
#define FLG_TXINT_OFFSET (9)
#define FLG_RXINT_OFFSET (8)

#define PC0_OFFSET         (0x14)
#define PC0_SOMIFUN_OFFSET (11)
#define PC0_SIMOFUN_OFFSET (10)
#define PC0_CLKFUN_OFFSET  (9)
#define PC0_SCSFUN_OFFSET  (0)

#define DAT1_OFFSET             (0x3c)
#define DAT1_DFSEL_OFFSET       (24)
#define DAT1_CSNR_OFFSET        (16)
#define DAT1_TXDATA_OFFSET      (0)
#define DAT1_TXDATA_BYTE_OFFSET (DAT1_OFFSET + 3)

#define BUF_OFFSET             (0x40)
#define BUF_RXEMPTY_OFFSET     (31)
#define BUF_TXFULL_OFFSET      (29)
#define BUF_RXDATA_OFFSET      (0)
#define BUF_RXDATA_BYTE_OFFSET (BUF_OFFSET + 3)

#define CSDEF_OFFSET (0x4c)

#define FMT_IDX             (0)
#define FMT_OFFSET_BASE     (0x50)
#define FMT_SHIFTDIR_OFFSET (20)
#define FMT_POLARITY_OFFSET (17)
#define FMT_PHASE_OFFSET    (16)
#define FMT_PRESCALE_OFFSET (8)
#define FMT_CHARLEN_OFFSET  (0)

struct tms570_spi_cfg {
        DEVICE_MMIO_ROM;

        const struct device *clk_ctrl;
        unsigned int clk_domain;

        const struct pinctrl_dev_config *pcfg;

        int dma_request_rx;
        int dma_channel_rx;
        int dma_request_tx;
        int dma_channel_tx;
        const struct device *dma_dev;
};

#ifdef CONFIG_SPI_TMS570_DMA
struct tms570_spi_dma {
        struct dma_block_config blk_config;
        struct dma_config config;
        struct k_sem sem;
        int status;
};
#endif

struct tms570_spi_data {
        DEVICE_MMIO_RAM;

        struct spi_context ctx;

#ifdef CONFIG_SPI_TMS570_DMA
        struct tms570_spi_dma dma_tx;
        struct tms570_spi_dma dma_rx;
#endif
};

static int tms570_spi_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
        const struct tms570_spi_cfg *cfg = dev->config;
        uint32_t fmt;
        uint32_t clk_rate;
        uint32_t psc;
        uint32_t dat1;
        int status;
        uintptr_t ctrl_reg_base;
        size_t word_size;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        /* Some of this are supported by the hardware, but not yet implemented in this driver. */
        if (spi_cfg->operation &
            (SPI_OP_MODE_SLAVE | SPI_HALF_DUPLEX | SPI_LOCK_ON | SPI_HOLD_ON_CS)) {
                return -ENOTSUP;
        }

        if (spi_cfg->slave >= MAX_SLAVES) {
                return -ENOTSUP;
        }

        /* HW technically support sizes <=16, but this driver only implements 8bit */
        word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
        if (word_size != 8) {
                return -EINVAL;
        }

        status = clock_control_get_rate(dev, (clock_control_subsys_t)&cfg->clk_domain, &clk_rate);
        if (status != 0) {
                return status;
        }

        psc = clk_rate / spi_cfg->frequency - 1;

        fmt = word_size << FMT_CHARLEN_OFFSET;
        fmt |= psc << FMT_PRESCALE_OFFSET;
        fmt |= (!!(spi_cfg->operation & SPI_MODE_CPOL)) << FMT_POLARITY_OFFSET;
        fmt |= (!!(spi_cfg->operation & SPI_MODE_CPHA)) << FMT_PHASE_OFFSET;
        fmt |= (!!(spi_cfg->operation & SPI_TRANSFER_LSB)) << FMT_SHIFTDIR_OFFSET;

        sys_write32(fmt, ctrl_reg_base + FMT_OFFSET_BASE + sizeof(uint32_t) * FMT_IDX);

        /* Set CS active state. TODO: Is this correct? */
        if (spi_cfg->operation & SPI_CS_ACTIVE_HIGH) {
                sys_clear_bit(ctrl_reg_base + CSDEF_OFFSET, spi_cfg->slave);
        } else {
                sys_set_bit(ctrl_reg_base + CSDEF_OFFSET, spi_cfg->slave);
        }

        /* Slave number, format index */
        dat1 = spi_cfg->slave << DAT1_CSNR_OFFSET;
        dat1 |= FMT_IDX << DAT1_DFSEL_OFFSET;
        sys_write32(dat1, ctrl_reg_base + DAT1_OFFSET);

        if (spi_cfg->operation & SPI_MODE_LOOP) {
                sys_set_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_LOOPBACK_OFFSET);
        } else {
                sys_clear_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_LOOPBACK_OFFSET);
        }

        /* Set master bit, clock mode */
        sys_set_bits(ctrl_reg_base + CGR1_OFFSET,
                     BIT(CGR1_MASTER_OFFSET) | BIT(CGR1_CLKMOD_OFFSET));

        return 0;
}

static void tms570_spi_module_toggle(const struct device *dev, bool on)
{
        uintptr_t ctrl_reg_base = DEVICE_MMIO_GET(dev);

        if (on) {
                sys_set_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_EN_OFFSET);
        } else {
                sys_clear_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_EN_OFFSET);
        }
}

#ifndef CONFIG_SPI_TMS570_DMA
static void tms570_spi_transfer(const struct device *dev)
{
        struct tms570_spi_data *data = dev->data;
        uint8_t tx_byte;
        uint8_t rx_byte;
        uintptr_t ctrl_reg_base;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        tx_byte = 0;
        if (spi_context_tx_on(&data->ctx)) {
                tx_byte = *data->ctx.tx_buf;
        }

        spi_context_update_tx(&data->ctx, 1, 1);

        /* Write byte */
        while (sys_test_bit(ctrl_reg_base + BUF_OFFSET, BUF_TXFULL_OFFSET)) {
        }
        sys_write8(tx_byte, ctrl_reg_base + DAT1_TXDATA_BYTE_OFFSET);

        /* Read received byte */
        while (sys_test_bit(ctrl_reg_base + BUF_OFFSET, BUF_RXEMPTY_OFFSET)) {
        }
        rx_byte = sys_read8(ctrl_reg_base + BUF_RXDATA_BYTE_OFFSET);

        if (spi_context_rx_on(&data->ctx)) {
                *data->ctx.rx_buf = rx_byte;
        }

        spi_context_update_rx(&data->ctx, 1, 1);
}

static int tms570_spi_transceive(const struct device *dev, const struct spi_config *spi_cfg,
                                 const struct spi_buf_set *tx_bufs,
                                 const struct spi_buf_set *rx_bufs, bool async, spi_callback_t cb,
                                 void *user_data)
{
        struct tms570_spi_data *data = dev->data;
        uintptr_t ctrl_reg_base;
        int status;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        spi_context_lock(&data->ctx, async, cb, user_data, spi_cfg);

        status = tms570_spi_configure(dev, spi_cfg);
        if (status != 0) {
                goto exit;
        }

        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
        spi_context_cs_control(&data->ctx, true);

        tms570_spi_module_toggle(dev, true);

        while (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
                tms570_spi_transfer(dev);
        }

        tms570_spi_module_toggle(dev, false);

        spi_context_cs_control(&data->ctx, false);

exit:
        spi_context_release(&data->ctx, status);
        return status;
}

#else
static void tms570_spi_dma_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
                                    int status)
{
        struct tms570_spi_dma *data = user_data;

        /* Only process fully complete (one block, but will trigger DMA_STATUS_COMPLETE in addition
         * to DMA_STATUS_BLOCK). */
        if (status == DMA_STATUS_BLOCK || status == DMA_STATUS_HALF_COMPLETE) {
                return;
        }

        data->status = status;
        k_sem_give(&data->sem);
}

static int tms570_spi_wait_dma_xfer(const struct device *dev, struct tms570_spi_dma *dma_data,
                                    k_timeout_t timeout)
{
        int status;

        status = k_sem_take(&dma_data->sem, timeout);
        if (status != 0) {
                return status;
        }

        if (dma_data->status != DMA_STATUS_COMPLETE) {
                return dma_data->status;
        }

        return 0;
}

static int tms570_spi_wait_dma(const struct device *dev)
{
        struct tms570_spi_data *data = dev->data;
        int status;
        k_timepoint_t expiry;

        expiry = sys_timepoint_calc(DMA_TIMEOUT);

        status = tms570_spi_wait_dma_xfer(dev, &data->dma_tx, sys_timepoint_timeout(expiry));
        if (status != 0) {
                return status;
        }

        return tms570_spi_wait_dma_xfer(dev, &data->dma_rx, sys_timepoint_timeout(expiry));
}

static int tms570_spi_transceive_dma(const struct device *dev, const struct spi_config *spi_cfg,
                                     const struct spi_buf_set *tx_bufs,
                                     const struct spi_buf_set *rx_bufs)
{
        const struct tms570_spi_cfg *cfg = dev->config;
        struct tms570_spi_data *data = dev->data;
        int status;
        size_t len;
        uint8_t tx_dummy;
        uint8_t rx_dummy;

        spi_context_lock(&data->ctx, false, NULL, NULL, spi_cfg);

        status = tms570_spi_configure(dev, spi_cfg);
        if (status != 0) {
                goto exit;
        }

        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

        tms570_spi_module_toggle(dev, true);
        spi_context_cs_control(&data->ctx, true);

        data->dma_rx.status = 0;
        data->dma_tx.status = 0;
        k_sem_reset(&data->dma_rx.sem);
        k_sem_reset(&data->dma_tx.sem);

        while (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {

                len = spi_context_max_continuous_chunk(&data->ctx);

                if (spi_context_tx_on(&data->ctx)) {
                        data->dma_tx.blk_config.source_address = (uint32_t)data->ctx.tx_buf;
                        data->dma_tx.blk_config.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
                } else {
                        data->dma_tx.blk_config.source_address = (uint32_t)&tx_dummy;
                        data->dma_tx.blk_config.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
                }

                if (spi_context_rx_on(&data->ctx)) {
                        data->dma_rx.blk_config.dest_address = (uint32_t)data->ctx.rx_buf;
                        data->dma_rx.blk_config.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
                } else {
                        data->dma_rx.blk_config.dest_address = (uint32_t)&rx_dummy;
                        data->dma_rx.blk_config.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
                }

                data->dma_rx.blk_config.block_size = len;
                data->dma_tx.blk_config.block_size = len;

                status = dma_config(cfg->dma_dev, cfg->dma_channel_rx, &data->dma_rx.config);
                if (status != 0) {
                        break;
                }

                status = dma_config(cfg->dma_dev, cfg->dma_channel_tx, &data->dma_tx.config);
                if (status != 0) {
                        break;
                }

                status = dma_start(cfg->dma_dev, cfg->dma_channel_rx);
                if (status != 0) {
                        break;
                }

                status = dma_start(cfg->dma_dev, cfg->dma_channel_tx);
                if (status != 0) {
                        (void)dma_stop(cfg->dma_dev, cfg->dma_channel_rx);
                        break;
                }

                /* Wait for chunk transfer to be done */
                status = tms570_spi_wait_dma(dev);
                if (status != 0) {
                        (void)dma_stop(cfg->dma_dev, cfg->dma_channel_rx);
                        (void)dma_stop(cfg->dma_dev, cfg->dma_channel_tx);
                        break;
                }

                spi_context_update_tx(&data->ctx, 1, len);
                spi_context_update_rx(&data->ctx, 1, len);
        }

        spi_context_cs_control(&data->ctx, false);
exit:
        tms570_spi_module_toggle(dev, false);
        spi_context_release(&data->ctx, status);
        return status;
}
#endif

static int tms570_spi_transceive_sync(const struct device *dev, const struct spi_config *spi_cfg,
                                      const struct spi_buf_set *tx_bufs,
                                      const struct spi_buf_set *rx_bufs)
{
#ifdef CONFIG_SPI_TMS570_DMA
        return tms570_spi_transceive_dma(dev, spi_cfg, tx_bufs, rx_bufs);
#else
        return tms570_spi_transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
#endif
}

static int tms570_spi_release(const struct device *dev, const struct spi_config *config)
{
        struct tms570_spi_data *data = dev->data;

        tms570_spi_module_toggle(dev, false);
        spi_context_unlock_unconditionally(&data->ctx);

        return 0;
}

static DEVICE_API(spi, tms570_spi_api) = {
        .transceive = tms570_spi_transceive_sync,
#ifdef CONFIG_SPI_RTIO
        .iodev_submit = spi_rtio_iodev_default_submit,
#endif
        .release = tms570_spi_release,
};

#ifdef CONFIG_SPI_TMS570_DMA
static void tms570_spi_dma_init(const struct device *dev)
{
        const struct tms570_spi_cfg *cfg = dev->config;
        struct tms570_spi_data *data = dev->data;
        uintptr_t ctrl_reg_base;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        data->dma_tx.blk_config = (struct dma_block_config){
                .dest_address = ctrl_reg_base + DAT1_TXDATA_BYTE_OFFSET,
                .dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
        };
        data->dma_tx.config = (struct dma_config){
                .source_data_size = 1,
                .dest_data_size = 1,
                .channel_direction = MEMORY_TO_PERIPHERAL,
                .user_data = (void *)&data->dma_tx,
                .dma_callback = tms570_spi_dma_callback,
                .dma_slot = cfg->dma_request_tx,
                .head_block = &data->dma_tx.blk_config,
                .block_count = 1,
                .channel_priority = 1,
                .cyclic = 1,
        };
        data->dma_rx.blk_config = (struct dma_block_config){
                .source_address = ctrl_reg_base + BUF_RXDATA_BYTE_OFFSET,
                .source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
        };
        data->dma_rx.config = (struct dma_config){
                .source_data_size = 1,
                .dest_data_size = 1,
                .channel_direction = PERIPHERAL_TO_MEMORY,
                .user_data = (void *)&data->dma_rx,
                .dma_callback = tms570_spi_dma_callback,
                .dma_slot = cfg->dma_request_rx,
                .head_block = &data->dma_rx.blk_config,
                .block_count = 1,
                .channel_priority = 1,
                .cyclic = 1,
        };

        (void)k_sem_init(&data->dma_tx.sem, 0, 1);
        (void)k_sem_init(&data->dma_rx.sem, 0, 1);

        sys_set_bit(ctrl_reg_base + INT0_OFFSET, INT0_DMAREQEN_OFFSET);
}
#endif

static int tms570_spi_init(const struct device *dev)
{
        const struct tms570_spi_cfg *cfg = dev->config;
        struct tms570_spi_data *data = dev->data;
        uintptr_t ctrl_reg_base;
        int status;

        DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

        spi_context_unlock_unconditionally(&data->ctx);

        status = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if (status != 0) {
                return status;
        }

#ifdef CONFIG_SPI_TMS570_DMA
        tms570_spi_dma_init(dev);
#endif

        ctrl_reg_base = DEVICE_MMIO_GET(dev);
        sys_set_bit(ctrl_reg_base + CGR0_OFFSET, CGR0_NRST_OFFSET);

        return 0;
}

#ifdef CONFIG_SPI_TMS570_DMA
#define TMS570_SPI_DMA_CFG(inst)                                                                   \
        .dma_dev = DEVICE_DT_GET(DT_INST(0, tms570_dma)),                                          \
        .dma_request_rx = DT_INST_PROP_OR(inst, dma_request_rx, -1),                               \
        .dma_request_tx = DT_INST_PROP_OR(inst, dma_request_tx, -1),                               \
        .dma_channel_rx = DT_INST_PROP_OR(inst, dma_channel_rx, -1),                               \
        .dma_channel_tx = DT_INST_PROP_OR(inst, dma_channel_tx, -1),
#else
#define TMS570_SPI_DMA_CFG(inst)
#endif

#define TMS570_SPI_INIT(inst)                                                                      \
        PINCTRL_DT_INST_DEFINE(inst);                                                              \
        static const struct tms570_spi_cfg tms570_spi_##inst##_cfg = {                             \
                DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
                .clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                              \
                .clk_domain = DT_INST_CLOCKS_CELL(inst, clk_id),                                   \
                .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), TMS570_SPI_DMA_CFG(inst)};           \
        static struct tms570_spi_data tms570_spi_##inst##_data = {                                 \
                SPI_CONTEXT_INIT_SYNC(tms570_spi_##inst##_data, ctx),                              \
        };                                                                                         \
        SPI_DEVICE_DT_INST_DEFINE(inst, tms570_spi_init, NULL, &tms570_spi_##inst##_data,          \
                                  &tms570_spi_##inst##_cfg, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, \
                                  &tms570_spi_api)

DT_INST_FOREACH_STATUS_OKAY(TMS570_SPI_INIT);
