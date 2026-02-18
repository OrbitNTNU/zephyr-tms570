
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
#define DAT1_DFSEL_OFFSET       (24 - 16) /* Offset minus 16 data bits */
#define DAT1_CSNR_OFFSET        (16 - 16) /* Offset minus 16 data bits */
#define DAT1_TXDATA_BYTE_OFFSET (DAT1_OFFSET + 3)

#define BUF_OFFSET             (0x40)
#define BUF_RXEMPTY_OFFSET     (31 - 16) /* Offset minus 16 data bits */
#define BUF_TXFULL_OFFSET      (29 - 16) /* Offset minus 16 data bits */
#define BUF_RXDATA_BYTE_OFFSET (BUF_OFFSET + 3)

#define DELAY_OFFSET     (0x48)
#define DELAY_C2T_MASK   BIT_MASK(8)
#define DELAY_C2T_OFFSET (24)
#define DELAY_T2C_MASK   BIT_MASK(8)
#define DELAY_T2C_OFFSET (16)

#define CSDEF_OFFSET (0x4c)

#define FMT_IDX             (0)
#define FMT_OFFSET_BASE     (0x50)
#define FMT_SHIFTDIR_OFFSET (20)
#define FMT_POLARITY_OFFSET (17)
#define FMT_PHASE_OFFSET    (16)
#define FMT_PRESCALE_OFFSET (8)
#define FMT_CHARLEN_OFFSET  (0)

#ifdef CONFIG_SPI_TMS570_DMA
struct spi_tms570_dma {
        struct dma_block_config blk_config;
        struct dma_config config;
        struct k_sem sem;
        int status;
        unsigned int channel;
        const struct device *dev;
};
#endif

struct spi_tms570_cfg {
        DEVICE_MMIO_ROM;

        const struct device *clk_ctrl;
        unsigned int clk_domain;

        const struct pinctrl_dev_config *pcfg;
};

struct spi_tms570_data {
        DEVICE_MMIO_RAM;

        struct spi_context ctx;

#ifdef CONFIG_SPI_TMS570_DMA
        struct spi_tms570_dma dma_tx;
        struct spi_tms570_dma dma_rx;
#endif
};

static uint32_t calc_cs_delay_reg(const struct spi_config *spi_cfg, uint32_t vclk_rate_hz)
{
        uint32_t val;
        uint64_t tmp;
        uint64_t vclk_per_ps;

        __ASSERT_NO_MSG(!spi_cfg->cs.cs_is_gpio);

        vclk_per_ps = (uint64_t)1e12 / vclk_rate_hz;
        tmp = MAX(((uint64_t)1e3 * spi_cfg->cs.setup_ns) / vclk_per_ps, 2) - 2;
        val = (tmp & DELAY_C2T_MASK) << DELAY_C2T_OFFSET;

        tmp = MAX(((uint64_t)1e3 * spi_cfg->cs.hold_ns) / vclk_per_ps, 1) - 1;
        val |= (tmp & DELAY_T2C_MASK) << DELAY_T2C_OFFSET;

        return val;
}

static int spi_tms570_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
        const struct spi_tms570_cfg *cfg = dev->config;
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

        status = clock_control_get_rate(cfg->clk_ctrl, (clock_control_subsys_t)&cfg->clk_domain,
                                        &clk_rate);
        if (status != 0) {
                return status;
        }

        /* Configure pins */
        sys_set_bits(ctrl_reg_base + PC0_OFFSET,
                     BIT(PC0_SIMOFUN_OFFSET) | BIT(PC0_SOMIFUN_OFFSET) | BIT(PC0_CLKFUN_OFFSET));
        if (!spi_cfg->cs.cs_is_gpio) {
                sys_write32(calc_cs_delay_reg(spi_cfg, clk_rate), ctrl_reg_base + DELAY_OFFSET);
                sys_set_bit(ctrl_reg_base + PC0_OFFSET, PC0_SCSFUN_OFFSET + spi_cfg->slave);
        } else {
                sys_clear_bit(ctrl_reg_base + PC0_OFFSET, PC0_SCSFUN_OFFSET + spi_cfg->slave);
        }

        /* Set master bit, clock mode */
        sys_set_bits(ctrl_reg_base + CGR1_OFFSET,
                     BIT(CGR1_MASTER_OFFSET) | BIT(CGR1_CLKMOD_OFFSET));

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

        /* Slave number, format index. Only write 16 bits so we don't attempt
         * to inititate transfer. */
        dat1 = 1 << (DAT1_CSNR_OFFSET + spi_cfg->slave);
        dat1 |= FMT_IDX << DAT1_DFSEL_OFFSET;
        sys_write16(dat1, ctrl_reg_base + DAT1_OFFSET);

        if (spi_cfg->operation & SPI_MODE_LOOP) {
                sys_set_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_LOOPBACK_OFFSET);
        } else {
                sys_clear_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_LOOPBACK_OFFSET);
        }

        return 0;
}

static void spi_tms570_module_toggle(const struct device *dev, bool on)
{
        uintptr_t ctrl_reg_base = DEVICE_MMIO_GET(dev);

        if (on) {
                sys_set_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_EN_OFFSET);
        } else {
                sys_clear_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_EN_OFFSET);
        }
}

static void spi_tms570_transfer(const struct device *dev)
{
        struct spi_tms570_data *data = dev->data;
        uint8_t tx_byte;
        uint8_t rx_byte;
        uintptr_t ctrl_reg_base;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        tx_byte = 0;
        if (spi_context_tx_on(&data->ctx)) {
                tx_byte = *data->ctx.tx_buf;
        }

        spi_context_update_tx(&data->ctx, 1, 1);

        /* Write byte. Only pull upper 16 bits to not clear/set any flags accidentally */
        while ((sys_read16(ctrl_reg_base + BUF_OFFSET) & BIT(BUF_TXFULL_OFFSET)) != 0) {
        }
        sys_write8(tx_byte, ctrl_reg_base + DAT1_TXDATA_BYTE_OFFSET);

        /* Read received byte. Only pull upper 16 bits to not clear/set any flags accidentally */
        while ((sys_read16(ctrl_reg_base + BUF_OFFSET) & BIT(BUF_RXEMPTY_OFFSET)) != 0) {
        }
        rx_byte = sys_read8(ctrl_reg_base + BUF_RXDATA_BYTE_OFFSET) & BIT_MASK(8);

        if (spi_context_rx_on(&data->ctx)) {
                *data->ctx.rx_buf = rx_byte;
        }

        spi_context_update_rx(&data->ctx, 1, 1);
}

static int spi_tms570_transceive(const struct device *dev, const struct spi_config *spi_cfg,
                                 const struct spi_buf_set *tx_bufs,
                                 const struct spi_buf_set *rx_bufs, bool async, spi_callback_t cb,
                                 void *user_data)
{
        struct spi_tms570_data *data = dev->data;
        uintptr_t ctrl_reg_base;
        int status;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        spi_context_lock(&data->ctx, async, cb, user_data, spi_cfg);

        status = spi_tms570_configure(dev, spi_cfg);
        if (status != 0) {
                goto exit;
        }

        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
        spi_context_cs_control(&data->ctx, true);

        spi_tms570_module_toggle(dev, true);

        while (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
                spi_tms570_transfer(dev);
        }

        spi_tms570_module_toggle(dev, false);

        spi_context_cs_control(&data->ctx, false);

exit:
        spi_context_release(&data->ctx, status);
        return status;
}

#ifdef CONFIG_SPI_TMS570_DMA
static void spi_tms570_dma_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
                                    int status)
{
        struct spi_tms570_dma *data = user_data;

        /* Only process fully complete (one block, but will trigger DMA_STATUS_COMPLETE in addition
         * to DMA_STATUS_BLOCK). */
        if (status == DMA_STATUS_BLOCK || status == DMA_STATUS_HALF_COMPLETE) {
                return;
        }

        data->status = status;
        k_sem_give(&data->sem);
}

static int spi_tms570_wait_dma_xfer(const struct device *dev, struct spi_tms570_dma *dma_data,
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

static int spi_tms570_wait_dma(const struct device *dev)
{
        struct spi_tms570_data *data = dev->data;
        int status;
        k_timepoint_t expiry;

        expiry = sys_timepoint_calc(DMA_TIMEOUT);

        status = spi_tms570_wait_dma_xfer(dev, &data->dma_tx, sys_timepoint_timeout(expiry));
        if (status != 0) {
                return status;
        }

        return spi_tms570_wait_dma_xfer(dev, &data->dma_rx, sys_timepoint_timeout(expiry));
}

static void spi_tms570_dma_toggle(const struct device *dev, bool on)
{
        uintptr_t ctrl_reg_base = DEVICE_MMIO_GET(dev);

        if (on) {
                sys_set_bit(ctrl_reg_base + INT0_OFFSET, INT0_DMAREQEN_OFFSET);
        } else {
                sys_clear_bit(ctrl_reg_base + INT0_OFFSET, INT0_DMAREQEN_OFFSET);
        }
}

static int spi_tms570_transceive_dma(const struct device *dev, const struct spi_config *spi_cfg,
                                     const struct spi_buf_set *tx_bufs,
                                     const struct spi_buf_set *rx_bufs)
{
        struct spi_tms570_data *data = dev->data;
        int status;
        size_t len;
        uint8_t tx_dummy;
        uint8_t rx_dummy;

        spi_context_lock(&data->ctx, false, NULL, NULL, spi_cfg);

        status = spi_tms570_configure(dev, spi_cfg);
        if (status != 0) {
                goto exit;
        }

        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

        spi_tms570_module_toggle(dev, true);
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

                status = dma_config(data->dma_rx.dev, data->dma_rx.channel, &data->dma_rx.config);
                if (status != 0) {
                        break;
                }

                status = dma_config(data->dma_tx.dev, data->dma_tx.channel, &data->dma_tx.config);
                if (status != 0) {
                        break;
                }

                status = dma_start(data->dma_rx.dev, data->dma_rx.channel);
                if (status != 0) {
                        break;
                }

                status = dma_start(data->dma_tx.dev, data->dma_tx.channel);
                if (status != 0) {
                        (void)dma_stop(data->dma_rx.dev, data->dma_rx.channel);
                        break;
                }

                /* Initiate request */
                spi_tms570_dma_toggle(dev, true);

                /* Wait for chunk transfer to be done */
                status = spi_tms570_wait_dma(dev);
                spi_tms570_dma_toggle(dev, false);

                if (status != 0) {
                        (void)dma_stop(data->dma_rx.dev, data->dma_rx.channel);
                        (void)dma_stop(data->dma_tx.dev, data->dma_tx.channel);
                        break;
                }

                spi_context_update_tx(&data->ctx, 1, len);
                spi_context_update_rx(&data->ctx, 1, len);
        }

        spi_context_cs_control(&data->ctx, false);
exit:
        spi_tms570_module_toggle(dev, false);
        spi_context_release(&data->ctx, status);
        return status;
}
#endif

static int spi_tms570_transceive_sync(const struct device *dev, const struct spi_config *spi_cfg,
                                      const struct spi_buf_set *tx_bufs,
                                      const struct spi_buf_set *rx_bufs)
{
#ifdef CONFIG_SPI_TMS570_DMA
        struct spi_tms570_data *data = dev->data;

        if (data->dma_rx.dev != NULL && data->dma_tx.dev != NULL) {
                return spi_tms570_transceive_dma(dev, spi_cfg, tx_bufs, rx_bufs);
        }
#endif

        return spi_tms570_transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

static int spi_tms570_release(const struct device *dev, const struct spi_config *config)
{
        struct spi_tms570_data *data = dev->data;

        spi_tms570_module_toggle(dev, false);
        spi_context_unlock_unconditionally(&data->ctx);

        return 0;
}

static DEVICE_API(spi, spi_tms570_api) = {
        .transceive = spi_tms570_transceive_sync,
#ifdef CONFIG_SPI_RTIO
        .iodev_submit = spi_rtio_iodev_default_submit,
#endif
        .release = spi_tms570_release,
};

#ifdef CONFIG_SPI_TMS570_DMA
static void spi_tms570_dma_init(const struct device *dev)
{
        struct spi_tms570_data *data = dev->data;
        uintptr_t ctrl_reg_base;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        data->dma_tx.blk_config = (struct dma_block_config){
                .dest_address = ctrl_reg_base + DAT1_TXDATA_BYTE_OFFSET,
                .dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
        };
        data->dma_rx.blk_config = (struct dma_block_config){
                .source_address = ctrl_reg_base + BUF_RXDATA_BYTE_OFFSET,
                .source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
        };

        (void)k_sem_init(&data->dma_tx.sem, 0, 1);
        (void)k_sem_init(&data->dma_rx.sem, 0, 1);
}
#endif

static int spi_tms570_init(const struct device *dev)
{
        const struct spi_tms570_cfg *cfg = dev->config;
        struct spi_tms570_data *data = dev->data;
        uintptr_t ctrl_reg_base;
        int status;

        DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

        spi_context_unlock_unconditionally(&data->ctx);

        status = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if (status != 0) {
                return status;
        }

#ifdef CONFIG_SPI_TMS570_DMA
        spi_tms570_dma_init(dev);
#endif

        ctrl_reg_base = DEVICE_MMIO_GET(dev);
        sys_set_bit(ctrl_reg_base + CGR0_OFFSET, CGR0_NRST_OFFSET);

        return 0;
}

#ifdef CONFIG_SPI_TMS570_DMA
#define SPI_TMS570_DMA_DIR_INIT(inst, dir, ch_dir)                                                 \
        .dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, dir)),                                \
        .channel = DT_INST_DMAS_CELL_BY_NAME(inst, dir, channel),                                  \
        .config = {                                                                                \
                .source_data_size = 1,                                                             \
                .dest_data_size = 1,                                                               \
                .channel_direction = ch_dir,                                                       \
                .user_data = &spi_tms570_##inst##_data.dma_##dir,                                  \
                .dma_callback = spi_tms570_dma_callback,                                           \
                .dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, dir, request),                         \
                .head_block = &spi_tms570_##inst##_data.dma_##dir.blk_config,                      \
                .block_count = 1,                                                                  \
                .channel_priority = 1,                                                             \
                .cyclic = 1,                                                                       \
        },

#define SPI_TMS570_DMA_DIR_DATA(inst, dir, ch_dir)                                                 \
        .dma_##dir = {COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, dir),                                \
                                  (SPI_TMS570_DMA_DIR_INIT(inst, dir, ch_dir)), (.dev = NULL))},

#define SPI_TMS570_DMA_DATA(inst)                                                                  \
        SPI_TMS570_DMA_DIR_DATA(inst, rx, PERIPHERAL_TO_MEMORY)                                    \
        SPI_TMS570_DMA_DIR_DATA(inst, tx, MEMORY_TO_PERIPHERAL)
#else
#define SPI_TMS570_DMA_DATA(inst)
#endif

#define SPI_TMS570_INIT(inst)                                                                      \
        PINCTRL_DT_INST_DEFINE(inst);                                                              \
        static struct spi_tms570_data spi_tms570_##inst##_data = {                                 \
                SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)                            \
                        SPI_CONTEXT_INIT_LOCK(spi_tms570_##inst##_data, ctx),                      \
                SPI_CONTEXT_INIT_SYNC(spi_tms570_##inst##_data, ctx), SPI_TMS570_DMA_DATA(inst)};  \
        static const struct spi_tms570_cfg spi_tms570_##inst##_cfg = {                             \
                DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
                .clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                              \
                .clk_domain = DT_INST_CLOCKS_CELL(inst, clk_id),                                   \
                .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
        };                                                                                         \
        SPI_DEVICE_DT_INST_DEFINE(inst, spi_tms570_init, NULL, &spi_tms570_##inst##_data,          \
                                  &spi_tms570_##inst##_cfg, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, \
                                  &spi_tms570_api)

DT_INST_FOREACH_STATUS_OKAY(SPI_TMS570_INIT);
