
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_tms570);

/* in ${ZEPHYR_BASE}/drivers/spi */
#include "spi_context.h"
#include "zephyr/dt-bindings/spi/spi.h"

#define DT_DRV_COMPAT ti_tms570_spi

#define MAX_FMT    (4)
#define MAX_SLAVES (8)

#define CGR0_OFFSET      (0x00)
#define CGR0_NRST_OFFSET (0)

#define CGR1_OFFSET        (0x04)
#define CGR1_EN_OFFSET     (24)
#define CGR1_CLKMOD_OFFSET (1)
#define CGR1_MASTER_OFFSET (0)

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
};

#ifdef SPI_TMS570_DMA
struct tms570_spi_dma {
};
#endif

struct tms570_spi_data {
        DEVICE_MMIO_RAM;

        struct spi_context ctx;

#ifdef SPI_TMS570_DMA
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
        int status;
        uintptr_t ctrl_reg_base;
        size_t word_size;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        /* Some of this are supported by the hardware, but not yet implemented in this driver. */
        if (spi_cfg->operation &
            (SPI_OP_MODE_SLAVE | SPI_MODE_LOOP | SPI_HALF_DUPLEX | SPI_LOCK_ON | SPI_HOLD_ON_CS)) {
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

        /* Set master bit, clock mode */
        sys_set_bits(ctrl_reg_base + CGR1_OFFSET,
                     BIT(CGR1_MASTER_OFFSET) | BIT(CGR1_CLKMOD_OFFSET));

        return 0;
}

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
        uint32_t dat1;
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

        dat1 = spi_cfg->slave << DAT1_CSNR_OFFSET;
        dat1 |= FMT_IDX << DAT1_DFSEL_OFFSET;
        sys_write32(dat1, ctrl_reg_base + DAT1_OFFSET);

        sys_set_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_EN_OFFSET);

        while (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
                tms570_spi_transfer(dev);
        }

        sys_clear_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_EN_OFFSET);

        spi_context_cs_control(&data->ctx, false);

exit:
        spi_context_release(&data->ctx, status);
        return status;
}

static int tms570_spi_transceive_sync(const struct device *dev, const struct spi_config *spi_cfg,
                                      const struct spi_buf_set *tx_bufs,
                                      const struct spi_buf_set *rx_bufs)
{
        return tms570_spi_transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int tms570_spi_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
                                       const struct spi_buf_set *tx_bufs,
                                       const struct spi_buf_set *rx_bufs, spi_callback_t cb,
                                       void *user_data)
{
        return tms570_spi_transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, user_data);
}
#endif

static int tms570_spi_release(const struct device *dev, const struct spi_config *config)
{
        struct tms570_spi_data *data = dev->data;
        uintptr_t ctrl_reg_base = DEVICE_MMIO_GET(dev);

        sys_clear_bit(ctrl_reg_base + CGR1_OFFSET, CGR1_EN_OFFSET);
        spi_context_unlock_unconditionally(&data->ctx);

        return 0;
}

static DEVICE_API(spi, tms570_spi_api) = {
        .transceive = tms570_spi_transceive_sync,
#ifdef CONFIG_SPI_ASYNC
        .transceive_async = tms570_spi_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
        .iodev_submit = spi_rtio_iodev_default_submit,
#endif
        .release = tms570_spi_release,
};

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

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        sys_set_bit(ctrl_reg_base + CGR0_OFFSET, CGR0_NRST_OFFSET);

        return 0;
}

#define TMS570_SPI_INIT(inst)                                                                      \
        PINCTRL_DT_INST_DEFINE(inst);                                                              \
        static const struct tms570_spi_cfg tms570_spi_##inst##_cfg = {                             \
                DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
                .clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                              \
                .clk_domain = DT_INST_CLOCKS_CELL(inst, clk_id),                                   \
                .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
        };                                                                                         \
        static struct tms570_spi_data tms570_spi_##inst##_data = {                                 \
                SPI_CONTEXT_INIT_SYNC(tms570_spi_##inst##_data, ctx),                              \
        };                                                                                         \
        SPI_DEVICE_DT_INST_DEFINE(inst, tms570_spi_init, NULL, &tms570_spi_##inst##_data,          \
                                  &tms570_spi_##inst##_cfg, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, \
                                  &tms570_spi_api)

DT_INST_FOREACH_STATUS_OKAY(TMS570_SPI_INIT);
