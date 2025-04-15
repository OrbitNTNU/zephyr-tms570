#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(spi_tms570, CONFIG_SPI_LOG_LEVEL);
#include "spi_context.h"

#define DT_DRV_COMPAT tms570_spi

typedef volatile struct {
        uint32_t GCR[2];
        uint32_t INT0;
        uint32_t LVL;
        uint32_t FLG;
        uint32_t IPC[9];
        uint32_t DAT[2];
        uint32_t BUF;
        uint32_t EMU;
        uint32_t DELAY;
        uint32_t DEF;
        uint32_t FMT[4];
        uint32_t INTVECT[2];
        uint32_t IPC9;
        uint32_t IPMCTRL;
        uint32_t MIBSPIE;
        uint32_t TGITENST;
        uint32_t TGITENCR;
        uint32_t TGITLVST;
        uint32_t TGITLVCR;
        uint32_t TGINTFLG;
        uint32_t reserved0[2];
        uint32_t TICKCNT;
        uint32_t LTGPEND;
        uint32_t TGxCTRL[16];
        uint32_t DMAxCTRL[4];
        uint32_t ICOUNT[8];
        uint32_t DMACNTLEN;
        uint32_t reserved1;
        uint32_t UERRCTRL;
        uint32_t UERRSTAT;
        uint32_t UERRADDR1;
        uint32_t UERRADDR0;
        uint32_t RXOVRN_BUF_ADDR;
        uint32_t IOLPBKTSTCR;
        uint32_t EXTENDED_PRESCALE1;
        uint32_t EXTENDED_PRESCALE2;
} tms570_spi_regs_t;

struct tms570_spi_config {
        tms570_spi_regs_t *regs;
        const struct pinctrl_dev_config *pcfg;
};

struct tms570_spi_data {
        struct spi_context ctx;
};

static int configure(const struct device *dev, const struct spi_config *config)
{
        const struct tms570_spi_config *cfg = dev->config;
        return pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
}

static int spi_tms570_transceive(const struct device *dev, const struct spi_config *config,
                                 const struct spi_buf_set *tx_bufs,
                                 const struct spi_buf_set *rx_bufs)
{
        struct tms570_spi_data *data = dev->data;
        const struct tms570_spi_config *cfg = dev->config;
        struct spi_context *ctx = &data->ctx;
        uint16_t txd, rxd;
        int err;

        spi_context_lock(ctx, false, NULL, NULL, config);

        err = configure(dev, config);
        if (err) {
                goto done;
        }

        spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

        spi_context_cs_control(ctx, true);

        do {
                if (spi_context_tx_buf_on(ctx)) {
                        txd = *ctx->tx_buf;
                } else {
                        txd = 0U;
                }

                while (cfg->regs->BUF & (1 << 29)) { // While TXFULL
                }
                cfg->regs->DAT[1] |= txd;

                spi_context_update_tx(ctx, 1, 1);

                while (cfg->regs->BUF & (1 << 31)) { // While RXEMPTY
                }
                rxd = cfg->regs->BUF & 0xffff;

                if (spi_context_rx_buf_on(ctx)) {
                        *ctx->rx_buf = rxd;
                }

                spi_context_update_rx(ctx, 1, 1);
        } while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx));

        spi_context_cs_control(ctx, false);

done:
        spi_context_release(ctx, err);
        return err;
}

static int spi_tms570_release(const struct device *dev, const struct spi_config *config)
{
        struct tms570_spi_data *data = dev->data;

        if (!spi_context_configured(&data->ctx, config)) {
                return -EINVAL;
        }

        spi_context_unlock_unconditionally(&data->ctx);

        return 0;
}

static int spi_tms570_init(const struct device *dev)
{
        const struct tms570_spi_config *cfg = dev->config;

        /** bring SPI out of reset */
        cfg->regs->GCR[0] = 1U;

        cfg->regs->IPC[0] |= ((uint32_t)(1) << 24) | ((uint32_t)(1) << 16) | (1 << 11) | (1 << 10) |
                             (1 << 9) | (1 << 8) | 1;

        cfg->regs->GCR[1] |= 0b11;

        /** - Delays */
        cfg->regs->DELAY = (0 << 24)   /* C2TDELAY */
                           | (0 << 16) /* T2CDELAY */
                           | (0 << 8)  /* T2EDELAY */
                           | 0;        /* C2EDELAY */

        /** - Data Format 0 */
        cfg->regs->FMT[0] = (0 << 24)   /* wdelay */
                            | (0 << 23) /* parity Polarity */
                            | (0 << 22) /* parity enable */
                            | (0 << 21) /* wait on enable */
                            | (0 << 20) /* shift direction */
                            | (1 << 17) /* clock polarity */
                            | (0 << 16) /* clock phase */
                            | (99 << 8) /* baudrate prescale */
                            | 8;        /* data word length */

        /** - Data Format 1 */
        cfg->regs->FMT[1] = (0 << 24)   /* wdelay */
                            | (0 << 23) /* parity Polarity */
                            | (0 << 22) /* parity enable */
                            | (0 << 21) /* wait on enable */
                            | (0 << 20) /* shift direction */
                            | (0 << 17) /* clock polarity */
                            | (0 << 16) /* clock phase */
                            | (99 << 8) /* baudrate prescale */
                            | 8;        /* data word length */

        /** - Data Format 2 */
        cfg->regs->FMT[2] = (0 << 24)   /* wdelay */
                            | (0 << 23) /* parity Polarity */
                            | (0 << 22) /* parity enable */
                            | (0 << 21) /* wait on enable */
                            | (0 << 20) /* shift direction */
                            | (0 << 17) /* clock polarity */
                            | (0 << 16) /* clock phase */
                            | (99 << 8) /* baudrate prescale */
                            | 8;        /* data word length */

        /** - Data Format 3 */
        cfg->regs->FMT[3] = (0 << 24)   /* wdelay */
                            | (0 << 23) /* parity Polarity */
                            | (0 << 22) /* parity enable */
                            | (0 << 21) /* wait on enable */
                            | (0 << 20) /* shift direction */
                            | (0 << 17) /* clock polarity */
                            | (0 << 16) /* clock phase */
                            | (99 << 8) /* baudrate prescale */
                            | 8;        /* data word length */

        /** - enable interrupts */
        cfg->regs->INT0 = (0 << 9)   /* TXINT */
                          | (0 << 8) /* RXINT */
                          | (0 << 6) /* OVRNINT */
                          | (0 << 4) /* BITERR */
                          | (0 << 3) /* DESYNC */
                          | (0 << 2) /* PARERR */
                          | (0 << 1) /* TIMEOUT */
                          | (0);     /* DLENERR */

        cfg->regs->DAT[1] &= (~((uint32_t)0xff << 16));
        cfg->regs->DAT[1] |= (1 << 16) | (1 << 24);

        cfg->regs->GCR[1] |= 1 << 24;

        return 0;
}

/* SPI driver APIs structure */
static const struct spi_driver_api tms570_spi_api = {
        .transceive = spi_tms570_transceive,
        .release = spi_tms570_release,
};

/* SPI driver registration */
#define SPI_TMS570_INIT(inst)                                                                      \
                                                                                                   \
        PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
        static struct tms570_spi_data tms570_spi_data_##inst = {                                   \
                SPI_CONTEXT_INIT_LOCK(tms570_spi_data_##inst, ctx),                                \
                SPI_CONTEXT_INIT_SYNC(tms570_spi_data_##inst, ctx),                                \
                SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)};                          \
                                                                                                   \
        static struct tms570_spi_config tms570_spi_cfg_##inst = {                                  \
                .regs = (tms570_spi_regs_t *)DT_INST_REG_ADDR(inst),                               \
                .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
        };                                                                                         \
                                                                                                   \
        DEVICE_DT_INST_DEFINE(inst, spi_tms570_init, NULL, &tms570_spi_data_##inst,                \
                              &tms570_spi_cfg_##inst, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,       \
                              &tms570_spi_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_TMS570_INIT)