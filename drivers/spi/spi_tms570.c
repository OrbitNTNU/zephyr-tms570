#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#include <stdio.h>

LOG_MODULE_REGISTER(spi_tms570, CONFIG_SPI_LOG_LEVEL);
#include "spi_context.h"

#define DT_DRV_COMPAT tms570_spi

typedef volatile struct {
        uint32_t GCR[2];
        uint32_t INT0;
        uint32_t LVL;
        uint32_t FLG;
        uint32_t PCFUN;  /**< 0x0014: Function Pin Enable */
        uint32_t PCDIR;  /**< 0x0018: Pin Direction */
        uint32_t PCDIN;  /**< 0x001C: Pin Input Latch */
        uint32_t PCDOUT; /**< 0x0020: Pin Output Latch */
        uint32_t PCSET;  /**< 0x0024: Output Pin Set */
        uint32_t PCCLR;  /**< 0x0028: Output Pin Clr */
        uint32_t PCPDR;  /**< 0x002C: Open Drain Output Enable */
        uint32_t PCDIS;  /**< 0x0030: Pullup/Pulldown Disable */
        uint32_t PCPSL;  /**< 0x0034: Pullup/Pulldown Selection */
        uint32_t DAT[2];
        uint32_t BUF;
        uint32_t EMU;
        uint32_t DELAY;
        uint32_t DEF;
        uint32_t FMT[4];
        uint32_t INTVECT[2];
        uint32_t PC9;
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
        LOG_INF("SPI Transceive");
        struct tms570_spi_data *data = dev->data;
        const struct tms570_spi_config *cfg = dev->config;
        struct spi_context *ctx = &data->ctx;
        uint16_t txd, rxd;
        int err;

        // LOG_INF("SPI context lock");
        // spi_context_lock(ctx, false, NULL, NULL, config);

        err = configure(dev, config);
        if (err) {
                LOG_ERR("configure failed err = %" PRIiLEAST32, err);
                goto done;
        }

        LOG_INF("Buffer setup");
        spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

        LOG_INF("CS control");
        spi_context_cs_control(ctx, true);

        do {
                LOG_INF("Transceive loop");
                if (spi_context_tx_buf_on(ctx)) {
                        txd = *ctx->tx_buf;
                } else {
                        txd = 0U;
                }
                LOG_INF("txd = %" PRIu16, txd);

                while (cfg->regs->BUF & (1 << 29)) { // While TXFULL
                        LOG_INF("TXFULL");
                }
                cfg->regs->DAT[1] = txd | (cfg->regs->DAT[1] & (0xffff0000));

                spi_context_update_tx(ctx, 1, 1);

                while (cfg->regs->BUF & (1 << 31)) { // While RXEMPTY
                        LOG_INF("RXEMPTY");
                }
                rxd = cfg->regs->BUF & 0xffff;
                LOG_INF("rxd = %" PRIu16, rxd);

                if (spi_context_rx_buf_on(ctx)) {
                        *ctx->rx_buf = rxd;
                }

                spi_context_update_rx(ctx, 1, 1);
        } while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx));

        spi_context_cs_control(ctx, false);

done:
        spi_context_release(ctx, err);
        LOG_INF("SPI Transceive complete");
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

        cfg->regs->GCR[0] = 1U; // Enable SPI by setting RESET bit

        cfg->regs->PCFUN = 0xffffffff; // Configure the SIMO, SOMI, SPICLK, and optional SPICS and
                                       // SPIENA pins for SPI functionality by setting the
                                       // corresponding bit in SPIPC0 register

        cfg->regs->GCR[1] |= 0b11; // Configure the module to function as Master or Slave using
                                   // CLKMOD and MASTER bits

        // Configure the required SPI data format using SPIFMTx register
        cfg->regs->FMT[0] = (0 << 24)   /* wdelay */
                            | (0 << 23) /* parity Polarity */
                            | (0 << 22) /* parity enable */
                            | (0 << 21) /* wait on enable */
                            | (0 << 20) /* shift direction */
                            | (1 << 17) /* clock polarity */
                            | (0 << 16) /* clock phase */
                            | (99 << 8) /* baudrate prescale */
                            | 8;        /* data word length */

        // If the module is selected to function as Master, the delay parameters can be configured
        // using SPIDELAY register
        cfg->regs->DELAY = (0 << 24)   /* C2TDELAY */
                           | (0 << 16) /* T2CDELAY */
                           | (0 << 8)  /* T2EDELAY */
                           | 0;        /* C2EDELAY */

        // Enable the Interrupts using SPIINT0 register if required

        cfg->regs->DAT[1] |= 0x3f << 16; // Select the chip select to be used by setting CSNR bits
                                         // in SPIDAT1 register

        // Configure CSHOLD and WDEL bits in SPIDAT1 register if required

        cfg->regs->DAT[1] &=
                ~(0b11
                  << 24); // Select the Data word format by setting DFSEL bits. Select the Number of
                          // the configured SPIFMTx register (0 to 3) to used for the communication

        // Set LOOPBACK bit to connect the transmitter to the receiver internally. (This feature is
        // used to perform a self-test. Do not configure for normal communication to external
        // devices)

        cfg->regs->GCR[1] |= 1 << 24;

        return 0;
}

static int spi_tms570_init2(const struct device *dev)
{
        const struct tms570_spi_config *cfg = dev->config;

        /** bring SPI out of reset */
        cfg->regs->GCR[0] = 1U;

        // cfg->regs->GCR[1] |= 1 << 16; // LOOPBACK

        // cfg->regs->PCFUN |= ((uint32_t)(1) << 24) | ((uint32_t)(1) << 16) | (1 << 11) | (1 << 10)
        // |
        //                     (1 << 9) | (1 << 8) | 1;

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

        cfg->regs->DAT[1] |= ((uint32_t)0x3f << 16); // CS
        // cfg->regs->DAT[1] |= 1 << 26;
        cfg->regs->DAT[1] |= 1 << 28;

        /* ---------- */

        /** - set interrupt levels */
        cfg->regs->LVL = (0 << 9)   /* TXINT */
                         | (0 << 8) /* RXINT */
                         | (0 << 6) /* OVRNINT */
                         | (0 << 4) /* BITERR */
                         | (0 << 3) /* DESYNC */
                         | (0 << 2) /* PARERR */
                         | (0 << 1) /* TIMEOUT */
                         | (0);     /* DLENERR */

        /** - clear any pending interrupts */
        cfg->regs->FLG = 0xFFFFFFFFU;

        /** - enable interrupts */
        cfg->regs->INT0 = (0 << 9)   /* TXINT */
                          | (0 << 8) /* RXINT */
                          | (0 << 6) /* OVRNINT */
                          | (0 << 4) /* BITERR */
                          | (0 << 3) /* DESYNC */
                          | (0 << 2) /* PARERR */
                          | (0 << 1) /* TIMEOUT */
                          | (0);     /* DLENERR */

        /** @b initalise @b SPI1 @b Port */

        /** - SPI1 Port output values */
        // cfg->regs->PCDOUT = 0            /* SCS[0] */
        //                     | (0 << 1)   /* SCS[1] */
        //                     | (0 << 2)   /* SCS[2] */
        //                     | (0 << 3)   /* SCS[3] */
        //                     | (0 << 8)   /* ENA */
        //                     | (0 << 9)   /* CLK */
        //                     | (0 << 10)  /* SIMO */
        //                     | (0 << 11); /* SOMI */
        cfg->regs->PCDOUT = 0xffffffff;
        /** - SPI1 Port direction */
        cfg->regs->PCDIR = 1            /* SCS[0] */
                           | (1 << 1)   /* SCS[1] */
                           | (1 << 2)   /* SCS[2] */
                           | (0 << 3)   /* SCS[3] */
                           | (0 << 8)   /* ENA */
                           | (1 << 9)   /* CLK */
                           | (1 << 10)  /* SIMO */
                           | (0 << 11); /* SOMI */

        /** - SPI1 Port open drain enable */
        cfg->regs->PCPDR = 0            /* SCS[0] */
                           | (0 << 1)   /* SCS[1] */
                           | (0 << 2)   /* SCS[2] */
                           | (0 << 3)   /* SCS[3] */
                           | (0 << 8)   /* ENA */
                           | (0 << 9)   /* CLK */
                           | (0 << 10)  /* SIMO */
                           | (0 << 11); /* SOMI */

        /** - SPI1 Port pullup / pulldown selection */
        cfg->regs->PCPSL = 1            /* SCS[0] */
                           | (1 << 1)   /* SCS[1] */
                           | (1 << 2)   /* SCS[2] */
                           | (0 << 3)   /* SCS[3] */
                           | (1 << 8)   /* ENA */
                           | (1 << 9)   /* CLK */
                           | (1 << 10)  /* SIMO */
                           | (1 << 11); /* SOMI */

        /** - SPI1 Port pullup / pulldown enable*/
        cfg->regs->PCDIS = 0            /* SCS[0] */
                           | (0 << 1)   /* SCS[1] */
                           | (0 << 2)   /* SCS[2] */
                           | (0 << 3)   /* SCS[3] */
                           | (0 << 8)   /* ENA */
                           | (0 << 9)   /* CLK */
                           | (0 << 10)  /* SIMO */
                           | (0 << 11); /* SOMI */

        /* SPI1 set all pins to functional */
        cfg->regs->PCFUN = 1            /* SCS[0] */
                           | (1 << 1)   /* SCS[1] */
                           | (1 << 2)   /* SCS[2] */
                           | (0 << 3)   /* SCS[3] */
                           | (1 << 8)   /* ENA */
                           | (1 << 9)   /* CLK */
                           | (1 << 10)  /* SIMO */
                           | (1 << 11); /* SOMI */

        /* ------ */

        cfg->regs->GCR[1] |= 1 << 24; // Enable

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