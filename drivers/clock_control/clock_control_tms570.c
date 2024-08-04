
#include <zephyr/types.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/device_mmio.h>

#include <orbit/dt-bindings/clock_control/clock_control_tms570.h>

struct tms570_clock_cfg {
        uint32_t clock_frequency;
};

#define DRV_PLL   DT_INST(0, tms570_pll)
#define DRV_CLOCK DT_INST(0, tms570_clock)
#define DRV_FREQ  DT_PROP(DRV_CLOCK, clock_frequency)
#define DRV_REG   DT_REG_ADDR(DRV_CLOCK)

BUILD_ASSERT(DRV_FREQ == CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
             "clock frequency of clock control does not match CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC");

/* PLL scaling coefficients */
#define VAL_NF ((DT_PROP(DRV_PLL, nf) - 1) << 8)
#define VAL_NR ((DT_PROP(DRV_PLL, nr) - 1) << 16)
#define VAL_R  (((DT_PROP(DRV_PLL, r) - 1)) << 24)
#define VAL_OD ((DT_PROP(DRV_PLL, od) - 1) << 9)

#define CSDIS_OFFSET    (0x30) /* Clock source disable register */
#define CDDIS_OFFSET    (0x3c) /* Clock domain disable register */
#define CDDISSET_OFFSET (0x40) /* Clock domain disable set register */
#define CDDISCLR_OFFSET (0x44) /* Clock domain disable clear register */
#define GHVSRC_OFFSET   (0x48) /* CLK G,H,V and V2 source register */
#define RCLKSRC_OFFSET  (0x50) /* Clock source of RTI */
#define PLLCTL1_OFFSET  (0x70) /* PLL Control 1, contains NF, NR, R for PLL 1 */
#define PLLCTL2_OFFSET  (0x74) /* PLL Control 2, contains OD for PLL1 */
#define CLKCNTL_OFFSET  (0xd0) /* Clock control register */

#define PLL_DISABLE (1 << 1)
#define CLKSRC_PLL  (1 << 0) /* Clock source PLL */

#define PERIENA_BIT (1 << 8)

#define RTIDIV_BITPOS (8)
#define RTIDIV_BITS   (0x3)

#define VCLK2R_BITPOS (24)
#define VCLK2R_BITS   (0xf)
#define VCLKR_BITPOS  (16)
#define VCLKR_BITS    (0xf)

static int pll_init(void)
{
        uint32_t tmp;
        uintptr_t reg_base = DRV_REG;

        /* Configure OD, contained in the PLLCTL2 register. We aren't using
         * modulation, so the rest of the bits can be cleared to 0 */
        sys_write32(VAL_OD, reg_base + PLLCTL2_OFFSET);

        /* Configure NF, NR and R, contained in the PLLCTL1 register */
        tmp = VAL_NF | VAL_NR | VAL_R;
        sys_write32(tmp, reg_base + PLLCTL1_OFFSET);

        /* Unset the PLL disable bit in the clock source disable register */
        sys_clear_bits(reg_base + CSDIS_OFFSET, PLL_DISABLE);

        /* Set current source to PLL */
        sys_set_bits(reg_base + GHVSRC_OFFSET, CLKSRC_PLL);

        return 0;
}

static bool is_valid(unsigned int domain)
{
        switch (domain) {
        case TMS570_CLK_GCLK:
        case TMS570_CLK_HCLK:
        case TMS570_CLK_VCLK:
        case TMS570_CLK_VCLK2:
        case TMS570_CLK_RTI:
                return true;
        default:
                return false;
        }

        CODE_UNREACHABLE;
}

static int clock_on(const struct device *dev, clock_control_subsys_t subsys)
{
        uintptr_t reg = DRV_REG + CDDISCLR_OFFSET;
        unsigned int domain = *(unsigned int *)subsys;

        if (!is_valid(domain)) {
                return -EINVAL;
        }

        sys_set_bits(reg, 1 << domain);

        return 0;
}

static int clock_off(const struct device *dev, clock_control_subsys_t subsys)
{
        uintptr_t reg = DRV_REG + CDDISSET_OFFSET;
        unsigned int domain = *(unsigned int *)subsys;

        if (!is_valid(domain)) {
                return -EINVAL;
        }

        sys_set_bits(reg, 1 << domain);

        return 0;
}

static unsigned int vclk2_ratio(uintptr_t reg_base)
{
        uintptr_t reg = reg_base + CLKCNTL_OFFSET;
        return ((sys_read32(reg) >> VCLK2R_BITPOS) & VCLK2R_BITS) + 1;
}

static unsigned int vclk_ratio(uintptr_t reg_base)
{
        uintptr_t reg = reg_base + CLKCNTL_OFFSET;
        return ((sys_read32(reg) >> VCLKR_BITPOS) & VCLKR_BITS) + 1;
}

static unsigned int clock_ratio(unsigned int domain, uintptr_t reg_base)
{
        switch (domain) {
        case TMS570_CLK_GCLK:
        case TMS570_CLK_HCLK:
                return 1;
        case TMS570_CLK_VCLK:
                return vclk_ratio(reg_base);
        case TMS570_CLK_VCLK2:
                return vclk2_ratio(reg_base);
        case TMS570_CLK_RTI: {
                /* RTI is derived from VCLK by default. The TRM states that
                 * clock source *other* than VCLK require the RTI prescaler
                 * to ensure that the frequency of RTI to be at least
                 * 3 times less than VCLK.
                 * As RCLKSRC can only hold dividers that are multiple of twos,
                 * 3 is rounded up to 4.
                 *
                 * This is documented in section 10.5.2.3 of the TRM. */
                return vclk_ratio(reg_base) * 4;
        }
        }

        CODE_UNREACHABLE;
}

static int clock_get_rate(const struct device *dev, clock_control_subsys_t subsys, uint32_t *rate)
{
        uintptr_t reg_base = DRV_REG;
        unsigned int domain = *(unsigned int *)subsys;
        const struct tms570_clock_cfg *cfg = dev->config;

        if (!is_valid(domain)) {
                return -EINVAL;
        }

        *rate = cfg->clock_frequency / clock_ratio(domain, reg_base);

        return 0;
}

static int clock_init(const struct device *dev)
{
        uintptr_t reg_base = DRV_REG;

        /* Enable peripherals */
        sys_set_bits(reg_base + CLKCNTL_OFFSET, PERIENA_BIT);

        /* Disable all domains except GCLK, HCLK and VCLK. Other domains can
         * be enabled by a call to clock_control_on. */
        uint32_t on = (1 << TMS570_CLK_GCLK) | (1 << TMS570_CLK_HCLK) | (1 << TMS570_CLK_VCLK);
        sys_set_bits(reg_base + CDDIS_OFFSET, ~on);

        return 0;
}

static const struct clock_control_driver_api clock_api = {
        .on = clock_on,
        .off = clock_off,
        .get_rate = clock_get_rate,
};

static const struct tms570_clock_cfg clock_cfg = {
        .clock_frequency = DT_PROP(DRV_CLOCK, clock_frequency),
};
DEVICE_DT_DEFINE(DRV_CLOCK, &clock_init, NULL, NULL, &clock_cfg, PRE_KERNEL_1,
                 CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_api);

SYS_INIT(pll_init, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY);
