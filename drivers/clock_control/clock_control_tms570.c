
#include <zephyr/types.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/device_mmio.h>

#include <orbit/dt-bindings/clock_control/clock_control_tms570.h>

struct tms570_pll_cfg {
        DEVICE_MMIO_ROM;
        uint8_t nr;
        uint8_t nf;
        uint8_t od;
        uint8_t r;
};

struct tms570_pll_data {
        DEVICE_MMIO_RAM;
};

struct tms570_clock_cfg {
        DEVICE_MMIO_ROM;
        uint32_t clock_frequency;
};

struct tms570_clock_data {
        DEVICE_MMIO_RAM;
};

/* PLL scaling coefficients */
#define VAL_NF(nf) (((nf - 1) << 8) << 0)
#define VAL_NR(nr) ((nr - 1) << 16)
#define VAL_R(r)   (((r - 1)) << 24)
#define VAL_OD(od) ((od - 1) << 9)

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

static int pll_init(const struct device *dev)
{
        uint32_t tmp;
        const struct tms570_pll_cfg *cfg = dev->config;

        DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);

        /* Configure NF, NR and R, contained in the PLLCTL1 register */
        tmp = VAL_NF(cfg->nf) | VAL_NR(cfg->nr) | VAL_R(cfg->r);
        sys_set_bits(reg_base + PLLCTL1_OFFSET, tmp);

        /* Configure OD, contained in the PLLCTL2 register */
        sys_set_bits(reg_base + PLLCTL2_OFFSET, VAL_OD(cfg->od));

        /* Unset the PLL disable bit in the clock source disable register */
        sys_clear_bits(reg_base + CSDIS_OFFSET, PLL_DISABLE);

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
        uintptr_t reg = DEVICE_MMIO_GET(dev) + CDDISCLR_OFFSET;
        unsigned int domain = *(unsigned int *)subsys;

        if (!is_valid(domain)) {
                return -EINVAL;
        }

        sys_set_bits(reg, 1 << domain);

        return 0;
}

static int clock_off(const struct device *dev, clock_control_subsys_t subsys)
{
        uintptr_t reg = DEVICE_MMIO_GET(dev) + CDDISSET_OFFSET;
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
        uint32_t vclk = ((sys_read32(reg) >> VCLKR_BITPOS) & VCLKR_BITS) + 1;
        uint32_t vclk2 = vclk2_ratio(reg_base);

        /* VCLK2 must always be >= 2 * VCLK */
        if (vclk < vclk2 * 2) {
                return vclk2 * 2;
        }

        return vclk;
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
                uint32_t rti = sys_read32(reg_base + RCLKSRC_OFFSET);
                rti = ((rti >> RTIDIV_BITPOS) & RTIDIV_BITS) + 1;

                return vclk_ratio(reg_base) * rti;
        }
        }

        CODE_UNREACHABLE;
}

static int clock_get_rate(const struct device *dev, clock_control_subsys_t subsys, uint32_t *api)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);
        const struct tms570_clock_cfg *cfg = dev->config;

        return cfg->clock_frequency / clock_ratio(*(unsigned int *)subsys, reg_base);
}

static int clock_init(const struct device *dev)
{
        DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);

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

#define DT_DRV_COMPAT tms570_clock
static const struct tms570_clock_cfg clock_cfg = {
        DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
        .clock_frequency = DT_INST_PROP(0, clock_frequency),
};
static struct tms570_clock_data clock_data;
DEVICE_DT_INST_DEFINE(0, &clock_init, NULL, &clock_data, &clock_cfg, PRE_KERNEL_1,
                      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_api);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT tms570_pll
static const struct tms570_pll_cfg pll_cfg = {
        DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
        .r = DT_INST_PROP(0, r),
        .nf = DT_INST_PROP(0, nf),
        .nr = DT_INST_PROP(0, nr),
        .od = DT_INST_PROP(0, od),
};
static struct tms570_pll_data pll_data;
DEVICE_DT_INST_DEFINE(0, &pll_init, NULL, &pll_data, &pll_cfg, PRE_KERNEL_1,
                      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, NULL);
