
#include <zephyr/types.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>

#define DT_DRV_COMPAT tms570_pll

#define DRV_REG DT_INST_REG_ADDR(0)
#define DRV_NR  DT_INST_PROP(0, nr)
#define DRV_NF  DT_INST_PROP(0, nf)
#define DRV_OD  DT_INST_PROP(0, od)
#define DRV_R   DT_INST_PROP(0, r)

#define VAL_NF (((DRV_NF - 1) << 8) << 0)
#define VAL_NR ((DRV_NR - 1) << 16)
#define VAL_R  (((DRV_R - 1)) << 24)
#define VAL_OD ((DRV_OD - 1) << 9)

#define CSDIS_OFFSET   (0x30) /* Clock source disable register */
#define CDDIS_OFFSET   (0x3c) /* Clock domain disable register */
#define PLLCTL1_OFFSET (0x70) /* PLL Control 1, contains NF, NR, R for PLL 1 */
#define PLLCTL2_OFFSET (0x74) /* PLL Control 2, contains OD for PLL1 */

#define PLL_DISABLE  (1 << 1)
#define RTI1_DISABLE (1 << 6)

static void configure_pll(void)
{
        uint32_t tmp;
        uintptr_t pll1mem, pll2mem, csdismem;

        pll1mem = DRV_REG + PLLCTL1_OFFSET;
        pll2mem = DRV_REG + PLLCTL2_OFFSET;
        csdismem = DRV_REG + CSDIS_OFFSET;

        /* Configure NF, NR and R, contained in the PLLCTL1 register */
        tmp = sys_read32(pll1mem);
        tmp |= VAL_NF | VAL_NR | VAL_R;
        sys_write32(tmp, pll1mem);

        /* Configure OD, contained in the PLLCTL2 register */
        tmp = sys_read32(pll2mem);
        tmp |= VAL_OD;
        sys_write32(tmp, pll2mem);

        /* Unset the PLL disable bit in the clock source disable register */
        tmp = sys_read32(csdismem);
        tmp &= ~PLL_DISABLE;
        sys_write32(tmp, csdismem);
}

static int clock_on(const struct device *dev, clock_control_subsys_t subsys)
{
        uint32_t tmp;
        uintptr_t addr = DRV_REG + CSDIS_OFFSET;

        tmp = sys_read32(addr);
        tmp &= ~PLL_DISABLE;
        sys_write32(tmp, addr);

        return 0;
}

static int clock_off(const struct device *dev, clock_control_subsys_t subsys)
{
        uint32_t tmp;
        uintptr_t addr = DRV_REG + CSDIS_OFFSET;

        tmp = sys_read32(addr);
        tmp |= PLL_DISABLE;
        sys_write32(tmp, addr);

        return 0;
}

static int clock_get_rate(const struct device *dev, clock_control_subsys_t subsys, uint32_t *api)
{
}

static int clock_init(const struct device *dev)
{
}

static const struct clock_control_driver_api clock_api = {
        .on = clock_on,
        .off = clock_off,
        .get_rate = clock_get_rate,
};
DEVICE_DT_INST_DEFINE(0, clock_init, NULL, NULL, NULL, PRE_KERNEL_1,
                      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_api);
