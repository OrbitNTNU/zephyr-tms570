
#include <zephyr/devicetree.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>

#define DT_DRV_COMPAT  tms570_timer
#define DRV_REG        DT_INST_REG_ADDR(0)
#define DRV_INTERRUPT  DT_INST_IRQN(0)
#define DRV_CLOCK_FREQ DT_INST_PROP(0, clock_frequency)

static const unsigned int drv_clock_id = DT_INST_CLOCKS_CELL(0, clk_id);
static const struct device *const clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0));

#define GCTRL_OFFSET      (0x00) /* Global control register */
#define CPU0_OFFSET       (0x18) /* Compare up counter 0 */
#define UDC0_OFFSET       (0x54) /* Update compare 0 */
#define SET_INTENA_OFFSET (0x80) /* Set interrupt enable */
#define CLR_INTENA_OFFSET (0x84) /* Clear interrupt enable */
#define INTFLAG_OFFSET    (0x88) /* Interrupt flag */

#define CPU0    (1)
#define CNT0EN  (1 << 0) /* Counter 0 enable */
#define INT0EN  (1 << 0) /* Interrupt 0 enable */
#define INT0CLR (1 << 0) /* Clear interrupt 0 */

#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

static volatile uint64_t ticks;

/* Tickless mode is not supported, and so this function can return 0 as
 * announce is called periodically on every interrupt anyways*/
uint32_t sys_clock_elapsed(void)
{
        return 0;
}

/* Calls to this function are ignored, as we are not supporting tickless mode */
void sys_clock_set_timeout(int32_t ticks, bool idle)
{
        ARG_UNUSED(ticks);
        ARG_UNUSED(idle);
}

uint32_t sys_clock_cycle_get_32(void)
{
        return (uint32_t)ticks;
}

uint64_t sys_clock_cycle_get_64(void)
{
        return ticks; 
}

static int set_udc0(void)
{
        uint32_t rate;
        int ret;

        ret = clock_control_get_rate(clk_ctrl, (clock_control_subsys_t)&drv_clock_id, &rate);
        if (ret < 0) {
                return ret;
        }

        rate /= CONFIG_SYS_CLOCK_TICKS_PER_SEC / (CPU0 + 1);
        sys_write32(rate, DRV_REG + UDC0_OFFSET);

        return 0;
}

static void timer_isr(void *arg)
{
        ARG_UNUSED(arg);

        ticks += k_ticks_to_cyc_floor32(1);

        /* Clear interrupt flag */
        sys_write32(INT0CLR, DRV_REG + INTFLAG_OFFSET);

        sys_clock_announce(1);
}

static int timer_init(void)
{
        int ret;

        if (!device_is_ready(clk_ctrl)) {
                return -ENODEV;
        }

        ret = clock_control_on(clk_ctrl, (clock_control_subsys_t)&drv_clock_id);
        if (ret < 0) {
                return ret;
        }

        /* Temporaribly disable counter */
        sys_write32(0, DRV_REG + GCTRL_OFFSET);

        /* Set periodicity of interrupts. This is derived from the equation:
         * t_compx = t_rticlk * (RTICPUCy + 1) * (RTIUDCPy) */
        sys_write32(CPU0, DRV_REG + CPU0_OFFSET);
        ret = set_udc0();
        if (ret < 0) {
                return ret;
        }

        IRQ_CONNECT(DRV_INTERRUPT, 0, timer_isr, NULL, 0);
        irq_enable(DRV_INTERRUPT);

        /* Enable interrupt on counter 0 */
        sys_write32(INT0EN, DRV_REG + SET_INTENA_OFFSET);

        /* Enable counter 0 */
        sys_write32(CNT0EN, DRV_REG + GCTRL_OFFSET);

        return 0;
}

SYS_INIT(timer_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
