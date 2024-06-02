
#include <zephyr/arch/arm/irq.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/interrupt_controller/intc_vim.h>

#define DT_DRV_COMPAT tms570_intc
#define DRV_REG       DT_INST_REG_ADDR(0)

#define IRQINDEX_OFFSET  (0x0) /* Index of currently active IRQ */
#define FIQINDEX_OFFSET  (0x4) /* Index of currently active FIQ */
#define REQENASET_OFFSET (0x30) /* Set/enable interrupt */
#define REQENACLR_OFFSET (0x40) /* Clear/disable interrupt */

/* Number of interrupts per group. This is specified in the technical
 * reference manual. */
#define IRQ_PER (32)

static void set_irq_bit(unsigned int irq, uintptr_t base_offset)
{
        unsigned int offset = irq / IRQ_PER * sizeof(uint32_t);
        unsigned int bit = irq % IRQ_PER;

        sys_write32(1 << bit, DRV_REG + base_offset + offset);
}

static int get_irq_bit(unsigned int irq, uintptr_t base_offset)
{
        unsigned int offset = (irq / IRQ_PER) * sizeof(uint32_t);
        unsigned int bit = irq % IRQ_PER;

        return sys_read32(DRV_REG + base_offset + offset) & (1 << bit);
}

void z_soc_irq_init(void)
{
}

void z_soc_irq_enable(unsigned int irq)
{
        set_irq_bit(irq, REQENASET_OFFSET);
}

void z_soc_irq_disable(unsigned int irq)
{
        set_irq_bit(irq, REQENACLR_OFFSET);
}

int z_soc_irq_is_enabled(unsigned int irq)
{
        return get_irq_bit(irq, REQENASET_OFFSET);
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, unsigned int flags)
{
        /* TODO */
        ARG_UNUSED(irq);
        ARG_UNUSED(prio);
        ARG_UNUSED(flags);
}

unsigned int z_soc_irq_get_active(void)
{
        /* 1 is subtracted from the IRQ, as 0 is used for unused and/or phantom
         * interrupt, which is not relevant to us. */
        uint32_t irq;

        irq = sys_read32(DRV_REG + FIQINDEX_OFFSET);
        if (irq != 0) {
                /* Disable current interrupt line. This will be re-enabled by
                 * the call to @ref z_soc_irq_eoi */
                z_soc_irq_disable(irq - 1);
                return irq - 1;
        }

        irq = sys_read32(DRV_REG + IRQINDEX_OFFSET);
        if (irq != 0) {
                /* Disable current interrupt line. This will be re-enabled by
                 * the call to @ref z_soc_irq_eoi */
                z_soc_irq_disable(irq - 1);
                return irq - 1;
        }

        return CONFIG_NUM_IRQS + 1;
}

void z_soc_irq_eoi(unsigned int irq)
{
        /* Re-enable the interrupt line that was disabled by the call to @ref
         * z_soc_irq_get_active */
        z_soc_irq_enable(irq);
}
