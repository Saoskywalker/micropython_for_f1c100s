#include "irq_misc.h"
#include <arm32.h>
#include <io.h>
#include <uart-16550.h>

typedef struct {
	irq_handle irq_hdl_proc;
	int tag;
	void *arg;
}irq_handle_stu;
static irq_handle_stu f1c100s_irq_vector[160]={0};

//no complete and test
void INTC_Set(void)
{
	// *(void* volatile *)0x38 = (void* volatile)__timer0_irq;//设置IRQ中断处理程序
    /* clear enable */
	write32(IRQ_ADDR_BASE+IRQ_ENABLE0, 0x00000000);
	write32(IRQ_ADDR_BASE+IRQ_ENABLE1, 0x00000000);
    /* mask interrupt */
	write32(IRQ_ADDR_BASE+IRQ_MASK0, 0xFFFFFFFF);
	write32(IRQ_ADDR_BASE+IRQ_MASK1, 0xFFFFFFFF);
    /* clear pending */
	write32(IRQ_ADDR_BASE+IRQ_PEND0, 0x00000000);
	write32(IRQ_ADDR_BASE+IRQ_PEND1, 0x00000000);
    /* set priority */
	write32(IRQ_ADDR_BASE+IRQ_RESP0, 0x00000000);
	write32(IRQ_ADDR_BASE+IRQ_RESP1, 0x00000000);
    /* close fiq interrupt */
	write32(IRQ_ADDR_BASE+IRQ_FORCE0, 0x00000000);
	write32(IRQ_ADDR_BASE+IRQ_FORCE1, 0x00000000);

	//开启中断使能
    // asm( 
    //     "temp:\n\t" 
    //     ".word 0\n\t"
    //     "mrs     temp, cpsr\n\t"  
    //     "and     temp, temp, 0x7f\n\t"  
    //     "msr     cpsr_c, temp\n\t"  
    // );  
}

static void irq_f1c100s_enable(int type)
{
	int gpio_irq;
	int offset;
	if(type < 64)
	{
		gpio_irq = 0;
		offset = type;
	}
	else if(type < 96)
	{
		gpio_irq = 1;
		offset = IRQ_GPIOD;
	}
	else if(type < 128)
	{
		gpio_irq = 2;
		offset = IRQ_GPIOE;
	}
	else
	{
		gpio_irq = 3;
		offset = IRQ_GPIOF;
	}

	int irq =  offset;
	unsigned int val;
	val = read32(IRQ_ADDR_BASE + IRQ_ENABLE0 + (irq / 32) * 4);
	val |= 1 << (irq % 32);
	write32(IRQ_ADDR_BASE + IRQ_ENABLE0 + (irq / 32) * 4, val);

	val = read32(IRQ_ADDR_BASE + IRQ_MASK0 + (irq / 32) * 4);
	val &= ~(1 << (irq % 32));
	write32(IRQ_ADDR_BASE+ IRQ_MASK0 + (irq / 32) * 4, val);

	if(gpio_irq)
	{

	}
}

//static void irq_f1c100s_disable(int offset)
//{
//	//struct irq_f1c100s_pdata_t * pdat = (struct irq_f1c100s_pdata_t *)chip->priv;
//	int irq = chip->base + offset;
//	u32_t val;
//
//	val = read32(pdat->virt + IRQ_ENABLE0 + (irq / 32) * 4);
//	val &= ~(1 << (irq % 32));
//	write32(pdat->virt + IRQ_ENABLE0 + (irq / 32) * 4, val);
//
//	val = read32(pdat->virt + IRQ_MASK0 + (irq / 32) * 4);
//	val |= 1 << (irq % 32);
//	write32(pdat->virt + IRQ_MASK0 + (irq / 32) * 4, val);
//}

void default_irq_proc(void *arg)
{
	// printf("no callback function for irq\n\r");
}

int request_irq(int irqno,irq_handle callback,void *arg)
{
	int ret = 0;
	arm32_interrupt_disable();
	irq_f1c100s_enable(irqno);
	if(!callback)
	{
		f1c100s_irq_vector[irqno].tag = irqno;
		f1c100s_irq_vector[irqno].arg = 0;
		f1c100s_irq_vector[irqno].irq_hdl_proc = default_irq_proc;
	}
	else
	{
		f1c100s_irq_vector[irqno].tag = irqno;
		f1c100s_irq_vector[irqno].arg = arg;
		f1c100s_irq_vector[irqno].irq_hdl_proc = callback;
	}
	arm32_interrupt_enable();
	return ret;
}

void interrupt_handle_exception(void * regs)
{
//	struct device_t * pos, * n;
//	struct irqchip_t * chip;
//
//	list_for_each_entry_safe(pos, n, &__device_head[DEVICE_TYPE_IRQCHIP], head)
//	{
//		chip = (struct irqchip_t *)(pos->priv);
//		if(chip->dispatch)
//			chip->dispatch(chip);
//	}
	unsigned int irq, hwirq;

	hwirq = read32(IRQ_ADDR_BASE + IRQ_VECTOR) >> 2;

	// printf("irq\n\r");
	while (hwirq != 0)
	{
		irq =  hwirq;
		//handle_IRQ(irq, regs);

		if(f1c100s_irq_vector[irq].irq_hdl_proc)
		{
			f1c100s_irq_vector[irq].irq_hdl_proc(f1c100s_irq_vector[irq].arg);
		}
		else
		{
			// printf("no irq:%d handle proc \n",irq);
		}

		hwirq = read32(IRQ_ADDR_BASE + IRQ_VECTOR) >> 2;
	}
}
