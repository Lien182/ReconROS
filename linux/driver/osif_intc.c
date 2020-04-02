/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Linux Driver - OSIF INTC (AXI FIFO)
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Daniel Borkmann, ETH Zürich
 *                 Sebastian Meisner, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  Driver for the OSIF interface. To speed up the FIFO
 *                 access, this driver now only includes the interrupt
 *                 handling, while the actual data access is done from
 *                 user space.
 *
 * ======================================================================
 */

#include "osif_intc.h"

#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>

/*
 * Register definitions as offset from base address
 *
 *   irq_enable_reg    - write to set interrupt enable
 */
#define IRQ_ENABLE_REG(hwt)    (0x00 + HWT_REG_OFFSET(hwt) * 4)

/*
 * Struct representing the interrupt controller device
 *
 *   name            - name to identify the device driver
 *   base_addr       - base addr (should always be BASE_ADDR)
 *   mem_size        - memory size (should always be MEM_SIZE)
 *   irq             - irq number (should always be IRQ)
 *
 *   mem             - pointer to the io memory
 *   wait            - wait queue for interrupts
 *   irq_reg         - array representing the interrupt signals
 *   irq_enable      - array representing the interrupt enables
 *   irq_break       - array indicating whether breaked in wait
 *
 *   mdev            - misc device data structure
 *
 *   lock            - spinlock for synchronization
 */
struct osif_intc_dev {
	char name[25];
	uint32_t base_addr;
	int mem_size;
	int irq;

	void __iomem *mem;
	wait_queue_head_t wait;
	uint32_t *irq_reg;
	uint32_t *irq_enable;
	uint32_t *irq_break;

	struct miscdevice mdev;

	spinlock_t lock;
};

static struct osif_intc_dev osif_intc;


/* == Low level functions ============================================== */

/*
 * Reads the interrupt register
 *
 *   dev - pointer to the interrupt controller struct
 */
static inline void read_interrupt(struct osif_intc_dev *dev) {
	int i;

	for (i = 0; i < DYNAMIC_REG_COUNT; i++) {
		dev->irq_reg[i] |= ioread32(dev->mem + i * 4) & dev->irq_enable[i];
	}
}
/*
 * Writes the interrupt enable registers
 *
 *   dev - pointer to the interrupt controller struct
 */
static inline void write_interrupt_enable(struct osif_intc_dev *dev) {
	int i;

	for (i = 0; i < DYNAMIC_REG_COUNT; i++) {
		iowrite32(dev->irq_enable[i], dev->mem + i * 4);
	}
}

/*
 * Enables an interrupt
 *
 *   dev - pointer to the interrupt controller struct
 *   irq - interrupt index to enable
 */
static inline void enable_interrupt(struct osif_intc_dev *dev,
                                    int irq) {
	dev->irq_reg[irq / 32] &= ~(0x1 << irq % 32);

	dev->irq_enable[irq / 32] |= 0x1 << irq % 32;
	write_interrupt_enable(dev);

	__printk(KERN_DEBUG "[reconos-osif-intc] "
	                    "enabling interrupt %d\n", irq);
}

/*
 * Disables an interrupt
 *
 *   dev - pointer to the interrupt controller struct
 *   irq - interrupt index to enable
 */
static inline void disable_interrupt(struct osif_intc_dev *dev,
                                     int irq) {
	dev->irq_enable[irq / 32] &= ~(0x1 << irq % 32);
	write_interrupt_enable(dev);

	__printk(KERN_DEBUG "[reconos-osif-intc] "
	                    "disabling interrupt %d\n", irq);
}

/*
 * Disables all active interrupts
 *
 *   dev - pointer to interrupt controller struct
 */
static inline void disable_interrupt_active(struct osif_intc_dev *dev) {
	int i;

	for (i = 0; i < DYNAMIC_REG_COUNT; i++) {
		dev->irq_enable[i] &= ~dev->irq_reg[i];
	}
	write_interrupt_enable(dev);
}

/*
 * Returns the status of an interrupt
 *
 *   dev - pointer to the interrupt controller struct
 *   irq - interrupt index to get
 */
static inline int get_interrupt(struct osif_intc_dev *dev,
                                int irq) {
	return (dev->irq_reg[irq / 32] >> irq % 32) & 0x1;
}

/*
 * Enables the break signal
 *
 *   dev - pointer to interrupt controller struct
 *   irq - interrupt index to set
 */
static inline void enable_break(struct osif_intc_dev *dev,
                                int irq) {
	dev->irq_break[irq / 32] |= 0x1 << irq % 32;
}

/*
 * Disables the break signal
 *
 *   dev - pointer to interrupt controller struct
 *   irq - interrupt index to set
 */
static inline void disable_break(struct osif_intc_dev *dev,
                                 int irq) {
	dev->irq_break[irq / 32] &= ~(0x1 << irq % 32);
}

/*
 * Returns the break signal of an interrupt
 *
 *   dev - pointer to interrupt controller struct
 *   irq - interrupt index to set
 */
static inline int get_break(struct osif_intc_dev *dev,
                             int irq) {
	return (dev->irq_break[irq / 32] >> irq % 32) & 0x1;
}

/* == File operations ================================================== */

/*
 * Function called when opening the device
 *
 *    @see kernel documentation
 */
static int osif_intc_open(struct inode *inode, struct file *filp) {
	filp->private_data = &osif_intc;

	return 0;
}

/*
 * Function called when issuing an ioctl
 *
 * @see kernel documentation
 */
static long osif_intc_ioctl(struct file *filp, unsigned int cmd,
                               unsigned long arg) {
	struct osif_intc_dev *dev;
	int irq;
	unsigned long ret, flags;

	dev = (struct osif_intc_dev *)filp->private_data;

	ret = copy_from_user(&irq, (unsigned int *)arg, sizeof(unsigned int));
	if (irq > NUM_HWTS) {
		__printk(KERN_WARNING "[reconos-osif-intc] "
		                      "index out of range, aborting wait ...\n");

		return 0;
	}

	switch (cmd) {
		case RECONOS_OSIF_INTC_WAIT:
			__printk(KERN_DEBUG "[reconos-osif-intc] "
			                    "waiting for interrupt %d\n", irq);

			spin_lock_irqsave(&dev->lock, flags);
			disable_break(dev, irq);
			enable_interrupt(dev, irq);
			spin_unlock_irqrestore(&dev->lock, flags);

			if (wait_event_interruptible(dev->wait, get_interrupt(dev, irq) || get_break(dev, irq)) < 0) {
				__printk(KERN_INFO "[reconos-osif-intc] "
				                   "interrupted in waiting, aborting ...\n");

				spin_lock_irqsave(&dev->lock, flags);
				disable_interrupt(dev, irq);
				spin_unlock_irqrestore(&dev->lock, flags);
			} else {
				__printk(KERN_DEBUG "[reconos-osif-intc] "
				                    "interrupted %d (irq: %d, signal: %d)\n",
				                    irq, get_interrupt(dev, irq), get_break(dev, irq));
			}
			break;

		case RECONOS_OSIF_INTC_BREAK:
			spin_lock_irqsave(&dev->lock, flags);
			enable_break(dev, irq);
			disable_interrupt(dev, irq);
			spin_unlock_irqrestore(&dev->lock, flags);

			wake_up_interruptible(&dev->wait);
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

/*
 * Struct for file operations to register driver
 *
 *    @see kernel documentation
 */
static struct file_operations osif_intc_fops = {
	.owner          = THIS_MODULE,
	.open           = osif_intc_open,
	.unlocked_ioctl = osif_intc_ioctl,
};


/* == Interrupt handling =============================================== */

/*
 * Interrupt handler for handling page faults
 *
 *   @see kernel documentation
 */
static irqreturn_t interrupt(int irq, void *data) {
	struct osif_intc_dev *dev;
	unsigned long flags;

	dev = (struct osif_intc_dev *)data;

	// read irqs and disable active ones
	spin_lock_irqsave(&dev->lock, flags);
	read_interrupt(dev);
	disable_interrupt_active(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	__printk(KERN_DEBUG "[reconos-osif-intc] "
	                    " osif interrupt: 0x%x\n", dev->irq_reg[0]);

	wake_up_interruptible(&dev->wait);

	return IRQ_HANDLED;
}


/* == Init and exit functions ========================================== */

/*
 * Struct for device tree matching
 */
static struct of_device_id osif_of_match[] =
{
    { .compatible = "upb,reconos-osif-intc-3.1"},
    {}
};

/*
 * @see header
 */
int osif_intc_init() {
	struct osif_intc_dev *dev = NULL;
	struct device_node *node = NULL;
	struct resource res;

	__printk(KERN_INFO "[reconos-osif] "
	                   "initializing driver ...\n");

	dev = &osif_intc;

	// get matching of node
	node = of_find_matching_node(NULL, osif_of_match);
	if (!node)
	{
		__printk(KERN_ERR "[reconos-osif] "
		                  "device tree node not found\n");
		goto of_failed;
	}
	__printk(KERN_INFO "[reconos-osif] "
	                   "found device %s\n", node->name);

	// set some general information of intc
	strncpy(dev->name, "reconos-osif-intc", 25);

	init_waitqueue_head(&dev->wait);
	spin_lock_init(&dev->lock);

	// allocating interrupt-register
	dev->irq_reg = kcalloc(DYNAMIC_REG_COUNT, sizeof(uint32_t), GFP_KERNEL);
	if (!dev->irq_reg) {
		__printk(KERN_WARNING "[reconos-osif-intc] "
		                      "cannot allocate irq-memory\n");
		goto irqreg_failed;
	}

	dev->irq_enable = kcalloc(DYNAMIC_REG_COUNT, sizeof(uint32_t), GFP_KERNEL);
	if (!dev->irq_enable) {
		__printk(KERN_WARNING "[reconos-osif-intc] "
		                      "cannot allocate irq-enable\n");
		goto irqenable_failed;
	}

	dev->irq_break = kcalloc(DYNAMIC_REG_COUNT, sizeof(uint32_t), GFP_KERNEL);
	if (!dev->irq_break) {
		__printk(KERN_WARNING "[reconos-osif-intc] "
		                      "cannot allocate irq-break\n");
		goto irqbreak_failed;
	}

	// getting address from device tree
	if (of_address_to_resource(node, 0, &res))
	{
		__printk(KERN_ERR "[reconos-osif] "
	                      "address could not be determined\n");
		goto req_failed;
	}
	dev->base_addr = res.start;
	dev->mem_size = res.end - res.start + 1;
	__printk(KERN_INFO "[reconos-osif] "
	                   "found memory at 0x%08x with size 0x%x\n",
	                   dev->base_addr, dev->mem_size);

	// allocation io memory to read intc registers
	if (!request_mem_region(dev->base_addr, dev->mem_size, dev->name)) {
		__printk(KERN_WARNING "[reconos-osif-intc] "
		                      "memory region busy\n");
		goto req_failed;
	}

	dev->mem = ioremap(dev->base_addr, dev->mem_size);
	if(!dev->mem) {
		__printk(KERN_WARNING "[reconos-osif-intc] "
		                      "ioremap failed\n");
		goto map_failed;
	}

	// disable all interrupts to avoid useless interrupts
	write_interrupt_enable(dev);

	// getting interrupt number from device tree
	dev->irq = irq_of_parse_and_map(node, 0);
	if (!dev->irq)
	{
		__printk(KERN_ERR "[reconos-osif] "
		                  "irq could not be determined\n");
		goto irq_failed;
	}
	__printk(KERN_INFO "[reconos-osif] "
	                   "found interrupt %d\n", dev->irq);

	// requesting interrupt
	if(request_irq(dev->irq, interrupt, 0, "reconos-osif-intc", dev)) {
		__printk(KERN_WARNING "[reconos-osif-intc] "
		                      "can't get irq\n");
		goto irq_failed;
	}

	// registering misc device
	dev->mdev.minor = MISC_DYNAMIC_MINOR;
	dev->mdev.fops = &osif_intc_fops;
	dev->mdev.name = dev->name;

	if (misc_register(&dev->mdev) < 0) {
		__printk(KERN_WARNING "[reconos-osif-intc] "
		                      "error while registering misc-device\n");
		goto reg_failed;
	}


	__printk(KERN_INFO "[reconos-osif-intc] "
	                   "registered interrupt controller\n");

	goto out;

reg_failed:
	free_irq(dev->irq, dev);

irq_failed:
	iounmap(dev->mem);

map_failed:
	release_mem_region(dev->base_addr, dev->mem_size);

req_failed:
	kfree(dev->irq_break);

irqbreak_failed:
	kfree(dev->irq_enable);

irqenable_failed:
	kfree(dev->irq_reg);

irqreg_failed:
of_failed:
	return -1;

out:
	return 0;
}

/*
 * @see header
 */
int osif_intc_exit() {
	struct osif_intc_dev *dev = &osif_intc;

	__printk(KERN_INFO "[reconos-osif-intc] "
	                   "removing driver ...\n");

	misc_deregister(&dev->mdev);

	free_irq(dev->irq, dev);

	iounmap(dev->mem);
	release_mem_region(dev->base_addr, dev->mem_size);

	kfree(dev->irq_enable);
	kfree(dev->irq_reg);
	kfree(dev->irq_break);

	return 0;
}
