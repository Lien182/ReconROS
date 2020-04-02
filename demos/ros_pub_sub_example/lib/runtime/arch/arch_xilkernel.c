/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Architecture specific code - Microblaze, Xilkernel
 *
 *   project:      ReconOS
 *   author:       Christoph Rüthing, University of Paderborn
 *   description:  Functions needed for ReconOS which are architecure
 *                 specific and are implemented here.
 *
 * ======================================================================
 */


#if defined(RECONOS_BOARD_ml605)
#if defined(RECONOS_OS_xilkernel)

#include "arch.h"

#include "xhwicap.h"

#include <pthread.h>
#include <semaphore.h>
#include <sys/intr.h>

unsigned int NUM_HWTS = 0;


/* == OSIF related functions ============================================ */

#define OSIF_INTC_BASE_ADDR  0x7B400000
#define OSIF_INTC_MEM_SIZE   0x10000
#define OSIF_INTC_IRQ        3

#define OSIF_FIFO_BASE_ADDR       0x75A00000
#define OSIF_FIFO_MEM_SIZE        0x10
#define OSIF_FIFO_RECV_REG        0
#define OSIF_FIFO_SEND_REG        1
#define OSIF_FIFO_RECV_STATUS_REG 2
#define OSIF_FIFO_SEND_STATUS_REG 3

#define OSIF_FIFO_RECV_STATUS_EMPTY_MASK 0x1 << 31
#define OSIF_FIFO_SEND_STATUS_FULL_MASK  0x1 << 31

#define OSIF_FIFO_RECV_STATUS_FILL_MASK 0xFFFF
#define OSIF_FIFO_SEND_STATUS_REM_MASK  0xFFFF

struct osif_fifo_dev {
	unsigned int index;

	volatile uint32_t *ptr;

	unsigned int fifo_fill;
	sem_t wait;
};

struct osif_intc_dev {
	volatile uint32_t *ptr;

	uint32_t irq_reg;
	uint32_t irq_enable;

	unsigned int irq_enable_count;

	pthread_mutex_t lock;
};

struct osif_fifo_dev *osif_fifo_dev;
struct osif_intc_dev osif_intc_dev;

int reconos_osif_open(int num) {
	if (num < 0 || num >= NUM_HWTS)
		return -1;
	else
		return num;
}

static inline unsigned int osif_fifo_hw2sw_fill(struct osif_fifo_dev *dev) {
	uint32_t reg;

	reg = dev->ptr[OSIF_FIFO_RECV_STATUS_REG];
	if (reg & OSIF_FIFO_RECV_STATUS_EMPTY_MASK)
		return 0;
	else
		return (reg & OSIF_FIFO_RECV_STATUS_EMPTY_MASK) + 1;
}

static inline void osif_intc_enable_interrupt(struct osif_intc_dev *dev, unsigned int irq) {
	pthread_mutex_lock(&dev->lock);

	dev->irq_enable |= 0x1 << irq % 32;
	dev->ptr[0] = dev->irq_enable;

	dev->irq_enable_count++;

	pthread_mutex_unlock(&dev->lock);
}

static inline void osif_intc_disable_interrupt(struct osif_intc_dev *dev, unsigned int irq) {
	pthread_mutex_lock(&dev->lock);

	dev->irq_enable &= ~(0x1 << irq % 32);
	dev->ptr[0] = dev->irq_enable;

	dev->irq_enable_count--;

	pthread_mutex_unlock(&dev->lock);
}

void osif_intc_interrupt(void *arg) {
	struct osif_intc_dev *dev = &osif_intc_dev;
	int i;

	dev->irq_reg = dev->ptr[0] & dev->irq_enable;

	for (i = 0; i < 32; i++) {
		if ((dev->irq_reg >> i) & 0x1) {
			osif_intc_disable_interrupt(dev, i);

			osif_fifo_dev[i].fifo_fill = osif_fifo_hw2sw_fill(&osif_fifo_dev[i]);
			sem_post(&osif_fifo_dev[i].wait);
		}
	}
}

uint32_t reconos_osif_read(int fd) {
	struct osif_fifo_dev *dev = &osif_fifo_dev[fd];

	if (dev->fifo_fill == 0) {
		dev->fifo_fill = osif_fifo_hw2sw_fill(dev);

		while (dev->fifo_fill == 0) {
			osif_intc_enable_interrupt(&osif_intc_dev, fd);
			sem_wait(&dev->wait);
		}
	}

	dev->fifo_fill--;
	return dev->ptr[OSIF_FIFO_RECV_REG];
}

void reconos_osif_write(int fd, uint32_t data) {
	uint32_t reg;

	do {
		reg = osif_fifo_dev[fd].ptr[OSIF_FIFO_SEND_STATUS_REG];
	} while (reg & OSIF_FIFO_SEND_STATUS_FULL_MASK);

	osif_fifo_dev[fd].ptr[OSIF_FIFO_SEND_REG] = data;
}

void reconos_osif_close(int fd) {
	// nothing to do here
}


/* == Proc control related functions ==================================== */

#define PROC_CONTROL_BASE_ADDR 0x6FE00000
#define PROC_CONTROL_MEM_SIZE  0x10000

#define PROC_CONTROL_NUM_HWTS_REG        0
#define PROC_CONTROL_PGD_ADDR_REG        1
#define PROC_CONTROL_PAGE_FAULT_ADDR_REG 2
#define PROC_CONTROL_TLB_HITS_REG        3
#define PROC_CONTROL_TLB_MISSES_REG      4
#define PROC_CONTROL_SYS_RESET_REG       5
#define PROC_CONTROL_HWT_RESET_REG       6

struct proc_control_dev {
	volatile uint32_t *ptr;
	uint32_t *hwt_reset;
	size_t hwt_reset_count;
};

struct proc_control_dev proc_control_dev;

int reconos_proc_control_open() {
	// nothing to do here
	return 0;
}

int reconos_proc_control_get_num_hwts(int fd) {
	return NUM_HWTS;
}

int reconos_proc_control_get_tlb_hits(int fd) {
	return proc_control_dev.ptr[PROC_CONTROL_TLB_HITS_REG];
}

int reconos_proc_control_get_tlb_misses(int fd) {
	return proc_control_dev.ptr[PROC_CONTROL_TLB_MISSES_REG];
}

uint32_t reconos_proc_control_get_fault_addr(int fd) {
	// nothing to do here since no MMU present
	while(1);
}

void reconos_proc_control_clear_page_fault(int fd) {
	// nothing to do here since no MMU present
}

void reconos_proc_control_set_pgd(int fd) {
	// nothing to do here since no MMU present
}

void reconos_proc_control_sys_reset(int fd) {
	int i;

	for (i = 0; i < NUM_HWTS; i++)
		proc_control_dev.hwt_reset[i] = 0xFFFFFFFF;
	proc_control_dev.ptr[PROC_CONTROL_SYS_RESET_REG] = 0;
}

void reconos_proc_control_hwt_reset(int fd, int num, int reset) {
	if (num >= 0 && num < NUM_HWTS) {
		if (reset)
			proc_control_dev.hwt_reset[num / 32] |= 0x1 << num % 32;
		else
			proc_control_dev.hwt_reset[num / 32] &= ~(0x1 << num % 32);

		proc_control_dev.ptr[PROC_CONTROL_HWT_RESET_REG + num / 32] = proc_control_dev.hwt_reset[num / 32];
	}
}

void reconos_proc_control_cache_flush(int fd) {
	int i;
	int baseaddr, bytesize,linelen;

	// these parameters need to be adjusted to the architecture
	// C_DCACHE_BASEADDR
	baseaddr = 0x20000000;
	// C_DCACHE_BYTE_SIZE
	bytesize = 64 * 1024;
	// C_DCACHE_LINE_LEN * 4
	linelen = 4 * 4;

	for (i = 0; i < bytesize; i += linelen)
		asm volatile ("wdc.flush %0, %1;" :: "d" (baseaddr), "d" (i));
}

void reconos_proc_control_close(int fd) {
	// nothing to do here
}


/* == Reconfiguration related functions ================================= */

int is_configured = 0;
pthread_mutex_t icap_mutex;
XHwIcap icap;

inline void init_icap() {
	XHwIcap_Config *icap_cfg;
	int res;

	if (!is_configured) {
		pthread_mutex_init(&icap_mutex, NULL);

		icap_cfg = XHwIcap_LookupConfig(0);
		res = XHwIcap_CfgInitialize(&icap, icap_cfg,icap_cfg->BaseAddress);
		if (res != XST_SUCCESS) {
			panic("error initializing icap, aborting.\n");
			exit(0);
		}

		is_configured = 1;
	}
}

int load_partial_bitstream(uint32_t *bitstream, unsigned int bitstream_length) {
	int res;

	pthread_mutex_lock(&icap_mutex);

	init_icap();

	res = XHwIcap_DeviceWrite(&icap, bitstream, bitstream_length);

	pthread_mutex_unlock(&icap_mutex);

	if (res == XST_SUCCESS)
		return 0;
	else
		return -1;
}


/* == Initialization function =========================================== */

void reconos_drv_init() {
	int i;

	proc_control_dev.ptr = (uint32_t *)PROC_CONTROL_BASE_ADDR;
	NUM_HWTS = proc_control_dev.ptr[PROC_CONTROL_NUM_HWTS_REG];


	// allocate memory for HWT resets
	proc_control_dev.hwt_reset_count = NUM_HWTS / 32 + 1;
	proc_control_dev.hwt_reset = (uint32_t*)malloc(proc_control_dev.hwt_reset_count * sizeof(uint32_t));
	if (!proc_control_dev.hwt_reset)
		panic("[reconos-proc-control] failed to allocate memory\n");


	// reset entire system
	for (i = 0; i < proc_control_dev.hwt_reset_count; i++)
		proc_control_dev.hwt_reset[i] = 0xFFFFFFFF;
	proc_control_dev.ptr[PROC_CONTROL_SYS_RESET_REG] = 0;


	// allocate and initialize intc device
	osif_intc_dev.ptr = (uint32_t *)OSIF_INTC_BASE_ADDR;
	osif_intc_dev.irq_reg = 0;
	osif_intc_dev.irq_enable = 0;
	osif_intc_dev.ptr[0] = osif_intc_dev.irq_enable;
	osif_intc_dev.irq_enable_count = 0;
	pthread_mutex_init(&osif_intc_dev.lock, NULL);

	register_int_handler(OSIF_INTC_IRQ, osif_intc_interrupt, NULL);
	enable_interrupt(OSIF_INTC_IRQ);


	// allocate and initialize osif devices
	osif_fifo_dev = (struct osif_fifo_dev*)malloc(NUM_HWTS * sizeof(struct osif_fifo_dev));
	if (!osif_fifo_dev)
		panic("[reconos-osif] failed to allocate memory\n");

	for (i = 0; i < NUM_HWTS; i++) {
		osif_fifo_dev[i].index = i;
		osif_fifo_dev[i].ptr = (uint32_t *)(OSIF_FIFO_BASE_ADDR + i * OSIF_FIFO_MEM_SIZE);
		osif_fifo_dev[i].fifo_fill = 0;
		sem_init(&osif_fifo_dev[i].wait, 0, 0);
	}
}

#endif
#endif