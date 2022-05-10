/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Architecture specific code - Linux
 *
 *   project:      ReconOS
 *   author:       Christoph Rüthing, University of Paderborn
 *   description:  Functions needed for ReconOS which are architecure
 *                 specific and are implemented here.
 *
 * ======================================================================
 */

#define dbg(...) printf(__VA_ARGS__); fflush(stdout)

#if defined(RECONOS_OS_linux_x86)

#include "arch64.h"
#include "../utils.h"

#include "arch_linux_kernel.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "pthread.h"

#define PROC_CONTROL_DEV "/dev/reconos-proc-control"
#define OSIF_INTC_DEV "/dev/reconos-osif-intc"

unsigned int NUM_HWTS = 0;


/* == OSIF related functions ============================================ */

#define OSIF_FIFO_BASE_ADDR       0xA0100000  //for US /lc
#define OSIF_FIFO_BASE_SIZE       0x10000
#define OSIF_FIFO_MEM_SIZE        0x20
#define OSIF_FIFO_RECV_REG        0
#define OSIF_FIFO_SEND_REG        1
#define OSIF_FIFO_RECV_STATUS_REG 2
#define OSIF_FIFO_SEND_STATUS_REG 3

#define OSIF_FIFO_RECV_STATUS_EMPTY_MASK 0x8000000000000000
#define OSIF_FIFO_SEND_STATUS_FULL_MASK  0x8000000000000000

#define OSIF_FIFO_RECV_STATUS_FILL_MASK 0xFFFF
#define OSIF_FIFO_SEND_STATUS_REM_MASK  0xFFFF

struct osif_fifo_dev {
	unsigned int index;

	volatile uint64_t *ptr;

	unsigned int fifo_fill;
	unsigned int fifo_rem;
};

int osif_intc_fd;
struct osif_fifo_dev *osif_fifo_dev;

int reconos_osif_open(int num) {
	return 0;
}

static inline unsigned int osif_fifo_hw2sw_fill(struct osif_fifo_dev *dev) {
	return 0;

}

static inline unsigned int osif_fifo_sw2hw_rem(struct osif_fifo_dev *dev) {
	return 0;
}

uint64_t reconos_osif_read(int fd) {
	uint64_t data = 0;

	return data;
}

void reconos_osif_write(int fd, uint64_t data) {

}

void reconos_osif_break(int fd) {

}

void reconos_osif_close(int fd) {

}


/* == Proc control related functions ==================================== */

int proc_control_fd;

int reconos_proc_control_open() {
	return proc_control_fd;
}

int reconos_proc_control_get_num_hwts(int fd) {
	int data = 0;


	return data;
}

int reconos_proc_control_get_tlb_hits(int fd) {
	int data = 0;

	return data;
}

int reconos_proc_control_get_tlb_misses(int fd) {
	int data = 0;

	return data;
}

uint64_t reconos_proc_control_get_fault_addr(int fd) {
    uint64_t data = 0;

	return data;
}

void reconos_proc_control_clear_page_fault(int fd) {
}

void reconos_proc_control_set_pgd(int fd) {
}

void reconos_proc_control_sys_reset(int fd) {
}

void reconos_proc_control_hwt_reset(int fd, int num, int reset) {
}

void reconos_proc_control_hwt_signal(int fd, int num, int sig) {
}

void reconos_proc_control_cache_flush(int fd) {
}

void reconos_proc_control_close(int fd) {
}


/* == Clock related functions =========================================== */

//#define CLOCK_BASE_ADDR    0x69E00000
#define CLOCK_BASE_ADDR    0xA0040000 //for US /lc
#define CLOCK_BASE_SIZE    0x10000
#define CLOCK_MEM_SIZE     0x20

#define CLOCK_REG_HIGH_BIT(t) (((t) & 0x0000003F) << 6)
#define CLOCK_REG_LOW_BIT(t)  (((t) & 0x0000003F) << 0)
#define CLOCK_REG_EDGE_BIT    0x00800000
#define CLOCK_REG_COUNT_BIT   0x00400000

struct clock_dev {
	volatile uint32_t *ptr;
};

struct clock_dev *clock_dev;

static inline void clock_write(struct clock_dev *dev, int clk, uint32_t reg) {
}

int reconos_clock_open() {
	debug("[reconos-clock] "
	      "opening ...\n");

	return 0;
}

void reconos_clock_set_divider(int fd, int clk, int divd) {

}

void reconos_clock_close(int fd) {
	debug("[reconos-clock] "
	      "closing ...\n");
}


/* == Reconfiguration related functions ================================= */

#if defined(RECONOS_BOARD_zedboard_c) || defined(RECONOS_BOARD_zedboard_c)

int is_configured = 0;
pthread_mutex_t mutex;

inline void init_xdevcfg() {

}

int load_partial_bitstream(uint32_t *bitstream, unsigned int bitstream_length) {
	int fd;
	char d = '1';


	return 0;
}

#elif RECONOS_BOARD_ml605

int load_partial_bitstream(uint32_t *bitstream, unsigned int bitstream_length) {
	panic("NOT IMPLEMENTED YET\n");

	return 0;
}

#endif


/* == Initialization function =========================================== */

void reconos_drv_init() {
	
}

#endif