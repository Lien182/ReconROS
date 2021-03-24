#include "reconos_calls.h"
#include "reconos_thread.h"

#include "ap_cint.h"

#define BLOCK_SIZE 2048

void sort_bubble(uint32 ram[BLOCK_SIZE]) {
	unsigned int i, j;
	uint32 tmp;
	for (i = 0; i < BLOCK_SIZE; i++) {
		for (j = 0; j < BLOCK_SIZE - 1; j++) {
			if (ram[j] > ram[j + 1]) {
				tmp = ram[j];
				ram[j] = ram[j + 1];
				ram[j + 1] = tmp;
			}
		}
	}
}

void sort_net(uint32 ram[BLOCK_SIZE]) {
	unsigned int i, k, stage;
	uint32 tmp;

	for(stage = 1; stage <= BLOCK_SIZE; stage++){
		k = (stage % 2 == 1) ? 0 : 1;
		for(i = k; i < BLOCK_SIZE - 1; i += 2){
			if (ram[i] > ram[i + 1]) {
				tmp = ram[i];
				ram[i] = ram[i + 1];
				ram[i + 1] = tmp;
			}
		}
	}
}

THREAD_ENTRY() {
	RAM(uint32, BLOCK_SIZE, ram);

	THREAD_INIT();

	while(1) {
		uint32 addr = MBOX_GET(resources_address);
		MEM_READ(addr, ram, BLOCK_SIZE * 4);

		sort_bubble(ram);

		MEM_WRITE(ram, addr, BLOCK_SIZE * 4);
		MBOX_PUT(resources_acknowledge, addr);
	}
}
