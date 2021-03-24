#include <limits.h>
#include <stdio.h>

#include "reconos_thread.h"
#include "reconos_calls.h"

#include "../../application/mmp.h"


void std_matrix_mul(int *i_matrix_a, int *i_matrix_b, int *o_matrix_c, int matrix_size) {
	int i, j, k;
	printf("std_matrix_mul i:");fflush(0);
	for (i=0; i<matrix_size; ++i) {
		printf(" %i ", i);fflush(0);
		for (j=0; j<matrix_size; ++j) {
			int temp = 0;
			int pos = i*matrix_size;
			for (k=0; k<matrix_size; ++k) {
				temp += i_matrix_a[pos+k]*i_matrix_b[k*matrix_size+j];
			}
			o_matrix_c[pos+j] = temp;
		}
	}
	printf ("\n");
}

void *rt_matrixmul(void *data) {
	unsigned int ret;
	int **ret2;
	int flag;

	while (1) {
		flag = 0;
		while(!flag){
			flag = MBOX_TRYGET(resources_address,ret);
		}

		if (ret == UINT_MAX) {
			THREAD_EXIT();
		}
		ret2 = (int **)ret;
		std_matrix_mul(ret2[0], ret2[1], ret2[2], STD_MMP_MATRIX_SIZE);
		MBOX_PUT(resources_acknowledge, (int)ret2[2]);
	}
	return NULL;
}
