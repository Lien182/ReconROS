/* matrix_functions.h */

#ifndef __MATRIX_FUNCTIONS_H__
#define __MATRIX_FUNCTIONS_H__

void add_matrixes(int* target, int *source_a, int *source_b, int matrix_size);
void sub_matrixes(int* target, int *source_a, int *source_b, int matrix_size);
void copy_matrix(int* target, int *source, int matrix_size);

int read_matrix(int* target, char *source_file, int matrix_size);
int write_matrix(int* source, char *target_file, int matrix_size);

#endif /* __MATRIX_FUNCTIONS_H__ */
