#ifndef _MATRIX_MAP_LABMEC_UAQ_
#define _MATRIX_MAP_LABMEC_UAQ_

#include "vector.h"

typedef struct { double entry[3][3]; } matrix;

matrix dhrotation(const double, const double);
matrix rollyawpitch(const double, const double, const double);
matrix mult_matrot(const matrix, const matrix);
vector mult_matvec(const matrix, const vector);
matrix transpose(const matrix);
void round_matrix(matrix *);
vector rotation_angles(const matrix);

#endif//_MATRIX_MAP_LABMEC_UAQ_
