#ifndef _VECTOR_MAP_LABMEC_UAQ_
#define _VECTOR_MAP_LABMEC_UAQ_

#include "mathdef.h"

typedef struct { double x, y, z; } vector;

vector create_vector(const double, const double, const double);
vector sum_vectors(const vector, const vector);
vector sub_vectors(const vector, const vector);
vector position_vector(const double, const double, const double);
void round_vector(vector *);

#endif//_VECTOR_MAP_LABMEC_UAQ_
