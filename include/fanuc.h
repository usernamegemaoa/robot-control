#ifndef _FANUC_MAP_LABMEC_UAQ_
#define _FANUC_MAP_LABMEC_UAQ_

#include "matrix.h"

/* Numeric control definitions */
#define MAX_LINEAR_ACCEL	2000.0
#define MAX_ANGULAR_ACCEL	200.0
#define MAX_LINEAR_SPEED	1000.0
#define MAX_ANGULAR_SPEED	90.0

void forwardKinematics(const double *, double *);
int inverseKinematics(const double *, double *);

#endif//_FANUC_MAP_LABMEC_UAQ_

