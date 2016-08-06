#ifndef _PROFILE_MAP_LABMEC_UAQ_
#define _PROFILE_MAP_LABMEC_UAQ_

#include "mathdef.h"

#define TS		0.001

#define STOPPED	0
#define ACCEL	1
#define SLEWVEL	2
#define DECEL	3

typedef struct
{
	int state;
	int count1, count2;
	int riseTime, slewTime;
	double curpos, prepos;
	double curvel, prevel;
	double accel;
	double maxSpeed, maxAccel;
} velprofile;

void init_profile(velprofile *);
void abort_profile(velprofile *);
void execute_profile(velprofile *);
void start_profile(velprofile *, double, double);
void stop_profile(velprofile *);

#endif//_PROFILE_MAP_LABMEC_UAQ_
