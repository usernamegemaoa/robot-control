#include "../include/profile.h"
#include <stdio.h>

void init_profile(velprofile *profile)
{
	profile->state = STOPPED;
}

void abort_profile(velprofile *profile)
{
	profile->state = STOPPED;
}

void execute_profile(velprofile *profile)
{
	switch(profile->state)
	{
	case ACCEL:
		profile->curvel = profile->prevel + profile->accel;
		profile->curpos = profile->prepos + profile->curvel;
		profile->count1++;
		profile->state = profile->count1 >= profile->riseTime? SLEWVEL : ACCEL;
		break;
	case SLEWVEL:
		profile->curpos = profile->prepos + profile->curvel;
		profile->count2++;
		profile->state = profile->count2 < profile->slewTime? SLEWVEL : DECEL;
		break;
	case DECEL:
		profile->curvel = profile->prevel - profile->accel;
		profile->curpos = profile->prepos + profile->curvel;
		profile->count1 = profile->count1 - 1;
		profile->state  = profile->count1 > 0? DECEL : STOPPED;
		break;
	case STOPPED:
		break;
	}
}

void start_profile(velprofile *profile, double despos, double desvel)
{
	int period;
	double ftime, rtime, stime, ntime, accel;
	if (profile->state != STOPPED)
		return;
	ftime  = fabs(3.0 * despos / (2.0 * desvel));
	period = (int)(ftime / (3.0 * TS));
	ntime  = 3.0 * TS * period;
	accel  = 3.0 * fabs(desvel) / ntime;
	if (period == 0.0)
	{
		perror("Motion can not be executed...\r\n");
		profile->curpos += despos;
		return;	
	}
	profile->riseTime = profile->slewTime = period;
	profile->accel = sgn(despos) * accel * TS * TS;
	profile->count1 = 0;
	profile->count2 = 0;
	profile->state = ACCEL;
}

void stop_profile(velprofile *profile)
{
	if (profile->state != STOPPED)
		profile->state = DECEL;
}

