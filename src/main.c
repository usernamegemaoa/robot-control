#include <stdio.h>
#include <stdlib.h>
#include "../include/profile.h"

int main(int argc, char *argv[])
{
	velprofile profile;
	init_profile(&profile);
	start_profile(&profile, 1.0, 1.0);
	return EXIT_SUCCESS;
}

