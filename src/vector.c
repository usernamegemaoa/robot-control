#include "../include/vector.h"

vector create_vector(const double x, const double y, const double z)
{
	vector res;
	res.x = x;
	res.y = y;
	res.z = z;
	return res;
}

vector sum_vectors(const vector vecA, const vector vecB)
{
	vector res;
	res.x = vecA.x + vecB.x;
	res.y = vecA.y + vecB.y;
	res.z = vecA.z + vecB.z;
	return res;
}

vector sub_vectors(const vector opeA, const vector opeB)
{
	vector res;
	res.x = opeA.x - opeB.x;
	res.y = opeA.y - opeB.y;
	res.z = opeA.z - opeB.z;
	return res;
}

vector position_vector(const double theta, const double a, const double b)
{
	vector res;
	res.x = a * cos(theta * RAD);
	res.y = a * sin(theta * RAD);
	res.z = b;
	return res;
}

void round_vector(vector *vec)
{
	vec->x = ((int)(vec->x * 1000.0)) / 1000.0;
	vec->y = ((int)(vec->y * 1000.0)) / 1000.0;
	vec->z = ((int)(vec->z * 1000.0)) / 1000.0;
}

