#include <math.h>
#include "../include/matrix.h"

matrix dhrotation(const double alpha, const double theta)
{
	matrix res;
	double sAlpha, cAlpha;
	double sTheta, cTheta;
	sAlpha = sin(alpha * RAD);
	cAlpha = cos(alpha * RAD);
	sTheta = sin(theta * RAD);
	cTheta = cos(theta * RAD);
	res.entry[0][0] = cTheta;
	res.entry[0][1] = -cAlpha * sTheta;
	res.entry[0][2] = sAlpha * sTheta;
	res.entry[1][0] = sTheta;
	res.entry[1][1] = cAlpha * cTheta;
	res.entry[1][2] = -sAlpha * cTheta;
	res.entry[2][0] = 0.0;
	res.entry[2][1] = sAlpha;
	res.entry[2][2] = cAlpha;
	return res;
}

matrix rollyawpitch(const double phi, const double theta, const double psi)
{
	matrix res;
	double sPhi, cPhi;
	double sPsi, cPsi;
	double sTheta, cTheta;
	sPhi = sin(phi * RAD);
	cPhi = cos(phi * RAD);
	sPsi = sin(psi * RAD);
	cPsi = cos(psi * RAD);
	sTheta = sin(theta * RAD);
	cTheta = cos(theta * RAD);
	res.entry[0][0] = cTheta * cPhi;
	res.entry[0][1] = cPhi * sTheta * sPsi - cPsi * sPhi;
	res.entry[0][2] = sPhi * sPsi + cPhi * cPsi * sTheta;
	res.entry[1][0] = cTheta * sPhi;
	res.entry[1][1] = cPhi * cPsi + sPhi * sTheta * sPsi;
	res.entry[1][2] = sPhi * sTheta * cPsi - cPhi * sPsi;
	res.entry[2][0] = -sTheta;
	res.entry[2][1] = cTheta * sPsi;
	res.entry[2][2] = cTheta * cPsi;
	return res;
}

matrix mult_matrot(const matrix matA, const matrix matB)
{
	int i, j, k;
	matrix res;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			res.entry[i][j] = 0.0;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			for (k = 0; k < 3; k++)
				res.entry[i][j] += matA.entry[i][k] * matB.entry[k][j];
	return res;
}

vector mult_matvec(const matrix mat, const vector vec)
{
	vector res;
	res.x = mat.entry[0][0]*vec.x + mat.entry[0][1]*vec.y + mat.entry[0][2]*vec.z;
	res.y = mat.entry[1][0]*vec.x + mat.entry[1][1]*vec.y + mat.entry[1][2]*vec.z;
	res.z = mat.entry[2][0]*vec.x + mat.entry[2][1]*vec.y + mat.entry[2][2]*vec.z;
	return res;
}

matrix transpose(const matrix mat)
{
	int i, j;
	matrix res;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			res.entry[i][j] = mat.entry[j][i];
	return res;
}

void round_matrix(matrix *mat)
{
	mat->entry[0][0] = ((int)(1e6 * mat->entry[0][0])) / 1e6;
	mat->entry[0][1] = ((int)(1e6 * mat->entry[0][1])) / 1e6;
	mat->entry[0][2] = ((int)(1e6 * mat->entry[0][2])) / 1e6;
	mat->entry[1][0] = ((int)(1e6 * mat->entry[1][0])) / 1e6;
	mat->entry[1][1] = ((int)(1e6 * mat->entry[1][1])) / 1e6;
	mat->entry[1][2] = ((int)(1e6 * mat->entry[1][2])) / 1e6;
	mat->entry[2][0] = ((int)(1e6 * mat->entry[2][0])) / 1e6;
	mat->entry[2][1] = ((int)(1e6 * mat->entry[2][1])) / 1e6;
	mat->entry[2][2] = ((int)(1e6 * mat->entry[2][2])) / 1e6;
}

vector rotation_angles(const matrix mat)
{
	vector res;
	double sinA, cosA;
	sinA = -mat.entry[2][0];
	cosA = sqrt(sqr(mat.entry[2][1]) + sqr(mat.entry[2][2]));
	res.y = atan2(sinA, cosA) * DEG;
	if (fabs(mat.entry[0][0]) < 1e-4 && fabs(mat.entry[1][0]) < 1e-4)
	{
		res.x = 0;
		res.z = atan2(-mat.entry[0][1], mat.entry[1][1]) * DEG;
	}
	else
	{
		res.x = atan2(mat.entry[1][0], mat.entry[0][0]) * DEG;
		res.z = atan2(mat.entry[2][1], mat.entry[2][2]) * DEG;
	}
	return res;
}
