#include "../include/fanuc.h"

static const double a[] = { 270.0, 900.0, 270.0, 0.0, 0.0, 0.0};
static const double b[] = {1000.0, 0.0, 0.0, 1300.0, 0.0, -270.0};
static const double alpha[] = {90.0, 0.0, 90.0, 90.0, 90.0, 0.0};
static const double maxJoint[] = {150.0, 140.0, 30.0, 180.0, 120.0, 120.0};
static const double minJoint[] = {-150.0, 25.0, -100.0, -180.0, -120.0, -180.0};
static const double stepJoints[] = {0.1, 0.5, 1.0, 5.0, 10.0};
static const double stepCartes[] = {1.0, 5.0, 10.0, 100.0, 500.0};

void forwardKinematics(const double *joints, double *coords)
{
	matrix Q11, Q22, Q33, Q44, Q55, Q66;
	vector a11, a22, a33, a44, a66;
	matrix Q21, Q31, Q41, Q51, Q;
	vector a21, a31, a41, a61;
	vector aux1, aux2, aux3, pos, att;
	Q11 = dhrotation(alpha[0], joints[0]);
	Q22 = dhrotation(alpha[1], joints[1]);
	Q33 = dhrotation(alpha[2], joints[2]);
	Q44 = dhrotation(alpha[3], joints[3]);
	Q55 = dhrotation(alpha[4], joints[4]);
	Q66 = dhrotation(alpha[5], joints[5]);
	a11 = position_vector(joints[0], a[0], b[0]);
	a22 = position_vector(joints[1], a[1], b[1]);
	a33 = position_vector(joints[2], a[2], b[2]);
	a44 = position_vector(joints[3], a[3], b[3]);
	a66 = position_vector(joints[5], a[5], b[5]);
	Q21 = mult_matrot(Q11, Q22);
	Q31 = mult_matrot(Q21, Q33);
	Q41 = mult_matrot(Q31, Q44);
	Q51 = mult_matrot(Q41, Q55);
	Q = mult_matrot(Q51, Q66);
	a21 = mult_matvec(Q11, a22);
	a31 = mult_matvec(Q21, a33);
	a41 = mult_matvec(Q31, a44);
	a61 = mult_matvec(Q51, a66);
	aux1 = sum_vectors(a11, a21);
	aux2 = sum_vectors(a31, a41);
	aux3 = sum_vectors(aux2, a61);
	pos = sum_vectors(aux1, aux3);
	round_vector(&pos);
	att = rotation_angles(Q);
	round_vector(&att);
	coords[0] = pos.x, coords[1] = pos.y, coords[2] = pos.z;
	coords[3] = att.x, coords[4] = att.y, coords[5] = att.z;
}

int inverseKinematics(const double *coords, double *joints)
{
	vector p, c, a67, aux;
	matrix Q, Q11, Q22, Q33, Q23, Q123, QT, R;
	double s1, c1, s2, c2, s3, c3;
	double num1, num2, sin35, cos35, theta35;
	double det, d11, d12, d21, d22;
	double s4, c4, s5, c5, s6, c6, sgn5;
	
	p = create_vector(coords[0], coords[1], coords[2]);
	Q = rollyawpitch(coords[3], coords[4], coords[5]);
	a67 = create_vector(0.0, 0.0, b[5]);
	aux = mult_matvec(Q, a67);
	c = sub_vectors(p, aux);
	
	joints[0] = atan2(c.y, c.x) * DEG;
	s1 = sin(joints[0] * RAD);
	c1 = cos(joints[0] * RAD);
	
	num1 = sqr(c.x) + sqr(c.y) + sqr(c.z - b[0]);
	num2 = -2.0 * a[0] * (c.x * c1 + c.y * s1);
	sin35 = (num1 + num2 - 2.5e+06) / 2.38994e+06;
	if (fabs(sin35) > 1.0)
		return -1;
	cos35 = sqrt(1 - sqr(sin35));
	theta35 = atan2(sin35, cos35);
	joints[2] = (theta35 - 0.204781) * DEG;
	s3 = sin(joints[2] * RAD);
	c3 = cos(joints[2] * RAD);
	
	det = 2 * a[1] * (a[2] * c3 + b[3] * s3) + 2.5729e+06;
	d11 = a[1] + a[2] * c3 + b[3] * s3;
	d12 = a[2] * s3 - b[3] * c3;
	d21 = -a[2] * s3 + b[3] * c3;
	d22 = a[1] + a[2] * c3 + b[3] * s3;
	c2 = (d11 * (c.x * c1 + c.y * s1 - a[0]) + d12 * (c.z - b[0])) / det;
	s2 = (d21 * (c.x * c1 + c.y * s1 - a[0]) + d22 * (c.z - b[0])) / det;
	joints[1] = atan2(s2, c2) * DEG;
	
	Q11 = dhrotation(alpha[0], joints[0]);
	Q22 = dhrotation(alpha[1], joints[1]);
	Q33 = dhrotation(alpha[2], joints[2]);
	
	Q23  = mult_matrot(Q22, Q33);
	Q123 = mult_matrot(Q11, Q23);
	QT = transpose(Q123);
	R  = mult_matrot(QT, Q);
	
	if (fabs(R.entry[2][2]) > 1.0)
		return -1;
	s5 = sqrt(1 - sqr(R.entry[2][2]));
	c5 = -R.entry[2][2];
	joints[4] = DEG * atan2(s5, c5);
	joints[4] = millround(joints[4]);
	sgn5 = sgn(joints[4]);
	
	if (fabs(R.entry[1][2]) < EPS && fabs(R.entry[0][2]) < EPS)
		joints[3] = 0.0;
	else
	{
		s4 = sgn5 * R.entry[1][2];
		c4 = sgn5 * R.entry[0][2];
		joints[3] = atan2(s4, c4) * DEG;
	}		
	if (fabs(R.entry[2][1]) < EPS && fabs(R.entry[2][0]) < EPS)
		joints[5] = 0.0;
	else
	{
		s6 = -sgn5 * R.entry[2][1];
		c6 = sgn5 * R.entry[2][0];
		joints[5] = atan2(s6, c6) * DEG;
	}
	return 0;
}

