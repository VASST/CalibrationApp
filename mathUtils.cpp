#pragma once
/*=========================================================================

Program:   touchless digitization GUI
Module:    $RCSfile: mathUtils.cpp,v $
Creator:   Elvis C. S. Chen <chene@robarts.ca>
Language:  C++
Author:    $Author: Elvis Chen $
Date:      $Date: 2016/08/22 14:50:30 $
Version:   $Revision: 0.99 $

==========================================================================

Copyright (c) Elvis C. S. Chen, elvis.chen@gmail.com

Use, modification and redistribution of the software, in source or
binary forms, are permitted provided that the following terms and
conditions are met:

1) Redistribution of the source code, in verbatim or modified
form, must retain the above copyright notice, this license,
the following disclaimer, and any notices that refer to this
license and/or the following disclaimer.

2) Redistribution in binary form must include the above copyright
notice, a copy of this license and the following disclaimer
in the documentation or with other materials provided with the
distribution.

3) Modified copies of the source code must be clearly marked as such,
and must not be misrepresented as verbatim copies of the source code.

THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE SOFTWARE "AS IS"
WITHOUT EXPRESSED OR IMPLIED WARRANTY INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE.  IN NO EVENT SHALL ANY COPYRIGHT HOLDER OR OTHER PARTY WHO MAY
MODIFY AND/OR REDISTRIBUTE THE SOFTWARE UNDER THE TERMS OF THIS LICENSE
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, LOSS OF DATA OR DATA BECOMING INACCURATE
OR LOSS OF PROFIT OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF
THE USE OR INABILITY TO USE THE SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.


=========================================================================*/

// local include
#include "mathUtils.h"
#include "lu.h"
#include "svd.h"
#include "jacobi.h"

// c++ include
#include <cmath>
#include <limits>

/*!
* Point to line registration
*
* use ICP to solve the following registration problem:
*
* O + a * D = R * X + t
*
* INPUTS: X - 3xn points
*         O - 3xn line origin
*         D - 3xn line orientation
*         tol - exit condition, try 1e-9
*
* OUTPUTS: R - 3x3 rotation
*          t - 3x1 translation
*/
void p2l(Matrix<double> &X, Matrix<double> &O, Matrix<double> &D, double tol,
	Matrix<double> &R, Matrix<double> &t, double &err)
{
	assert(X.num_rows() == O.num_rows() &&
		X.num_rows() == D.num_rows() &&
		X.num_cols() == O.num_cols() &&
		X.num_cols() == D.num_cols() &&
		X.num_rows() == 3);

	// assume column vector
	int n = X.num_cols();
	Matrix<double> e(1, n, 1.0);
	err = std::numeric_limits<double>::infinity();
	Matrix<double> E_old(3, n, 1000), E;
	Matrix<double> Y = O + D;
	Matrix<double> d(1, n), tempM;
	while (err > tol)
	{
		OPA_rigid(X, Y, R, t);
		tempM = R * X + t*e - O;
		for (int i = 0; i < n; i++)
			d[0][i] = tempM[0][i] * D[0][i] + tempM[1][i] * D[1][i] + tempM[2][i] * D[2][i];

		for (int i = 0; i < n; i++)
			for (int j = 0; j < 3; j++)
				Y[j][i] = O[j][i] + d[0][i] * D[j][i];

		E = Y - R * X - t*e;
		err = matFrobeniusNorm(E - E_old);
		E_old = E;
	}

	// compute the Eucledean distance between points and lines
	err = 0.0;
	for (int i = 0; i < E.num_cols(); i++)
	{
		err += sqrt(E[0][i] * E[0][i] + E[1][i] * E[1][i] + E[2][i] * E[2][i]);
	}
	err = err / (double)E.num_cols();

	// alternatively
	//
	// E is X after transformation
	/*
	Matrix<double> p(3,1), a(3,1), dir(3,1);
	E = R * X + t * e;
	err = 0.0;
	for ( int i = 0; i < n; i++ )
	{
	for ( int j = 0; j < 3; j++ )
	{
	p[j][0] = E[j][i];
	a[j][0] = O[j][i];
	dir[j][0] = D[j][i];
	}
	err += point2lineDistance( p, a, dir );
	}
	*/
}

/*!
* least-square solution for rigid Orthogonal Procrustes Analysis
*/
void OPA_rigid(Matrix<double> &X, Matrix<double> &Y,
	Matrix<double> &R, Matrix<double> &t)
{
	// assume column vector
	int n = X.num_cols();
	assert(X.num_rows() == Y.num_rows() &&
		X.num_cols() == Y.num_cols());
	Matrix<double> e(n, 1, 1.0), U, V;
	Matrix<double> II; II = eye(n, 1.0) - 1.0 / n * e * transpose(e);
	Vec<double> S;

	svdcmp(Y * II * transpose(X), S, U, V);

	//
	// compute rotation
	//
	// reuse II
	if (U.num_rows() == 3 && V.num_rows() == 3) // note the V transpose
	{
		II = eye(3, 1.0);
		double tempd = (double)(lu_determinant(U * transpose(V)));
		double tempi = 1.0;
		if (tempd < 0.0) tempi = -1.0;
		II[2][2] = tempi;
		R = U * II * transpose(V);
	}
	else
	{
		R = U * transpose(V);
	}

	//
	// compute translation
	//
	// reuse II again
	II = Y - R * X;
	t.newsize(X.num_rows(), 1, 0.0);
	for (int i = 0; i < X.num_cols(); i++)
	{
		for (int j = 0; j < X.num_rows(); j++)
		{
			t[j][0] += II[j][i];
		}
	}
	for (int j = 0; j < X.num_rows(); j++)
		t[j][0] = (1.0 / (double)X.num_cols()) * t[j][0];
}

/*!
* compute the determinant of a 3x3 matrix directly
*/
double detm3x3(Matrix<double> &M)
{
	assert(M.num_cols() == 3 && M.num_cols() == 3);

	return(M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
		M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
		M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]));
}
/*!
* compute the inverse of a 3x3 matrix, explicitely by hard-coding
*/
void invm3x3(Matrix<double> &M, Matrix<double> &outM)
{
	// invert a 3x3 matrix by hand
	int sizex = M.num_rows();
	int sizey = M.num_cols();
	assert(sizex == 3 && sizey == 3);
	outM.newsize(sizex, sizey);

	// may want to check if the determinant is too small
	double d = 1.0 / detm3x3(M);

	outM[0][0] = d*(M[1][1] * M[2][2] - M[1][2] * M[2][1]);
	outM[0][1] = d*(M[0][2] * M[2][1] - M[0][1] * M[2][2]);
	outM[0][2] = d*(M[0][1] * M[1][2] - M[0][2] * M[1][1]);

	outM[1][0] = d*(M[1][2] * M[2][0] - M[1][0] * M[2][2]);
	outM[1][1] = d*(M[0][0] * M[2][2] - M[0][2] * M[2][0]);
	outM[1][2] = d*(M[0][2] * M[1][0] - M[0][0] * M[1][2]);

	outM[2][0] = d*(M[1][0] * M[2][1] - M[1][1] * M[2][0]);
	outM[2][1] = d*(M[0][1] * M[2][0] - M[0][0] * M[2][1]);
	outM[2][2] = d*(M[0][0] * M[1][1] - M[0][1] * M[1][0]);
}


/*!
* compute the common line intersection amoung n lines
*
* least-square solution, works for any dimension
*/
void linesIntersection(Matrix<double> &a, // mxn line origin
	Matrix<double> &N, // mxn line direction
	Matrix<double> &p, // mx1 point
	double &err)      // FRE
{
	/*
	based on the following doc by Johannes Traa (UIUC 2013)

	Least-Squares Intersection of Lines
	http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf

	*/
	int sizex = a.num_rows();
	int sizey = a.num_cols();
	assert(sizex == N.num_rows() && sizey == N.num_cols());
	p.newsize(sizex, 1);

	Matrix<double> I; I = eye(sizex, 1.0);
	Matrix<double> R(sizex, sizex, 0.0);
	Matrix<double> q(sizex, 1, 0.0);
	Matrix<double> tempn(sizex, 1), tempa(sizex, 1), tempR, tempq;

	for (int i = 0; i < sizey; i++)
	{
		// normalize N
		double l = 1.0 / sqrt(N[0][i] * N[0][i] + N[1][i] * N[1][i] + N[2][i] * N[2][i]);
		for (int j = 0; j < sizex; j++)
		{
			tempn[j][0] = l * N[j][i];
			tempa[j][0] = a[j][i];
		}
		tempR = I - tempn * transpose(tempn);
		tempq = tempR * tempa;

		R = R + tempR;
		q = q + tempq;
	}

	// in 3D we can compute the inverse of a 3x3 matrix directly
	if (sizex == 3)
	{
		// reuse tempa
		invm3x3(R, tempa);
		p = tempa * q;
	}
	else // else use other methods to compute the (pseudo)inverse
	{
		p = lu_inverse(R) * q;
	}

	// compute FRE
	err = 0.0;
	tempa.newsize(sizex, 1);
	tempn.newsize(sizex, 1);
	for (int i = 0; i < sizey; i++)
	{
		for (int j = 0; j < sizex; j++)
		{
			tempa[j][0] = a[j][i];
			tempn[j][0] = N[j][i];
		}
		err += point2lineDistance(p, tempa, tempn);
	}
	err = err / (double)sizey;
}

/*!
* compute the distance between a point (p)
* and a line (a+t*n)
*
* assume colume vector
*/
double point2lineDistance(Matrix<double> &p, Matrix<double> &a, Matrix<double> &N)
{
	assert(p.num_rows() == a.num_rows() &&
		p.num_rows() == N.num_rows() &&
		p.num_cols() == 1 &&
		a.num_cols() == 1 &&
		N.num_cols() == 1);

	// normalize the line direction
	double l = 0.0;
	for (int i = 0; i < N.num_rows(); i++)
		l += (N[i][0] * N[i][0]);
	l = 1.0 / sqrt(l);
	Matrix<double> n = l*N;

	//d = sqrt((a-p)' * (eye(ndim)-n*n')*(a-p));
	Matrix<double> d = (transpose(a - p) * (eye(p.num_rows(), 1.0) - (n*transpose(n))) * (a - p));
	return(sqrt(d[0][0]));
}

/*!
* compute the matrix Frobenius Norm
*/
double matFrobeniusNorm(Matrix<double> &M)
{
	double sqrSum = 0.0;
	for (int i = 0; i < M.num_rows(); i++)
		for (int j = 0; j < M.num_cols(); j++)
			sqrSum += (M[i][j] * M[i][j]);

	return(sqrt(sqrSum));
}

/*!
* convert an axis-angle [aw ax ay az]' representation
* to a quaternion [qx qy qz qw]'
*/
void aa2q(Matrix<double> &aa, Matrix<double> &q)
{
	assert(aa.num_rows() == 4 && aa.num_cols() == 1);
	q.newsize(4, 1);

	// aa is obtained from converting a vtkTransform to
	// GetOrientationWXYZ 

	double angle = aa[0][0];
	double ax = aa[1][0];
	double ay = aa[2][0];
	double az = aa[3][0];

	double sa = sin(angle / 2.0);
	double ca = cos(angle / 2.0);

	q.newsize(4, 1);
	q[0][0] = sa * ax;
	q[1][0] = sa * ay;
	q[2][0] = sa * az;
	q[3][0] = ca;
}

/*!
* convert a quaternion [qx qy qz qw] to axis-angle [wxyz] representation
*/
void q2aa(Matrix<double> &qq, Matrix<double> &aa)
{
	assert(qq.num_rows() == 4 && qq.num_cols() == 1);
	aa.newsize(4, 1);

	Matrix<double> q = qq;
	qnorm(q);

	double ca = q[3][0];
	double sa = sqrt(1.0 - ca*ca);
	double angle = atan2(sa, ca) * 2.0;
	aa[0][0] = angle;
	aa[1][0] = 1.0 / sa * q[0][0];
	aa[2][0] = 1.0 / sa * q[1][0];
	aa[3][0] = 1.0 / sa * q[2][0];
}

/*!
* compute the average transformation [qx qy qz qw tx ty tz]'
*/
void q7avg(Matrix<double> &qin, Matrix<double> &qout)
{
	assert(qin.num_rows() == 7);
	qout.newsize(7, 1, 0.0);
	Matrix<double> q4in(4, qin.num_cols()), q4out;

	for (int i = 0; i < qin.num_cols(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			q4in[j][i] = qin[j][i];
		}
		for (int j = 4; j < 7; j++)
		{
			qout[j][0] += qin[j][i];
		}
	}
	//std::cerr << "in q7avg: " << q4in << std::endl;
	// average quaternion
	qavg(q4in, q4out);
	//std::cerr << "in q7avg: " << q4out << std::endl;
	for (int i = 0; i < 4; i++) qout[i][0] = q4out[i][0];

	// average translation
	qout[4][0] = 1.0 / (double)qin.num_cols() * qout[4][0];
	qout[5][0] = 1.0 / (double)qin.num_cols() * qout[5][0];
	qout[6][0] = 1.0 / (double)qin.num_cols() * qout[6][0];
}

/*!
* compute an "average" quaternion
*/
void qavg(Matrix<double> &qin, Matrix<double> &qout)
{
	assert(qin.num_rows() == 4);
	int n = qin.num_cols();
	qout.newsize(4, 1);

	Matrix<double> Q(4, 4, 0.0), qn(4, 1), tempQ;

	for (int i = 0; i < n; i++)
	{
		// check the sign, sinze -q=q
		double s = 0.0;
		for (int j = 0; j < 4; j++)
		{
			qn[j][0] = qin[j][i];
			s += qin[j][i] * qin[j][0];
		}

		if (s < 0.0)
			for (int j = 0; j < 4; j++) qn[j][0] = -qn[j][0];

		qnorm(qn);
		tempQ = qn * transpose(qn);

		for (int j = 0; j < 4; j++)
			for (int k = 0; k < 4; k++)
				Q[j][k] = Q[j][k] + tempQ[j][k];
	}
	Matrix<double> eigv, eigd;
	int nrot;
	jacobi(Q, eigv, eigd, nrot);
	//std::cerr << "in q4avg: " << Q << eigv << eigd << std::endl;

	// find the largest eigenvector corresponding to the largest eigen value
	int ind = 0;
	double indv = eigv[0][0]; // 1st
	for (int i = 1; i < 4; i++)
		if (indv < eigv[i][0])
		{
			indv = eigv[i][0];
			ind = i;
		}

	// qout is the eigen vector corresponding to the largest eigen value
	for (int i = 0; i < 4; i++) qout[i][0] = eigd[i][ind];

	// check the sign
	double s = qn[0][0] * qout[0][0] + qn[1][0] * qout[1][0] + qn[2][0] * qout[2][0] + qn[3][0] * qout[3][0];
	if (s < 0.0)
	{
		for (int i = 0; i < 4; i++) qout[i][0] = -qout[i][0];
	}
}
/*!
* normalize a quaternion
*/
void qnorm(Matrix<double> &q)
{
	assert(q.num_cols() == 1 && q.num_rows() == 4);

	double l = 1.0 / sqrt(q[0][0] * q[0][0] +
		q[1][0] * q[1][0] +
		q[2][0] * q[2][0] +
		q[3][0] * q[3][0]);
	q[0][0] = l* q[0][0];
	q[1][0] = l* q[1][0];
	q[2][0] = l* q[2][0];
	q[3][0] = l* q[3][0];
}

// append columns to the end of a matrix
void matAppend(Matrix<double> &M, Matrix<double> &v)
{
	assert(M.num_rows() == v.num_rows());
	int n = M.num_cols();

	// copy the matrix
	Matrix<double> tempM(M.num_rows(), n + v.num_cols());
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < M.num_rows(); j++)
		{
			tempM[j][i] = M[j][i];
		}
	}

	// append columns
	for (int i = 0; i < v.num_cols(); i++)
		for (int j = 0; j < M.num_rows(); j++)
		{
			tempM[j][n + i] = v[j][i];
		}
	M = tempM;
}

double dot(Matrix<double> &a, Matrix<double> &b)
{
	if (a.num_cols() != 1 || b.num_cols() != 1 ||
		a.num_rows() != b.num_rows())
	{
		std::cerr << "size mismatch in dot" << std::endl;
		exit(1);
	}
	double v = 0;
	for (int i = 0; i < a.num_rows(); i++)
		v += a[i][0] * b[i][0];
	return(v);
}

void cross(Matrix<double> &a, Matrix<double> &b, Matrix<double> &c)
{
	c.newsize(3, 1);
	// assumes a, b, c are all 3x1
	c[0][0] = a[1][0] * b[2][0] - b[1][0] * a[2][0];
	c[1][0] = a[2][0] * b[0][0] - b[2][0] * a[0][0];
	c[2][0] = a[0][0] * b[1][0] - b[0][0] * a[1][0];
}


// pseudoinverse using svd, using matlab's implementation
Matrix<double> pinv(Matrix<double> &a)
{
	Matrix<double> U, V;
	Vec<double> S;
	svdcmp(a, S, U, V);
	double tol = 1e-16;

	// std::cerr << S << U << V;
	Vec<int> badS(S.size());
	int summ = 0;
	for (int i = 0; i < S.size(); i++)
	{
		if (S[i] > tol) badS[i] = 0;
		else {
			summ += 1;
			badS[i] = 1;
		}
	}
	Matrix<double> u(U.num_rows(), U.num_cols() - summ);
	Matrix<double> v(V.num_rows(), V.num_cols() - summ);
	Matrix<double> s(S.size() - summ, 1);
	summ = 0;
	for (int i = 0; i < U.num_cols(); i++)
	{
		if (!badS[i])
		{
			for (int j = 0; j < U.num_rows(); j++)
			{
				u[j][summ] = U[j][i];
				v[j][summ] = V[j][i];
				s[summ][0] = 1.0 / S[i];
			}
			summ++;
		}
	}

	for (int i = 0; i < u.num_cols(); i++)
	{
		for (int j = 0; j < u.num_rows(); j++)
		{
			v[j][i] = v[j][i] * s[i][0];
		}
	}

	Matrix<double> out;
	out = v * transpose(u);

	return(out);
}

Matrix<double> amq(Matrix<double> &q)
{
	Matrix<double> A(4, 4, 0.0);
	Matrix<double> tempA(3, 3), temp(4, 1);
	for (int i = 0; i < 4; i++) temp[i][0] = q[i][0];
	tempA = mq(temp);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) A[i][j] = tempA[i][j];

	A[0][3] = q[4][0];
	A[1][3] = q[5][0];
	A[2][3] = q[6][0];
	A[3][3] = 1.0;

	return(A);
}
// rotation matrix from quaternion
// A = MQ(q) returns the 3x3 rotation matrix defined by the quaternion q
// A is applied to the column vectors. q is input as 4x1 matrix
Matrix<double> mq(Matrix<double> &q)
{
	Matrix<double> A(3, 3);
	double q1 = q[0][0];
	double q2 = q[1][0];
	double q3 = q[2][0];
	double q4 = q[3][0];

	// normalize q;
	double l = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
	q1 /= l;
	q2 /= l;
	q3 /= l;
	q4 /= l;

	A[0][0] = q1*q1 + q2*q2 - q3*q3 - q4*q4;
	A[0][1] = 2.0*(q2*q3 - q1*q4);
	A[0][2] = 2.0*(q2*q4 + q1*q3);

	A[1][0] = 2.0*(q2*q3 + q1*q4);
	A[1][1] = q1*q1 + q3*q3 - q2*q2 - q4*q4;
	A[1][2] = 2.0*(q3*q4 - q1*q2);

	A[2][0] = 2.0*(q2*q4 - q1*q3);
	A[2][1] = 2.0*(q3*q4 + q1*q2);
	A[2][2] = q1*q1 + q4*q4 - q2*q2 - q3*q3;

	return(A);
}
