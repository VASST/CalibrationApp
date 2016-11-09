#pragma once
/*=========================================================================

Program:   touchless digitization GUI
Module:    $RCSfile: mathUtils.h,v $
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

#ifndef __MATHUTILS_H__
#define __MATHUTILS_H__

// C++ includes
#include <assert.h> 
#include <vector>

// local includes
#include "matrix.h"
//#include "svd.h"
#include "lu.h"

namespace echen
{
	/*!
	* compute the matrix Frobenius Norm
	*/
	double matFrobeniusNorm(Matrix<double> &M);

	/*!
	* point to line registration
	*/
	void p2l(Matrix<double> &X, Matrix<double> &O, Matrix<double> &D, double tol,
		Matrix<double> &R, Matrix<double> &t, double &err);

	/*!
	* compute the distance between a point (p)
	* and a line (a+t*n)
	*
	* assume colume vector
	*/
	double point2lineDistance(Matrix<double> &p, Matrix<double> &a, Matrix<double> &n);

	/*!
	* compute the determinant of a 3x3 matrix directly
	*/
	double detm3x3(Matrix<double> &M);

	/*!
	* compute the inverse of a 3x3 matrix, explicitely by hard-coding
	*/
	void invm3x3(Matrix<double> &M, Matrix<double> &outM);

	/*!
	* compute the common line intersection amoung n lines
	*
	* least-square solution, works for any dimension
	*/
	void linesIntersection(Matrix<double> &a, // 3xn line origin
		Matrix<double> &N, // 3xn line direction
		Matrix<double> &p, // mx1 point 
		double &err);     // FRE

						  /*!
						  * least-square solution for rigid Orthogonal Procrustes Analysis
						  */
	void OPA_rigid(Matrix<double> &X, Matrix<double> &Y,
		Matrix<double> &R, Matrix<double> &t);

	// matrix utilities

	// append columns to the end of a matrix
	void matAppend(Matrix<double> &M, Matrix<double> &v);

	// quaternion related

	/*!
	* convert an axis-angle [aw ax ay az]' representation
	* to a quaternion [qx qy qz qw]'
	*/
	void aa2q(Matrix<double> &aa, Matrix<double> &q);

	/*!
	* convert a quaternion to axis-angle representation
	*/
	void q2aa(Matrix<double> &q, Matrix<double> &aa);

	/*!
	* normalize a quaternion
	*/
	void qnorm(Matrix<double> &q);

	/*!
	* compute an "average" quaternion
	*/
	void qavg(Matrix<double> &qin, Matrix<double> &qout);

	/*!
	* compute the average transformation [qx qy qz qw tx ty tz]'
	*/
	void q7avg(Matrix<double> &qin, Matrix<double> &qout);


	// cross product
	void cross(Matrix<double> &a, Matrix<double> &b, Matrix<double> &c);

	// dot product, assuming a and b are column vectors
	double dot(Matrix<double> &a, Matrix<double> &b);

	// rotation matrix from quaternion
	// A = MQ(q) returns the 3x3 rotation matrix defined by the quaternion q
	// A is applied to the column vectors
	Matrix<double> mq(Matrix<double> &q);
	Matrix<double> amq(Matrix<double> &q);

	// pseudoinverse using svd
	Matrix<double> pinv(Matrix<double> & a);

	double linetre(Matrix<double> &P,
		Matrix<double> &N,
		std::vector< Matrix<double> > &S,
		Matrix<double> &tar);

	void stiffnesses(Matrix<double> &K, // 6x6 input
		Matrix<double> &tar, // 3x1 input
		Matrix<double> &sigma,
		Matrix<double> &mu,
		Matrix<double> &mueq);

	void kline(Matrix<double> &P,                 // 3xn input
		Matrix<double> &N,                  // 3xn input
		std::vector < Matrix<double> > &S,  // 3x3xn input
		Matrix<double> &K,                  // 6x6 output
		Matrix<double> &A,                  // 3x3 output
		Matrix<double> &B,                  // 3x3 output
		Matrix<double> &C                   // 3x3 output
	);

	void anisofstiff3(Matrix<double> &P,                 // 3xn input
		std::vector < Matrix<double> > &S,  // 3x3xn input
		Matrix<double> &K,                  // 6x6 output
		Matrix<double> &A,                  // 3x3 output
		Matrix<double> &B,                  // 3x3 output
		Matrix<double> &D                   // 3x3 output
	);
}

#endif // of MATHUTILS_H__