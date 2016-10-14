#pragma once
/*=========================================================================

Program:   touchless digitization GUI
Module:    $RCSfile: lu.h,v $
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

//
// Elvis Chen
// chene@cs.queensu.ca
//
// Department of Computing and Information Science
// Queen's University, Kingston, Ontario, Canada
//
// Feb. 17, 2000
//

//
// Filename:  lu.h
//

// Initial implementation of LU factorization.
//
// Algorithms are taken from Numerical Recipes.
//

#ifndef __LU_H__
#define __LU_H__


#include <cmath>
#include "matrix.h"

// Given a matrix A[1..n][1..n], this function replaces it by the LU
// decomposition of a rowwise permutation of itself.  "A" is input, 
// and is also output at the end of execution.  indx[1..n] is an
// output vector that records the row permutation effected by the
// partial pivoting (needed to increase numerical stability).  This
// function returns +-1 depending on whether the number of row 
// interchanges was even or odd, respectively.
//
// this function returns -1 is factorization is unsuccessful.
//

#define TINY 1.0e-20


template<class MAT, class VEC>
int ludcmp(MAT &a, VEC &indx)
{
	int output = 1; // no row interchanges yet.
	Subscript i, j, k, imax = 0;

	Subscript X = a.num_rows();
	Subscript Y = a.num_cols();

	if ((X == 0) || (Y == 0)) return -1;

	// the codes in Numerical Recipes only works for square matrix  
	if (X != Y) return -1;

	if (indx.dim() != X) indx.newsize(X); // need to make sure the piviting
										  // matrix is the same size

	typename MAT::element_type big, temp, sum, dum;

	Vec<double> vv(X);
	// vv stores the implicit scaling of each row

	for (i = 1; i <= X; i++) {
		// loop over rows to get the implicit scaling information

		big = 0.0;
		for (j = 1; j <= X; j++)
			if ((temp = fabs(a(i, j))) > big) big = temp;

		if (big == 0.0) return -1; // singular matrix

		vv(i) = 1.0 / big; // save the scaling
	}


	// This is the loop over columns of Crout's method
	for (j = 1; j <= X; j++) {
		for (i = 1; i < j; i++) {
			sum = a(i, j);

			for (k = 1; k < i; k++) sum -= a(i, k) * a(k, j);

			a(i, j) = sum;
		}

		big = 0.0; // Initialize for the search for largest pivot element
		for (i = j; i <= X; i++) {
			sum = a(i, j);

			for (k = 1; k < j; k++)
				sum -= a(i, k) * a(k, j);

			a(i, j) = sum;

			if ((dum = vv(i) * fabs(sum)) >= big) {
				// is the figure of merit for the pivot better than the best so far?
				big = dum;
				imax = i;
			}
		}

		if (j != imax) {
			// Do we need to interchange rows?

			for (k = 1; k <= X; k++) { // Yes, we do
				dum = a(imax, k);
				a(imax, k) = a(j, k);
				a(j, k) = dum;
			}

			output = -(output); // change the parity of output

			vv(imax) = vv(j); // interchange the scale factor.
		}


		indx(j) = imax;

		if (a(j, j) == 0.0) a(j, j) = TINY;

		// if the pivot element is zero the matrix is singular
		// (at least to the prevision of the algorithm).  For
		// some applications on singular matrices, it is desirable
		// to substitute TINY for zero

		if (j != X) {
			// Now, finally, divide by the pivot element
			dum = (1.0) / (a(j, j));
			for (i = (j + 1); i <= X; i++) a(i, j) *= dum;
		}
	} // go back for the next column in the reduction

	return output;
}


template<class MAT, class VEC, class VECS>
int lubksb(const MAT &a, const VECS &indx, VEC &b)
{
	//
	// NOTE:  TNT and Numerical Recipes are actually the same in there...
	//  
	Subscript i, ii = 0, ip, j;
	Subscript n = b.dim();

	typename MAT::element_type sum = 0.0;


	for (i = 1; i <= n; i++) {
		// When ii is set to positive value, it will become the
		// index of the first nonvanishing element of b.
		//  We now do the forward substitutio.  The only new
		// wrinkle is to unscramble the permutation as we go.

		ip = indx(i);
		sum = b(ip);
		b(ip) = b(i);

		if (ii)
			for (j = ii; j <= (i - 1); j++) sum -= a(i, j)*b(j);

		else if (sum) ii = i;    // a nonzero element was encountered,
								 // so from now on we will have to do
								 // the sums in the loop above

		b(i) = sum;
	}
	// cout << endl;

	for (i = n; i >= 1; i--) {

		// When ii is set to positive value, it will become the
		// index of the first nonvanishing element of b.
		//  We now do the forward substitutio.  The only new
		// wrinkle is to unscramble the permutation as we go.

		sum = b(i);

		for (j = i + 1; j <= n; j++) sum -= a(i, j)*b(j);

		b(i) = sum / a(i, i); // store a component of solution vector X
	}

	// all done!
	return 0;

}


template< class T >
Matrix<T> lu_inverse(const Matrix<T> &A)
{
	Subscript X = A.num_rows();
	Subscript Y = A.num_cols();
	Subscript i, j;

	assert(X == Y);

	Matrix<T> out(X, Y);
	Matrix<T> tmp(A);  // we don't want to destroy A

	Vec<Subscript> indx(X);
	Vec<T> col(X);

	ludcmp(tmp, indx);

	for (j = 1; j <= X; j++) {
		for (i = 1; i <= X; i++) col(i) = 0.0;
		col(j) = 1.0;

		lubksb(tmp, indx, col);

		for (i = 1; i <= X; i++) out(i, j) = col(i);
	}

	return (out);
}

template< class T >
double lu_determinant(const Matrix<T> &A)
{
	Subscript X = A.num_rows();
	Subscript Y = A.num_cols();
	Subscript j;

	assert(X == Y);

	Matrix<T> tmp(A); // we don't want to destroy A
	Vec<Subscript> indx(X);

	double d = (double)ludcmp(tmp, indx);

	for (j = 1; j <= X; j++) d *= (double)tmp(j, j);

	return d;
}

#endif // of __LU_H__
