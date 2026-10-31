#include "kalman_internal.h"
#include "float.h"


#ifndef SRUKF_MH
#define SRUKF_MH

/* Return false if any matrix entry is NaN or infinity. */
bool srmat_is_finite(
    const double *A,
    size_t rows,
    size_t columns,
    size_t stride
);


/*
 * Check symmetry and compute lower Cholesky factor:
 * A = L*L^T.
 * Fail if A is not sufficiently positive definite.
 */
SrukfStatus srmat_cholesky_lower(
    const double *A,
    size_t n,
    size_t A_stride,
    double *L,
    size_t L_stride
);


/*
 * Given A, compute a lower factor L such that L*L^T = A*A^T
 * using QR factorization of A^T.
 * Normalize diagonal signs consistently.
 */
SrukfStatus srmat_qr_lower_factor(
    const double *A,
    size_t rows,
    size_t columns,
    size_t A_stride,
    double *L,
    size_t L_stride,
    SrukfWorkspace *workspace
);


/*
 * Modify L so that:
 * L_new*L_new^T = L_old*L_old^T + sign*u*u^T.
 *
 * sign = +1: update.
 * sign = -1: downdate.
 * The vector u is scratch and may be overwritten.
 */
SrukfStatus srmat_cholesky_rank1(
    double *L,
    size_t n,
    size_t L_stride,
    double *u,
    int sign
);


/* Solve L*x = b by forward substitution. */
SrukfStatus srmat_solve_lower(
    const double *L,
    size_t n,
    size_t L_stride,
    const double *b,
    double *x
);


/* Solve L^T*x = b by backward substitution. */
SrukfStatus srmat_solve_lower_transpose(
    const double *L,
    size_t n,
    size_t L_stride,
    const double *b,
    double *x
);


/* Reconstruct P = L*L^T, principally for output/debugging. */
void srmat_covariance_from_factor(
    const double *L,
    size_t n,
    size_t L_stride,
    double *P,
    size_t P_stride
);

#endif