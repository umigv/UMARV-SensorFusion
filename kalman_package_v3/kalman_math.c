#include "kalman_workspace.h"
#include <math.h>
#include <float.h>
#include "kalman_math.h"

/* Return false if any matrix entry is NaN or infinity. */
bool srmat_is_finite(
    const double *A,
    size_t rows,
    size_t columns,
    size_t stride
) {
    if (A == NULL || rows == 0 || columns == 0 || stride < columns) {
        return false;
    }
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < columns; ++j) {
            if (!isfinite(A[i * stride + j])) return false;
        }
    }
    return true;
}


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
) {
    if (A == NULL ||
        L == NULL ||
        n == 0 ||
        A_stride < n ||
        L_stride < n) {
        return SRUKF_INVALID_ARGUMENT;
    }

    if (!srmat_is_finite(A, n, n, A_stride)) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /*
     * Allow small relative differences caused by rounding.
     * Normalize before subtracting to avoid overflow.
     */
    const double symmetry_tolerance = 64.0 * DBL_EPSILON;

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < i; ++j) {
            const double a = A[i * A_stride + j];
            const double b = A[j * A_stride + i];
            const double scale = fmax(fabs(a), fabs(b));

            if (scale > 0.0 &&
                fabs(a / scale - b / scale) > symmetry_tolerance) {
                return SRUKF_INVALID_ARGUMENT;
            }
        }
    }

    /* Clear the active output matrix, including the upper triangle. */
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            L[i * L_stride + j] = 0.0;
        }
    }

    /* Calculate L one row at a time. */
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            double value = A[i * A_stride + j];

            for (size_t k = 0; k < j; ++k) {
                value -= L[i * L_stride + k] *
                         L[j * L_stride + k];
            }

            if (!isfinite(value)) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            if (i == j) {
                /*
                 * Diagonal:
                 * L[i][i] = sqrt(A[i][i] - sum(L[i][k]^2)).
                 *
                 * A zero or negative pivot means the matrix
                 * is not positive definite at working precision.
                 */
                if (value <= 0.0) {
                    return SRUKF_NUMERICAL_FAILURE;
                }

                L[i * L_stride + i] = sqrt(value);
            } else {
                /*
                 * Off-diagonal:
                 * L[i][j] =
                 *   (A[i][j] - sum(L[i][k]*L[j][k])) / L[j][j].
                 *
                 * L[j][j] is positive from an earlier iteration.
                 */
                const double entry =
                    value / L[j * L_stride + j];

                if (!isfinite(entry)) {
                    return SRUKF_NUMERICAL_FAILURE;
                }

                L[i * L_stride + j] = entry;
            }
        }
    }

    return SRUKF_OK;
}


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
) {
    if (A == NULL ||
        L == NULL ||
        workspace == NULL ||
        rows == 0 ||
        rows > SRUKF_N ||
        columns < rows ||
        columns > SRUKF_QR_MAX_COLUMNS ||
        A_stride < columns ||
        L_stride < rows) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /*
     * Working matrix B = A^T.
     *
     * B has 'columns' rows and 'rows' columns.
     * QR is performed in place on this workspace buffer.
     */
    double (*B)[SRUKF_N] = workspace->qr_transpose;

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < columns; ++j) {
            const double value = A[i * A_stride + j];

            if (!isfinite(value)) {
                return SRUKF_INVALID_ARGUMENT;
            }

            B[j][i] = value;
        }
    }

    /* Reference column norms for detecting numerical rank deficiency. */
    double original_norm[SRUKF_N] = {0};
    for (size_t j = 0; j < rows; ++j) {
        for (size_t i = 0; i < columns; ++i) {
            original_norm[j] = hypot(original_norm[j], B[i][j]);
        }
        if (!isfinite(original_norm[j]) || original_norm[j] == 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    /*
     * Reduce B to upper-triangular form with Householder
     * reflections. We do not need to construct Q explicitly.
     */
    for (size_t k = 0; k < rows; ++k) {
        /*
         * Norm of the active portion of column k.
         * hypot() avoids unnecessarily squaring large values.
         */
        double norm = 0.0;

        for (size_t i = k; i < columns; ++i) {
            norm = hypot(norm, B[i][k]);
        }

        if (!isfinite(norm) || norm <= 0.0 ||
            norm / original_norm[k] <= 64.0 * DBL_EPSILON) {
            return SRUKF_NUMERICAL_FAILURE;
        }

        /*
         * Choose the reflected diagonal with the opposite sign
         * from the first entry to avoid cancellation.
         */
        const double sign = (B[k][k] >= 0.0) ? 1.0 : -1.0;
        const double diagonal = -sign * norm;

        /*
         * Form a normalized Householder vector u:
         *
         * u initially = x / ||x|| + sign(x[0]) * e0
         * then normalize u.
         *
         * H = I - 2*u*u^T
         */
        double u[SRUKF_QR_MAX_COLUMNS] = {0};

        for (size_t i = k; i < columns; ++i) {
            u[i] = B[i][k] / norm;
        }

        u[k] += sign;

        double u_norm = 0.0;

        for (size_t i = k; i < columns; ++i) {
            u_norm = hypot(u_norm, u[i]);
        }

        if (!isfinite(u_norm) || u_norm <= 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }

        for (size_t i = k; i < columns; ++i) {
            u[i] /= u_norm;
        }

        /*
         * Apply H to each remaining column:
         *
         * column = column - 2*u*(u^T*column)
         */
        for (size_t j = k + 1; j < rows; ++j) {
            double dot = 0.0;

            for (size_t i = k; i < columns; ++i) {
                dot += u[i] * B[i][j];
            }

            if (!isfinite(dot)) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            for (size_t i = k; i < columns; ++i) {
                const double updated =
                    B[i][j] - (2.0 * u[i]) * dot;

                if (!isfinite(updated)) {
                    return SRUKF_NUMERICAL_FAILURE;
                }

                B[i][j] = updated;
            }
        }

        /*
         * The transformed active column is known exactly
         * in mathematical form: [diagonal, 0, ..., 0].
         */
        B[k][k] = diagonal;

        for (size_t i = k + 1; i < columns; ++i) {
            B[i][k] = 0.0;
        }
    }

    /*
     * The leading rows-by-rows block of B now contains R.
     * Verify it before changing the caller's output.
     */
    for (size_t i = 0; i < rows; ++i) {
        if (!isfinite(B[i][i]) || B[i][i] == 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }

        for (size_t j = i; j < rows; ++j) {
            if (!isfinite(B[i][j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }
    }

    /*
     * Make R's diagonal positive by flipping entire rows of R.
     * Row sign changes preserve R^T*R.
     *
     * Transpose that sign-adjusted R into L.
     */
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < rows; ++j) {
            if (j <= i) {
                const double row_sign =
                    (B[j][j] < 0.0) ? -1.0 : 1.0;

                L[i * L_stride + j] = row_sign * B[j][i];
            } else {
                L[i * L_stride + j] = 0.0;
            }
        }
    }

    return SRUKF_OK;
}


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
) {
    if (L == NULL ||
        u == NULL ||
        n == 0 ||
        n > SRUKF_N ||
        L_stride < n ||
        (sign != 1 && sign != -1)) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /*
     * Work on local copies so failure leaves L and u unchanged.
     * This implementation supports the project's maximum
     * matrix dimension, SRUKF_N.
     */
    double factor[SRUKF_N][SRUKF_N] = {{0}};
    double vector[SRUKF_N] = {0};

    for (size_t i = 0; i < n; ++i) {
        if (!isfinite(u[i])) {
            return SRUKF_INVALID_ARGUMENT;
        }

        vector[i] = u[i];

        for (size_t j = 0; j <= i; ++j) {
            const double value = L[i * L_stride + j];

            if (!isfinite(value)) {
                return SRUKF_INVALID_ARGUMENT;
            }

            factor[i][j] = value;
        }

        if (factor[i][i] <= 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    for (size_t k = 0; k < n; ++k) {
        const double diagonal = factor[k][k];
        const double element = vector[k];

        double r;

        if (sign == 1) {
            /*
             * r = sqrt(diagonal^2 + element^2)
             *
             * hypot() avoids explicitly squaring the inputs.
             */
            r = hypot(diagonal, element);
        } else {
            /*
             * r = sqrt(diagonal^2 - element^2)
             *
             * The updated diagonal must remain positive.
             */
            if (fabs(element) >= diagonal) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            const double ratio = fabs(element) / diagonal;

            /*
             * Factored form avoids squaring large inputs.
             * It also avoids subtracting two nearly equal squares.
             */
            r = diagonal *
                (sqrt(1.0 - ratio) * sqrt(1.0 + ratio));
        }

        if (!isfinite(r) || r <= 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }

        const double c = r / diagonal;
        const double s = element / diagonal;

        if (!isfinite(c) || c <= 0.0 || !isfinite(s)) {
            return SRUKF_NUMERICAL_FAILURE;
        }

        factor[k][k] = r;

        /*
         * Update the entries below this diagonal, then update
         * the remaining vector using the NEW column entries.
         */
        for (size_t i = k + 1; i < n; ++i) {
            const double new_entry =
                fma((double)sign * s, vector[i], factor[i][k]) / c;

            if (!isfinite(new_entry)) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            const double scaled_vector = c * vector[i];

            if (!isfinite(scaled_vector)) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            const double new_vector =
                fma(-s, new_entry, scaled_vector);

            if (!isfinite(new_vector)) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            factor[i][k] = new_entry;
            vector[i] = new_vector;
        }

        /* This vector component has now been eliminated. */
        vector[k] = 0.0;
    }

    /* Commit the completed factor and clear the scratch vector. */
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            L[i * L_stride + j] =
                (j <= i) ? factor[i][j] : 0.0;
        }

        u[i] = vector[i];
    }

    return SRUKF_OK;
}


/* Solve L*x = b by forward substitution. */
SrukfStatus srmat_solve_lower(
    const double *L,
    size_t n,
    size_t L_stride,
    const double *b,
    double *x
) {
    if (L == NULL ||
        b == NULL ||
        x == NULL ||
        n == 0 ||
        n > SRUKF_N ||
        L_stride < n) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /* Validate the active lower triangle and right-hand side. */
    for (size_t i = 0; i < n; ++i) {
        if (!isfinite(b[i])) {
            return SRUKF_INVALID_ARGUMENT;
        }

        for (size_t j = 0; j <= i; ++j) {
            if (!isfinite(L[i * L_stride + j])) {
                return SRUKF_INVALID_ARGUMENT;
            }
        }

        if (L[i * L_stride + i] == 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    /* Keep the caller's output unchanged until success. */
    double solution[SRUKF_N] = {0};

    for (size_t i = 0; i < n; ++i) {
        double value = b[i];

        for (size_t j = 0; j < i; ++j) {
            value = fma(
                -L[i * L_stride + j],
                solution[j],
                value);

            if (!isfinite(value)) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }

        solution[i] = value / L[i * L_stride + i];

        if (!isfinite(solution[i])) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    for (size_t i = 0; i < n; ++i) {
        x[i] = solution[i];
    }

    return SRUKF_OK;
}


/* Solve L^T*x = b by backward substitution. */
SrukfStatus srmat_solve_lower_transpose(
    const double *L,
    size_t n,
    size_t L_stride,
    const double *b,
    double *x
) {
    if (L == NULL ||
        b == NULL ||
        x == NULL ||
        n == 0 ||
        n > SRUKF_N ||
        L_stride < n) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /* Validate the active lower triangle and right-hand side. */
    for (size_t i = 0; i < n; ++i) {
        if (!isfinite(b[i])) {
            return SRUKF_INVALID_ARGUMENT;
        }

        for (size_t j = 0; j <= i; ++j) {
            if (!isfinite(L[i * L_stride + j])) {
                return SRUKF_INVALID_ARGUMENT;
            }
        }

        if (L[i * L_stride + i] == 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    /* Preserve the caller's output until all calculations succeed. */
    double solution[SRUKF_N] = {0};

    /*
     * Iterate backward without underflowing the unsigned index.
     * L^T[i][j] is L[j][i].
     */
    for (size_t remaining = n; remaining > 0; --remaining) {
        const size_t i = remaining - 1;
        double value = b[i];

        for (size_t j = i + 1; j < n; ++j) {
            value = fma(
                -L[j * L_stride + i],
                solution[j],
                value);

            if (!isfinite(value)) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }

        solution[i] = value / L[i * L_stride + i];

        if (!isfinite(solution[i])) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    for (size_t i = 0; i < n; ++i) {
        x[i] = solution[i];
    }

    return SRUKF_OK;
}


/* Reconstruct P = L*L^T, principally for output/debugging. */
void srmat_covariance_from_factor(
    const double *L,
    size_t n,
    size_t L_stride,
    double *P,
    size_t P_stride
) {
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            double value = 0.0;

            /*
             * P[i][j] = sum(L[i][k] * L[j][k]).
             *
             * Because L is lower triangular and j <= i,
             * only k = 0 through j can contribute.
             */
            for (size_t k = 0; k <= j; ++k) {
                value = fma(
                    L[i * L_stride + k],
                    L[j * L_stride + k],
                    value);
            }

            P[i * P_stride + j] = value;
            P[j * P_stride + i] = value;
        }
    }
}
