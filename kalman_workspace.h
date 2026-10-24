#ifndef SRUKF_WORKSPACE_H
#define SRUKF_WORKSPACE_H

#include "kalman_interface.h"

/*
 * QR input columns:
 * up to 11 sigma-point deviations + 5 noise-factor columns.
 */
#define SRUKF_QR_MAX_COLUMNS (SRUKF_SIGMA_COUNT + SRUKF_N)

struct SrukfWorkspace {
    /* Sigma points before and after motion prediction. */
    double sigma_points[SRUKF_SIGMA_COUNT][SRUKF_N];
    double predicted_points[SRUKF_SIGMA_COUNT][SRUKF_N];

    /* Sigma points mapped into GPS or odometry measurement space. */
    double measurement_points[SRUKF_SIGMA_COUNT][SRUKF_MAX_MEAS];

    /* Predicted state and measurement means. */
    double predicted_mean[SRUKF_N];
    double measurement_mean[SRUKF_MAX_MEAS];

    /* Noise and uncertainty factors. */
    double process_noise_factor[SRUKF_N][SRUKF_N];
    double measurement_noise_factor[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS];
    double predicted_S[SRUKF_N][SRUKF_N];
    double measurement_S[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS];

    /* Measurement correction intermediates. */
    double cross_covariance[SRUKF_N][SRUKF_MAX_MEAS];
    double kalman_gain[SRUKF_N][SRUKF_MAX_MEAS];
    double innovation[SRUKF_MAX_MEAS];

    /* Columns of K * measurement_S, used for downdates. */
    double downdate_columns[SRUKF_N][SRUKF_MAX_MEAS];

    /* Candidate results, committed only after successful calculation. */
    double candidate_state[SRUKF_N];
    double candidate_S[SRUKF_N][SRUKF_N];

    /* QR input and its transposed working copy. */
    double qr_input[SRUKF_N][SRUKF_QR_MAX_COLUMNS];
    double qr_transpose[SRUKF_QR_MAX_COLUMNS][SRUKF_N];

    /* Small scratch vectors for matrix operations. */
    double vector_a[SRUKF_N];
    double vector_b[SRUKF_N];
};

#endif