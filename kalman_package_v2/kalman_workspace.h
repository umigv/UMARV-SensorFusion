#ifndef SRUKF_WORKSPACE_H
#define SRUKF_WORKSPACE_H

#include "kalman_interface.h"

#define SRUKF_QR_MAX_COLUMNS (SRUKF_SIGMA_COUNT + SRUKF_N)

/* Per-filter scratch storage; do not share between concurrent calls. */
struct SrukfWorkspace {
    double sigma_points[SRUKF_SIGMA_COUNT][SRUKF_N];
    double predicted_points[SRUKF_SIGMA_COUNT][SRUKF_N];
    double process_noise_factor[SRUKF_N][SRUKF_N];
    double qr_transpose[SRUKF_QR_MAX_COLUMNS][SRUKF_N];
};

#endif
