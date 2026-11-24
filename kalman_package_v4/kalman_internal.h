#ifndef SRUKF_IH
#define SRUKF_IH

#include "kalman_interface.h"

/*
 * Calculate the nine mean/covariance weights and sigma-point scale
 * from alpha, beta, kappa, and state dimension.
 */
SrukfStatus srukf_compute_weights(
    const SrukfConfig *config,
    double mean_weights[SRUKF_SIGMA_COUNT],
    double covariance_weights[SRUKF_SIGMA_COUNT],
    double *sigma_scale
);


/*
 * Generate sigma points from state and lower-triangular S.
 * Each row of sigma_points is one four-element state.
 */
void srukf_generate_sigma_points(
    const double state[SRUKF_N],
    const double S[SRUKF_N][SRUKF_N],
    double sigma_scale,
    double sigma_points[SRUKF_SIGMA_COUNT][SRUKF_N]
);


/*
 * Advance one state through the motion model.
 * Use midpoint heading and the supplied encoder velocity.
 * Velocity is an input, never a stored state.
 */
void srukf_motion_model(
    const double state[SRUKF_N],
    double dt,
    double velocity,
    double predicted_state[SRUKF_N]
);


/*
 * Build a discrete process-noise factor Lq such that Q = Lq*Lq^T.
 * Include encoder variance and configured acceleration noise densities.
 * Evaluate input-noise sensitivity at the mean midpoint heading.
 *
 * Lq may be rank deficient; it need not be a Cholesky factor.
 */
SrukfStatus srukf_process_noise_factor(
    const SrukfConfig *config,
    const double state[SRUKF_N],
    double dt,
    double velocity_variance,
    double Lq[SRUKF_N][SRUKF_N]
);


/*
 * Perform one prediction interval:
 * sigma points -> motion model -> mean -> square-root covariance.
 * Commit state and uncertainty only if all calculations succeed.
 */
SrukfStatus srukf_predict_step(
    Srukf *filter,
    double dt,
    double velocity,
    double velocity_variance,
    SrukfWorkspace *workspace
);

/*
 * Return measurement dimension:
 * VN = 4; unsupported sensor = 0.
 */
size_t srukf_measurement_dimension(SrukfSensor sensor);


/*
 * Extract a predicted measurement from one state:
 * VN -> [x, y, heading, omega] (identity observation).
 */
void srukf_measurement_model(
    SrukfSensor sensor,
    const double state[SRUKF_N],
    double predicted_measurement[SRUKF_MAX_MEAS]
);


/*
 * Calculate state-measurement cross-covariance.
 * Wrap heading differences in both spaces where applicable.
 *
 * Only the first measurement_dimension columns are used.
 */
void srukf_cross_covariance(
    SrukfSensor sensor,
    const double state_points[SRUKF_SIGMA_COUNT][SRUKF_N],
    const double measurement_points
        [SRUKF_SIGMA_COUNT][SRUKF_MAX_MEAS],
    const double state_mean[SRUKF_N],
    const double measurement_mean[SRUKF_MAX_MEAS],
    const double covariance_weights[SRUKF_SIGMA_COUNT],
    double cross_covariance[SRUKF_N][SRUKF_MAX_MEAS]
);


/*
 * Solve K*(Sz*Sz^T) = Pxz using triangular solves.
 * Never form the inverse of measurement covariance.
 */
SrukfStatus srukf_compute_gain(
    size_t measurement_dimension,
    const double Pxz[SRUKF_N][SRUKF_MAX_MEAS],
    const double Sz[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS],
    double K[SRUKF_N][SRUKF_MAX_MEAS]
);


/*
 * Calculate normalized innovation squared:
 * NIS = residual^T * (Sz*Sz^T)^(-1) * residual.
 * Implement by solving Sz*u = residual, then computing u^T*u.
 */
SrukfStatus srukf_innovation_score(
    size_t measurement_dimension,
    const double residual[SRUKF_MAX_MEAS],
    const double Sz[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS],
    double *nis
);


/*
 * Apply state += K*residual and wrap heading.
 * Update S by downdating with each column of U = K*Sz.
 * Return failure if the downdate is not numerically valid.
 */
SrukfStatus srukf_apply_correction(
    Srukf *filter,
    size_t measurement_dimension,
    const double residual[SRUKF_MAX_MEAS],
    const double K[SRUKF_N][SRUKF_MAX_MEAS],
    const double Sz[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS],
    SrukfWorkspace *workspace
);


/*
 * Shared measurement update at the filter's CURRENT timestamp.
 * Does not predict forward.
 *
 * Regenerate sigma points from the current predicted state/S,
 * transform them to measurement space, calculate uncertainty,
 * gate the innovation, and apply the correction.
 *
 * R is the full 4x4 VN covariance.
 */
SrukfStatus srukf_measurement_update(
    Srukf *filter,
    SrukfSensor sensor,
    const double measurement[SRUKF_MAX_MEAS],
    const double R[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS],
    double gate_threshold,
    SrukfWorkspace *workspace
);

/* Normalize an angle to [-pi, pi). */
double srukf_wrap_angle(double angle);


/*
 * Compute a weighted mean of sigma points.
 * angle_index = -1 means no angular component.
 *
 * For heading, use a consistent local angular mean that handles
 * wraparound and signed UKF weights.
 * Report failure if the angular distribution is too ambiguous.
 */
SrukfStatus srukf_weighted_mean(
    const double *points,
    size_t point_count,
    size_t dimension,
    size_t point_stride,
    const double *weights,
    int angle_index,
    double *mean
);


/*
 * Calculate a - b, wrapping the angular component if present.
 * angle_index = -1 means ordinary subtraction throughout.
 */
void srukf_residual(
    const double *a,
    const double *b,
    size_t dimension,
    int angle_index,
    double *residual
);


/*
 * Construct square-root covariance from sigma-point deviations
 * and an additive noise factor.
 *
 * Use QR for positive-weight terms and noise columns.
 * Apply signed Cholesky updates/downdates where required.
 * Do not take sqrt() of a negative covariance weight.
 */
SrukfStatus srukf_build_sqrt_covariance(
    const double *points,
    size_t point_count,
    size_t dimension,
    size_t point_stride,
    const double *mean,
    const double *covariance_weights,
    int angle_index,
    const double *noise_factor,
    size_t noise_columns,
    size_t noise_stride,
    double *S_out,
    size_t S_stride,
    SrukfWorkspace *workspace
);

#endif
