#include "kalman_interface.h"

/*
 * Fill configuration with documented starting values.
 * Noise settings still require tuning for the actual vehicle.
 */
void srukf_config_default(SrukfConfig *config) {

    //chatgpt slop implementation, so the default values used are basically suggetions, will have to test and fix

    if (config == NULL) {
        return;
    }

    /*
     * Illustrative starting configuration.
     * Tune process noise and measurement uncertainties together.
     */

    /* Sigma-point distribution. */
    config->alpha = 0.5;
    config->beta  = 2.0;
    config->kappa = 0.0;

    /* Continuous process-noise spectral densities. */
    config->linear_accel_noise_density  = 0.25; /* m²/s³ */
    config->angular_accel_noise_density = 0.01; /* rad²/s³ */

    /* Maximum duration of one prediction step. */
    config->max_prediction_dt = 0.02; /* 20 milliseconds */

    /* Approximately 99% innovation gates. */
    config->gps_gate_threshold      = 9.21;  /* 2 measurements */
    config->odometry_gate_threshold = 11.34; /* 3 measurements */
}


/*
 * Check parameter ranges, finite values, positive time limits,
 * and whether the sigma-point scaling is valid.
 */
SrukfStatus srukf_config_validate(const SrukfConfig *config) {
    if (config == NULL) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /* Every configuration value must be finite. */
    if (!isfinite(config->alpha) ||
        !isfinite(config->beta) ||
        !isfinite(config->kappa) ||
        !isfinite(config->linear_accel_noise_density) ||
        !isfinite(config->angular_accel_noise_density) ||
        !isfinite(config->max_prediction_dt) ||
        !isfinite(config->gps_gate_threshold) ||
        !isfinite(config->odometry_gate_threshold)) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /* Validate parameter ranges. */
    if (config->alpha <= 0.0 ||
        config->linear_accel_noise_density < 0.0 ||
        config->angular_accel_noise_density < 0.0 ||
        config->max_prediction_dt <= 0.0) {
        return SRUKF_INVALID_ARGUMENT;
    }

    const double n = (double)SRUKF_N;
    const double n_plus_kappa = n + config->kappa;

    if (!isfinite(n_plus_kappa) || n_plus_kappa <= 0.0) {
        return SRUKF_INVALID_ARGUMENT;
    }


    //These lines calculate the sigma-point spacing and weights, then check that the calculations produce usable numbers.
    
    /*
     * c = n + lambda = alpha² * (n + kappa).
     *
     * Calculate c directly to avoid cancellation from computing
     * lambda first and then adding n back.
     */
    const double alpha_squared = config->alpha * config->alpha;
    const double c = alpha_squared * n_plus_kappa;

    if (!isfinite(alpha_squared) ||
        alpha_squared <= 0.0 ||
        !isfinite(c) ||
        c <= 0.0) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /* Check that derived sigma-point quantities are usable. */
    const double sigma_scale = sqrt(c);
    const double mean_weight_0 = 1.0 - n / c;
    const double covariance_weight_0 =
        mean_weight_0 + (1.0 - alpha_squared + config->beta);

    /* Equivalent to 1 / (2*c), without overflowing 2*c. */
    const double other_weight = 0.5 / c;

    if (!isfinite(sigma_scale) ||
        sigma_scale <= 0.0 ||
        !isfinite(mean_weight_0) ||
        !isfinite(covariance_weight_0) ||
        !isfinite(other_weight) ||
        other_weight <= 0.0) {
        return SRUKF_INVALID_ARGUMENT;
    }

    return SRUKF_OK;
}


/*
 * Initialize using a complete state and full covariance.
 * Compute its Cholesky factor and sigma-point weights.
 *
 * initial_state represents the vehicle at 'timestamp'.
 * initial_covariance must be symmetric positive definite.
 */
SrukfStatus srukf_init(
    Srukf *filter,
    const SrukfConfig *config,
    double timestamp,
    const double initial_state[SRUKF_N],
    const double initial_covariance[SRUKF_N][SRUKF_N]
) {
    if (filter == NULL ||
        config == NULL ||
        initial_state == NULL ||
        initial_covariance == NULL) {
        return SRUKF_INVALID_ARGUMENT;
    }

    SrukfStatus status = srukf_config_validate(config);

    if (status != SRUKF_OK) {
        return status;
    }

    /*
     * No previous timestamp exists during initialization.
     * Any finite timestamp is allowed.
     */
    if (!isfinite(timestamp)) {
        return SRUKF_INVALID_TIME;
    }

    for (size_t i = 0; i < SRUKF_N; ++i) {
        if (!isfinite(initial_state[i])) {
            return SRUKF_INVALID_ARGUMENT;
        }

        for (size_t j = 0; j < SRUKF_N; ++j) {
            if (!isfinite(initial_covariance[i][j])) {
                return SRUKF_INVALID_ARGUMENT;
            }
        }
    }

    /* Build a candidate without modifying the caller's filter. */
    Srukf candidate = {0};

    candidate.config = *config;
    candidate.timestamp = timestamp;

    memcpy(candidate.state,
           initial_state,
           sizeof candidate.state);

    candidate.state[SRUKF_HEADING] =
        srukf_wrap_angle(candidate.state[SRUKF_HEADING]);

    if (!isfinite(candidate.state[SRUKF_HEADING])) {
        return SRUKF_NUMERICAL_FAILURE;
    }

    status = srukf_compute_weights(
        &candidate.config,
        candidate.mean_weights,
        candidate.covariance_weights,
        &candidate.sigma_scale);

    if (status != SRUKF_OK) {
        return status;
    }

    /*
     * Pack into flat buffers for the matrix utility API.
     * Each row contains SRUKF_N elements.
     */
    double covariance[SRUKF_N * SRUKF_N];
    double factor[SRUKF_N * SRUKF_N] = {0};

    for (size_t i = 0; i < SRUKF_N; ++i) {
        for (size_t j = 0; j < SRUKF_N; ++j) {
            covariance[i * SRUKF_N + j] =
                initial_covariance[i][j];
        }
    }

    /*
     * Check symmetry and positive definiteness, then calculate:
     *
     * initial_covariance = S * S^T
     *
     * Per its declared contract, srmat_cholesky_lower() performs
     * the symmetry and positive-definiteness checks.
     */
    status = srmat_cholesky_lower(
        covariance,
        SRUKF_N,
        SRUKF_N,
        factor,
        SRUKF_N);

    if (status != SRUKF_OK) {
        return status;
    }

    /* Copy the lower triangle; the upper triangle stays zero. */
    for (size_t i = 0; i < SRUKF_N; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            candidate.sqrt_covariance[i][j] =
                factor[i * SRUKF_N + j];
        }
    }

    candidate.initialized = true;

    /* Commit only after every initialization step succeeds. */
    *filter = candidate;

    return SRUKF_OK;
}


/*
 * Clear the filter and mark it uninitialized.
 * Call srukf_init() before using it again.
 */
void srukf_reset(Srukf *filter);


/*
 * Predict forward to a timestamp without a measurement update.
 * Equal timestamps are allowed; older timestamps are rejected.
 * Split large intervals using max_prediction_dt.
 */
SrukfStatus srukf_predict_to(
    Srukf *filter,
    double timestamp,
    SrukfWorkspace *workspace
);


/*
 * Validate GPS data, predict to its timestamp, gate the residual,
 * and correct the state using measured x and y.
 */
SrukfStatus srukf_process_gps(
    Srukf *filter,
    const SrukfGpsMeasurement *measurement,
    SrukfWorkspace *workspace
);


/*
 * Validate odometry, predict to its timestamp, gate the residual,
 * and correct using speed, heading, and angular velocity.
 */
SrukfStatus srukf_process_odometry(
    Srukf *filter,
    const SrukfOdometryMeasurement *measurement,
    SrukfWorkspace *workspace
);


/*
 * Return the current estimate and reconstruct covariance S*S^T.
 * Does not advance or modify the filter.
 */
SrukfStatus srukf_get_estimate(
    const Srukf *filter,
    SrukfEstimate *estimate
);


/* Return readable text for logging a status code. */
const char *srukf_status_string(SrukfStatus status) {
    switch (status) {
        case SRUKF_OK:
            return "Success";

        case SRUKF_NOT_INITIALIZED:
            return "Filter not initialized";

        case SRUKF_INVALID_ARGUMENT:
            return "Invalid argument";

        case SRUKF_INVALID_TIME:
            return "Invalid timestamp";

        case SRUKF_MEASUREMENT_REJECTED:
            return "Measurement rejected by innovation gate";

        case SRUKF_NUMERICAL_FAILURE:
            return "Numerical calculation failed";

        default:
            return "Unknown filter status";
    }
}