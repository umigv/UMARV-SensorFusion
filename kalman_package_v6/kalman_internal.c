#include "kalman_math.h"
#include "kalman_workspace.h"
#include <math.h>
#include <string.h>
#include "kalman_internal.h"

/*
 * Calculate the eleven mean/covariance weights and sigma-point scale
 * from alpha, beta, kappa, and state dimension.
 */
SrukfStatus srukf_compute_weights(
    const SrukfConfig *config,
    double mean_weights[SRUKF_SIGMA_COUNT],
    double covariance_weights[SRUKF_SIGMA_COUNT],
    double *sigma_scale
) {
    if (config == NULL ||
        mean_weights == NULL ||
        covariance_weights == NULL ||
        sigma_scale == NULL) {
        return SRUKF_INVALID_ARGUMENT;
    }

    SrukfStatus status = srukf_config_validate(config);

    if (status != SRUKF_OK) {
        return status;
    }

    const double n = (double)SRUKF_N;
    const double alpha_squared = config->alpha * config->alpha;

    /* c = n + lambda, calculated directly to avoid cancellation. */
    const double c = alpha_squared * (n + config->kappa);

    const double central_mean_weight = 1.0 - n / c;
    const double central_covariance_weight =
        central_mean_weight + (1.0 - alpha_squared + config->beta);

    const double other_weight = 0.5 / c;
    const double scale = sqrt(c);

    /*
     * Validation already checked these derived quantities.
     * No outputs are modified until validation succeeds.
     */
    mean_weights[0] = central_mean_weight;
    covariance_weights[0] = central_covariance_weight;

    for (size_t i = 1; i < SRUKF_SIGMA_COUNT; ++i) {
        mean_weights[i] = other_weight;
        covariance_weights[i] = other_weight;
    }

    *sigma_scale = scale;

    return SRUKF_OK;
}


/*
 * Generate sigma points from state and lower-triangular S.
 * Each row of sigma_points is one five-element state.
 */
void srukf_generate_sigma_points(
    const double state[SRUKF_N],
    const double S[SRUKF_N][SRUKF_N],
    double sigma_scale,
    double sigma_points[SRUKF_SIGMA_COUNT][SRUKF_N]
) {
    /* Sigma point 0 is the current estimated state. */
    for (size_t j = 0; j < SRUKF_N; ++j) {
        sigma_points[0][j] = state[j];
    }

    /*
     * For each column i of S:
     *
     * point[i + 1]           = state + sigma_scale * S[:, i]
     * point[i + 1 + SRUKF_N] = state - sigma_scale * S[:, i]
     */
    for (size_t i = 0; i < SRUKF_N; ++i) {
        const size_t positive = i + 1;
        const size_t negative = i + 1 + SRUKF_N;

        for (size_t j = 0; j < SRUKF_N; ++j) {
            const double offset = sigma_scale * S[j][i];

            sigma_points[positive][j] = state[j] + offset;
            sigma_points[negative][j] = state[j] - offset;
        }
    }

    /* Keep every heading in [-pi, pi). */
    for (size_t i = 0; i < SRUKF_SIGMA_COUNT; ++i) {
        sigma_points[i][SRUKF_HEADING] =
            srukf_wrap_angle(sigma_points[i][SRUKF_HEADING]);
    }
}


/* The same midpoint model uses stored velocity before either correction. */
void srukf_motion_model(
    const double state[SRUKF_N],
    double dt,
    double predicted_state[SRUKF_N]
) {
    const double x = state[SRUKF_X], y = state[SRUKF_Y];
    const double heading = state[SRUKF_HEADING];
    const double omega = state[SRUKF_OMEGA];
    const double velocity = state[SRUKF_V];
    const double turn = omega * dt;
    const double midpoint = heading + 0.5 * turn;
    predicted_state[SRUKF_X] = x + velocity * dt * cos(midpoint);
    predicted_state[SRUKF_Y] = y + velocity * dt * sin(midpoint);
    predicted_state[SRUKF_HEADING] = srukf_wrap_angle(heading + turn);
    predicted_state[SRUKF_OMEGA] = omega;
    predicted_state[SRUKF_V] = velocity;
}

/* Integrated continuous acceleration noise at mean midpoint heading.
 * Linear block: Var(ds)=q*dt^3/3, Cov(ds,dv)=q*dt^2/2, Var(dv)=q*dt.
 * Angular block has the same form for heading/rate. Encoder measurement R
 * is used only in the correction, never added again as process noise.
 */
SrukfStatus srukf_process_noise_factor(
    const SrukfConfig *config,
    const double state[SRUKF_N],
    double dt,
    double Lq[SRUKF_N][SRUKF_N]
) {
    if (config == NULL || state == NULL || Lq == NULL)
        return SRUKF_INVALID_ARGUMENT;
    if (!isfinite(dt) || dt < 0.0) return SRUKF_INVALID_TIME;
    SrukfStatus status = srukf_config_validate(config);
    if (status != SRUKF_OK) return status;
    for (size_t i = 0; i < SRUKF_N; ++i)
        if (!isfinite(state[i])) return SRUKF_INVALID_ARGUMENT;
    double factor[SRUKF_N][SRUKF_N] = {{0}};
    if (dt != 0.0) {
        const double midpoint = state[SRUKF_HEADING] +
                                0.5 * state[SRUKF_OMEGA] * dt;
        const double c = cos(midpoint), s = sin(midpoint);
        const double speed_std = sqrt(dt) * sqrt(config->linear_accel_noise_density);
        const double rate_std = sqrt(dt) * sqrt(config->angular_accel_noise_density);
        const double shared = 0.5 * dt * speed_std;
        const double extra = dt / sqrt(12.0) * speed_std;
        factor[SRUKF_X][0] = c * shared;
        factor[SRUKF_Y][0] = s * shared;
        factor[SRUKF_V][0] = speed_std;
        factor[SRUKF_X][1] = c * extra;
        factor[SRUKF_Y][1] = s * extra;
        factor[SRUKF_HEADING][2] = 0.5 * dt * rate_std;
        factor[SRUKF_OMEGA][2] = rate_std;
        factor[SRUKF_HEADING][3] = dt / sqrt(12.0) * rate_std;
    }
    for (size_t i = 0; i < SRUKF_N; ++i)
        for (size_t j = 0; j < SRUKF_N; ++j)
            if (!isfinite(factor[i][j])) return SRUKF_NUMERICAL_FAILURE;
    memcpy(Lq, factor, sizeof factor);
    return SRUKF_OK;
}

/*
 * Perform one prediction interval:
 * sigma points -> motion model -> mean -> square-root covariance.
 * Commit state and uncertainty only if all calculations succeed.
 */
SrukfStatus srukf_predict_step(
    Srukf *filter,
    double dt,
    SrukfWorkspace *workspace
) {
    if (filter == NULL || workspace == NULL) {
        return SRUKF_INVALID_ARGUMENT;
    }

    if (!filter->initialized) {
        return SRUKF_NOT_INITIALIZED;
    }

    if (!isfinite(dt) || dt < 0.0) {
        return SRUKF_INVALID_TIME;
    }

    SrukfStatus valid_config = srukf_config_validate(&filter->config);
    if (valid_config != SRUKF_OK) return valid_config;

    if (dt == 0.0) {
        return SRUKF_OK;
    }

    /*
     * 1. Generate sigma points around the current estimate.
     *
     * Each row represents one possible five-element state.
     */
    srukf_generate_sigma_points(
        filter->state,
        filter->sqrt_covariance,
        filter->sigma_scale,
        workspace->sigma_points);

    /*
     * 2. Propagate every sigma point through the motion model.
     */
    for (size_t i = 0; i < SRUKF_SIGMA_COUNT; ++i) {
        for (size_t j = 0; j < SRUKF_N; ++j) {
            if (!isfinite(workspace->sigma_points[i][j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }

        srukf_motion_model(
            workspace->sigma_points[i],
            dt,
            workspace->predicted_points[i]);

        for (size_t j = 0; j < SRUKF_N; ++j) {
            if (!isfinite(workspace->predicted_points[i][j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }
    }

    /*
     * Pack the points into a genuinely flat array for helpers
     * that take row-major double* buffers.
     */
    double points[SRUKF_SIGMA_COUNT * SRUKF_N];

    for (size_t i = 0; i < SRUKF_SIGMA_COUNT; ++i) {
        for (size_t j = 0; j < SRUKF_N; ++j) {
            points[i * SRUKF_N + j] =
                workspace->predicted_points[i][j];
        }
    }

    /*
     * 3. Calculate the predicted mean.
     *
     * The heading component requires angle-aware averaging.
     */
    double predicted_mean[SRUKF_N];

    SrukfStatus status = srukf_weighted_mean(
        points,
        SRUKF_SIGMA_COUNT,
        SRUKF_N,
        SRUKF_N,
        filter->mean_weights,
        SRUKF_HEADING,
        predicted_mean);

    if (status != SRUKF_OK) {
        return status;
    }

    predicted_mean[SRUKF_HEADING] =
        srukf_wrap_angle(predicted_mean[SRUKF_HEADING]);

    for (size_t i = 0; i < SRUKF_N; ++i) {
        if (!isfinite(predicted_mean[i])) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    /*
     * 4. Construct the process-noise factor for this interval.
     *
     * This design evaluates the additive noise model using
     * the state at the beginning of the prediction step.
     *
     * Q = Lq * Lq^T
     */
    status = srukf_process_noise_factor(
        &filter->config,
        filter->state,
        dt,
        workspace->process_noise_factor);

    if (status != SRUKF_OK) {
        return status;
    }

    double noise_factor[SRUKF_N * SRUKF_N];

    for (size_t i = 0; i < SRUKF_N; ++i) {
        for (size_t j = 0; j < SRUKF_N; ++j) {
            const double value =
                workspace->process_noise_factor[i][j];

            if (!isfinite(value)) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            noise_factor[i * SRUKF_N + j] = value;
        }
    }

    /*
     * 5. Calculate the predicted square-root covariance.
     *
     * The helper combines weighted sigma-point deviations
     * and process noise using QR and signed Cholesky updates.
     */
    double predicted_S[SRUKF_N * SRUKF_N] = {0};

    status = srukf_build_sqrt_covariance(
        points,
        SRUKF_SIGMA_COUNT,
        SRUKF_N,
        SRUKF_N,
        predicted_mean,
        filter->covariance_weights,
        SRUKF_HEADING,
        noise_factor,
        SRUKF_N,
        SRUKF_N,
        predicted_S,
        SRUKF_N,
        workspace);

    if (status != SRUKF_OK) {
        return status;
    }

    /* Verify the lower-triangular factor before committing it. */
    for (size_t i = 0; i < SRUKF_N; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            if (!isfinite(predicted_S[i * SRUKF_N + j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }

        if (predicted_S[i * SRUKF_N + i] <= 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    /*
     * 6. Commit only after all calculations succeed.
     * Preserve configuration, weights, and timestamp.
     */
    memcpy(filter->state, predicted_mean, sizeof predicted_mean);

    for (size_t i = 0; i < SRUKF_N; ++i) {
        for (size_t j = 0; j < SRUKF_N; ++j) {
            filter->sqrt_covariance[i][j] =
                (j <= i) ? predicted_S[i * SRUKF_N + j] : 0.0;
        }
    }

    return SRUKF_OK;
}

/* Sparse sensor observations; no synthetic measurements for missing fields. */
size_t srukf_measurement_dimension(SrukfSensor sensor) {
    if (sensor == SRUKF_SENSOR_VN) return 4;
    if (sensor == SRUKF_SENSOR_ENCODER) return 1;
    return 0;
}

void srukf_measurement_model(
    SrukfSensor sensor,
    const double state[SRUKF_N],
    double predicted_measurement[SRUKF_MAX_MEAS]
) {
    for (size_t i = 0; i < SRUKF_MAX_MEAS; ++i) predicted_measurement[i] = 0.0;
    if (sensor == SRUKF_SENSOR_VN) {
        for (size_t i = 0; i < 4; ++i) predicted_measurement[i] = state[i];
        predicted_measurement[2] = srukf_wrap_angle(state[SRUKF_HEADING]);
    } else if (sensor == SRUKF_SENSOR_ENCODER) {
        predicted_measurement[0] = state[SRUKF_V];
    } else {
        for (size_t i = 0; i < SRUKF_MAX_MEAS; ++i) predicted_measurement[i] = NAN;
    }
}

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
) {
    const size_t m = srukf_measurement_dimension(sensor);

    const int measurement_angle_index =
        (sensor == SRUKF_SENSOR_VN) ? SRUKF_HEADING : -1;

    /* Clear the entire output, including all measurement columns. */
    for (size_t row = 0; row < SRUKF_N; ++row) {
        for (size_t col = 0; col < SRUKF_MAX_MEAS; ++col) {
            cross_covariance[row][col] = 0.0;
        }
    }

    /* The measurement-update caller validates the sensor first. */
    if (m == 0 || m > SRUKF_MAX_MEAS) {
        return;
    }

    /* Compensation for roundoff during weighted accumulation. */
    double compensation[SRUKF_N][SRUKF_MAX_MEAS] = {{0}};

    for (size_t i = 0; i < SRUKF_SIGMA_COUNT; ++i) {
        const double weight = covariance_weights[i];

        if (weight == 0.0) {
            continue;
        }

        double state_difference[SRUKF_N];
        double measurement_difference[SRUKF_MAX_MEAS] = {0};

        srukf_residual(
            state_points[i],
            state_mean,
            SRUKF_N,
            SRUKF_HEADING,
            state_difference);

        srukf_residual(
            measurement_points[i],
            measurement_mean,
            m,
            measurement_angle_index,
            measurement_difference);

        /*
         * Add the weighted outer product:
         * weight * state_difference * measurement_difference^T.
         */
        for (size_t row = 0; row < SRUKF_N; ++row) {
            for (size_t col = 0; col < m; ++col) {
                const double term =
                    weight *
                    state_difference[row] *
                    measurement_difference[col];

                /* Kahan compensated summation. */
                const double adjusted =
                    term - compensation[row][col];

                const double previous = cross_covariance[row][col];
                const double next = previous + adjusted;

                compensation[row][col] =
                    (next - previous) - adjusted;

                cross_covariance[row][col] = next;
            }
        }
    }
}


/*
 * Solve K*(Sz*Sz^T) = Pxz using triangular solves.
 * Never form the inverse of measurement covariance.
 */
SrukfStatus srukf_compute_gain(
    size_t measurement_dimension,
    const double Pxz[SRUKF_N][SRUKF_MAX_MEAS],
    const double Sz[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS],
    double K[SRUKF_N][SRUKF_MAX_MEAS]
) {
    if (Pxz == NULL ||
        Sz == NULL ||
        K == NULL ||
        measurement_dimension == 0 ||
        measurement_dimension > SRUKF_MAX_MEAS) {
        return SRUKF_INVALID_ARGUMENT;
    }

    const size_t m = measurement_dimension;

    /* Pack the active lower triangle for the matrix helpers. */
    double factor[SRUKF_MAX_MEAS * SRUKF_MAX_MEAS] = {0};

    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            if (!isfinite(Sz[i][j])) {
                return SRUKF_INVALID_ARGUMENT;
            }

            factor[i * SRUKF_MAX_MEAS + j] = Sz[i][j];
        }

        if (Sz[i][i] <= 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    for (size_t i = 0; i < SRUKF_N; ++i) {
        for (size_t j = 0; j < m; ++j) {
            if (!isfinite(Pxz[i][j])) {
                return SRUKF_INVALID_ARGUMENT;
            }
        }
    }

    /* Keep K unchanged unless all rows are calculated successfully. */
    double gain[SRUKF_N][SRUKF_MAX_MEAS] = {{0}};

    for (size_t row = 0; row < SRUKF_N; ++row) {
        double intermediate[SRUKF_MAX_MEAS] = {0};

        /*
         * For this row, solve:
         *
         * Sz * intermediate = Pxz[row]^T
         * Sz^T * gain[row]^T = intermediate
         *
         * Together:
         * (Sz * Sz^T) * gain[row]^T = Pxz[row]^T
         */
        SrukfStatus status = srmat_solve_lower(
            factor,
            m,
            SRUKF_MAX_MEAS,
            Pxz[row],
            intermediate);

        if (status != SRUKF_OK) {
            return status;
        }

        status = srmat_solve_lower_transpose(
            factor,
            m,
            SRUKF_MAX_MEAS,
            intermediate,
            gain[row]);

        if (status != SRUKF_OK) {
            return status;
        }

        for (size_t col = 0; col < m; ++col) {
            if (!isfinite(gain[row][col])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }
    }

    /* Clear the entire gain buffer before filling the active columns. */
    memcpy(K, gain, sizeof gain);

    return SRUKF_OK;
}


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
) {
    if (residual == NULL ||
        Sz == NULL ||
        nis == NULL ||
        measurement_dimension == 0 ||
        measurement_dimension > SRUKF_MAX_MEAS) {
        return SRUKF_INVALID_ARGUMENT;
    }

    const size_t m = measurement_dimension;

    /* Pack the active lower triangle for the matrix helper. */
    double factor[SRUKF_MAX_MEAS * SRUKF_MAX_MEAS] = {0};

    for (size_t i = 0; i < m; ++i) {
        if (!isfinite(residual[i])) {
            return SRUKF_INVALID_ARGUMENT;
        }

        for (size_t j = 0; j <= i; ++j) {
            if (!isfinite(Sz[i][j])) {
                return SRUKF_INVALID_ARGUMENT;
            }

            factor[i * SRUKF_MAX_MEAS + j] = Sz[i][j];
        }

        if (Sz[i][i] <= 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    /* Solve Sz * normalized_residual = residual. */
    double normalized_residual[SRUKF_MAX_MEAS] = {0};

    SrukfStatus status = srmat_solve_lower(
        factor,
        m,
        SRUKF_MAX_MEAS,
        residual,
        normalized_residual);

    if (status != SRUKF_OK) {
        return status;
    }

    /* Accumulate the norm without directly squaring each entry. */
    double norm = 0.0;

    for (size_t i = 0; i < m; ++i) {
        if (!isfinite(normalized_residual[i])) {
            return SRUKF_NUMERICAL_FAILURE;
        }

        norm = hypot(norm, normalized_residual[i]);
    }

    const double score = norm * norm;

    if (!isfinite(score)) {
        return SRUKF_NUMERICAL_FAILURE;
    }

    /* Only modify the output after successful calculation. */
    *nis = score;

    return SRUKF_OK;
}


/* Square-root Joseph correction, valid for both optimal and masked gains.
 * P+ = (I-KH) P- (I-KH)^T + K R K^T.
 * QR factors A=[(I-KH)S, K*Lr] without forming P or an inverse.
 * A simple P-K*Pzz*K^T downdate would be incorrect for a masked gain.
 */
SrukfStatus srukf_apply_correction(
    Srukf *filter,
    SrukfSensor sensor,
    const double residual[SRUKF_MAX_MEAS],
    const double K[SRUKF_N][SRUKF_MAX_MEAS],
    const double Lr[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS],
    SrukfWorkspace *workspace
) {
    const size_t m = srukf_measurement_dimension(sensor);
    if (filter == NULL || residual == NULL || K == NULL || Lr == NULL ||
        workspace == NULL || m == 0) return SRUKF_INVALID_ARGUMENT;
    if (!filter->initialized) return SRUKF_NOT_INITIALIZED;
    const size_t columns = SRUKF_N + m;
    double A[SRUKF_N * (SRUKF_N + SRUKF_MAX_MEAS)] = {0};
    double factor[SRUKF_N * SRUKF_N];
    double corrected[SRUKF_N];
    for (size_t j = 0; j < m; ++j) {
        if (!isfinite(residual[j]) || !isfinite(Lr[j][j]) || Lr[j][j] <= 0)
            return SRUKF_INVALID_ARGUMENT;
        for (size_t k = 0; k <= j; ++k)
            if (!isfinite(Lr[j][k])) return SRUKF_INVALID_ARGUMENT;
    }
    for (size_t i = 0; i < SRUKF_N; ++i) {
        if (!isfinite(filter->state[i]) ||
            !isfinite(filter->sqrt_covariance[i][i]) || filter->sqrt_covariance[i][i] <= 0)
            return SRUKF_NUMERICAL_FAILURE;
        corrected[i] = filter->state[i];
        for (size_t j = 0; j < m; ++j) {
            if (!isfinite(K[i][j])) return SRUKF_INVALID_ARGUMENT;
            corrected[i] = fma(K[i][j], residual[j], corrected[i]);
        }
        if (!isfinite(corrected[i])) return SRUKF_NUMERICAL_FAILURE;
        for (size_t col = 0; col < SRUKF_N; ++col) {
            double value = col <= i ? filter->sqrt_covariance[i][col] : 0.0;
            for (size_t j = 0; j < m; ++j) {
                const size_t observed = sensor == SRUKF_SENSOR_VN ? j : SRUKF_V;
                const double hs = col <= observed ? filter->sqrt_covariance[observed][col] : 0.0;
                value -= K[i][j] * hs;
            }
            A[i * columns + col] = value;
        }
        for (size_t col = 0; col < m; ++col) {
            double value = 0.0;
            for (size_t j = col; j < m; ++j) value += K[i][j] * Lr[j][col];
            A[i * columns + SRUKF_N + col] = value;
        }
    }
    corrected[SRUKF_HEADING] = srukf_wrap_angle(corrected[SRUKF_HEADING]);
    if (!isfinite(corrected[SRUKF_HEADING])) return SRUKF_NUMERICAL_FAILURE;
    for (size_t i = 0; i < SRUKF_N * columns; ++i)
        if (!isfinite(A[i])) return SRUKF_NUMERICAL_FAILURE;
    SrukfStatus status = srmat_qr_lower_factor(
        A, SRUKF_N, columns, columns, factor, SRUKF_N, workspace);
    if (status != SRUKF_OK) return status;
    memcpy(filter->state, corrected, sizeof corrected);
    for (size_t i = 0; i < SRUKF_N; ++i)
        for (size_t j = 0; j < SRUKF_N; ++j)
            filter->sqrt_covariance[i][j] = j <= i ? factor[i * SRUKF_N + j] : 0.0;
    return SRUKF_OK;
}

/*
 * Shared measurement update at the filter's CURRENT timestamp.
 * Does not predict forward.
 *
 * Regenerate sigma points from the current predicted state/S,
 * transform them to measurement space, calculate uncertainty,
 * gate the innovation, and apply the correction.
 *
 * R is the 4x4 VN covariance, or its leading scalar for the encoder.
 */
SrukfStatus srukf_measurement_update(
    Srukf *filter,
    SrukfSensor sensor,
    const double measurement[SRUKF_MAX_MEAS],
    const double R[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS],
    double gate_threshold,
    SrukfWorkspace *workspace
) {
    if (filter == NULL ||
        measurement == NULL ||
        R == NULL ||
        workspace == NULL ||
        !isfinite(gate_threshold)) {
        return SRUKF_INVALID_ARGUMENT;
    }

    if (!filter->initialized) {
        return SRUKF_NOT_INITIALIZED;
    }

    if (sensor != SRUKF_SENSOR_VN && sensor != SRUKF_SENSOR_ENCODER) {
        return SRUKF_INVALID_ARGUMENT;
    }

    const size_t m = srukf_measurement_dimension(sensor);

    if (m == 0 || m > SRUKF_MAX_MEAS) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /*
     * VN: [x, y, heading, omega] -> heading at index SRUKF_HEADING
     */
    const int angle_index =
        (sensor == SRUKF_SENSOR_VN) ? SRUKF_HEADING : -1;

    for (size_t j = 0; j < m; ++j) {
        if (!isfinite(measurement[j])) {
            return SRUKF_INVALID_ARGUMENT;
        }
    }

    /*
     * 1. Validate and factor the measurement-noise covariance.
     *
     * Only the active m-by-m block is used.
     */
    double R_flat[SRUKF_MAX_MEAS * SRUKF_MAX_MEAS] = {0};
    double noise_factor[SRUKF_MAX_MEAS * SRUKF_MAX_MEAS] = {0};

    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < m; ++j) {
            R_flat[i * SRUKF_MAX_MEAS + j] = R[i][j];
        }
    }

    SrukfStatus status = srmat_cholesky_lower(
        R_flat,
        m,
        SRUKF_MAX_MEAS,
        noise_factor,
        SRUKF_MAX_MEAS);

    if (status != SRUKF_OK) {
        return status;
    }

    /*
     * 2. Generate fresh sigma points from the current state.
     *
     * Local buffers keep these points intact when subsequent
     * matrix helpers use workspace as scratch memory.
     */
    double state_points[SRUKF_SIGMA_COUNT][SRUKF_N];
    double measurement_points
        [SRUKF_SIGMA_COUNT][SRUKF_MAX_MEAS] = {{0}};

    srukf_generate_sigma_points(
        filter->state,
        filter->sqrt_covariance,
        filter->sigma_scale,
        state_points);

    /*
     * 3. Map each state sigma point into measurement space.
     */
    double points_flat[SRUKF_SIGMA_COUNT * SRUKF_MAX_MEAS] = {0};

    for (size_t i = 0; i < SRUKF_SIGMA_COUNT; ++i) {
        for (size_t j = 0; j < SRUKF_N; ++j) {
            if (!isfinite(state_points[i][j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }

        srukf_measurement_model(
            sensor,
            state_points[i],
            measurement_points[i]);

        for (size_t j = 0; j < m; ++j) {
            if (!isfinite(measurement_points[i][j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            points_flat[i * SRUKF_MAX_MEAS + j] =
                measurement_points[i][j];
        }
    }

    /*
     * 4. Compute the predicted measurement mean.
     */
    double measurement_mean[SRUKF_MAX_MEAS] = {0};

    status = srukf_weighted_mean(
        points_flat,
        SRUKF_SIGMA_COUNT,
        m,
        SRUKF_MAX_MEAS,
        filter->mean_weights,
        angle_index,
        measurement_mean);

    if (status != SRUKF_OK) {
        return status;
    }

    /*
     * 5. Build innovation covariance factor Sz:
     *
     * Pzz = Sz * Sz^T
     *
     * This includes both predicted measurement uncertainty
     * and measurement noise R.
     */
    double Sz_flat[SRUKF_MAX_MEAS * SRUKF_MAX_MEAS] = {0};

    status = srukf_build_sqrt_covariance(
        points_flat,
        SRUKF_SIGMA_COUNT,
        m,
        SRUKF_MAX_MEAS,
        measurement_mean,
        filter->covariance_weights,
        angle_index,
        noise_factor,
        m,
        SRUKF_MAX_MEAS,
        Sz_flat,
        SRUKF_MAX_MEAS,
        workspace);

    if (status != SRUKF_OK) {
        return status;
    }

    double Sz[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS] = {{0}};

    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < m; ++j) {
            Sz[i][j] = Sz_flat[i * SRUKF_MAX_MEAS + j];
        }
    }

    /*
     * 6. Calculate the innovation, wrapping heading if present.
     */
    double residual[SRUKF_MAX_MEAS] = {0};

    srukf_residual(
        measurement,
        measurement_mean,
        m,
        angle_index,
        residual);

    for (size_t j = 0; j < m; ++j) {
        if (!isfinite(residual[j])) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    /*
     * 7. Reject measurements whose normalized innovation
     * exceeds the configured threshold.
     */
    if (gate_threshold > 0.0) {
        double nis = 0.0;

        status = srukf_innovation_score(
            m,
            residual,
            Sz,
            &nis);

        if (status != SRUKF_OK) {
            return status;
        }

        if (!isfinite(nis) || nis < 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }

        if (nis > gate_threshold) {
            return SRUKF_MEASUREMENT_REJECTED;
        }
    }

    /*
     * 8. Calculate state-measurement cross-covariance.
     */
    double Pxz[SRUKF_N][SRUKF_MAX_MEAS] = {{0}};

    srukf_cross_covariance(
        sensor,
        state_points,
        measurement_points,
        filter->state,
        measurement_mean,
        filter->covariance_weights,
        Pxz);

    for (size_t i = 0; i < SRUKF_N; ++i) {
        for (size_t j = 0; j < m; ++j) {
            if (!isfinite(Pxz[i][j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }
    }

    /*
     * 9. Calculate the Kalman gain using triangular solves.
     */
    double K[SRUKF_N][SRUKF_MAX_MEAS] = {{0}};

    status = srukf_compute_gain(m, Pxz, Sz, K);

    if (status != SRUKF_OK) {
        return status;
    }

    if (!filter->config.correct_unmeasured_states) {
        for (size_t i = 0; i < SRUKF_N; ++i) {
            const bool measured = sensor == SRUKF_SENSOR_VN ? i < 4 : i == SRUKF_V;
            if (!measured)
                for (size_t j = 0; j < m; ++j) K[i][j] = 0.0;
        }
    }
    double Lr[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS] = {{0}};
    for (size_t i = 0; i < m; ++i)
        for (size_t j = 0; j <= i; ++j)
            Lr[i][j] = noise_factor[i * SRUKF_MAX_MEAS + j];

    /*
     * 10. Apply the correction to a candidate.
     * Commit only after the state and covariance update succeed.
     */
    Srukf candidate = *filter;

    status = srukf_apply_correction(
        &candidate,
        sensor,
        residual,
        K,
        Lr,
        workspace);

    if (status != SRUKF_OK) {
        return status;
    }

    *filter = candidate;

    return SRUKF_OK;
}

/* Normalize an angle to [-pi, pi). */
double srukf_wrap_angle(double angle) {
    const double pi = 3.14159265358979323846;
    const double two_pi = 2.0 * pi;

    if (!isfinite(angle)) return NAN;

    double wrapped = fmod(angle, two_pi);

    if (wrapped >= pi) {
        wrapped -= two_pi;
    } else if (wrapped < -pi) {
        wrapped += two_pi;
    }

    return wrapped;
}


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
) {
    if (points == NULL ||
        weights == NULL ||
        mean == NULL ||
        point_count == 0 ||
        dimension == 0 ||
        point_stride < dimension ||
        angle_index < -1 ||
        (angle_index >= 0 && (size_t)angle_index >= dimension)) {
        return SRUKF_INVALID_ARGUMENT;
    }

    /* Validate inputs before calculating any output. */
    for (size_t i = 0; i < point_count; ++i) {
        if (!isfinite(weights[i])) {
            return SRUKF_INVALID_ARGUMENT;
        }

        for (size_t j = 0; j < dimension; ++j) {
            if (!isfinite(points[i * point_stride + j])) {
                return SRUKF_INVALID_ARGUMENT;
            }
        }
    }

    const double pi = 3.14159265358979323846;

    for (size_t j = 0; j < dimension; ++j) {
        const int is_angle =
            angle_index >= 0 && j == (size_t)angle_index;

        const double reference = is_angle
            ? srukf_wrap_angle(points[j])
            : 0.0;

        double sum = 0.0;
        double compensation = 0.0;

        for (size_t i = 0; i < point_count; ++i) {
            double value = points[i * point_stride + j];

            if (is_angle) {
                /*
                 * Wrap each angle before subtracting, avoiding
                 * overflow when finite input angles are huge.
                 */
                value = srukf_wrap_angle(value);
                value = srukf_wrap_angle(value - reference);
            }

            const double term = weights[i] * value;

            if (!isfinite(term)) {
                return SRUKF_NUMERICAL_FAILURE;
            }

            /*
             * Kahan compensated summation reduces roundoff
             * when positive and negative terms cancel.
             */
            const double adjusted = term - compensation;
            const double next_sum = sum + adjusted;

            compensation = (next_sum - sum) - adjusted;
            sum = next_sum;

            if (!isfinite(sum) || !isfinite(compensation)) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }

        if (is_angle) {
            /*
             * Require a consistent local angular branch.
             * Antipodal points or a mean that would require
             * changing a point's unwrap branch are ambiguous
             * for this local-mean implementation.
             */
            for (size_t i = 0; i < point_count; ++i) {
                if (weights[i] == 0.0) {
                    continue;
                }

                const double angle =
                    srukf_wrap_angle(points[i * point_stride + j]);

                const double offset =
                    srukf_wrap_angle(angle - reference);

                const double difference = offset - sum;

                if (fabs(offset) >= pi ||
                    !isfinite(difference) ||
                    fabs(difference) >= pi) {
                    return SRUKF_NUMERICAL_FAILURE;
                }
            }

            /* Wrap sum first to keep the final addition bounded. */
            mean[j] = srukf_wrap_angle(
                reference + srukf_wrap_angle(sum));
        } else {
            mean[j] = sum;
        }
    }

    return SRUKF_OK;
}


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
) {
    for (size_t i = 0; i < dimension; ++i) {
        if (angle_index >= 0 && i == (size_t)angle_index) {
            /*
             * Wrap inputs first to avoid overflowing the
             * subtraction when finite angles are very large.
             */
            const double angle_a = srukf_wrap_angle(a[i]);
            const double angle_b = srukf_wrap_angle(b[i]);

            residual[i] = srukf_wrap_angle(angle_a - angle_b);
        } else {
            residual[i] = a[i] - b[i];
        }
    }
}


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
) {
    /*
     * This implementation uses the project's fixed maximum sizes.
     * A NULL noise_factor is allowed when noise_columns is zero.
     */
    if (points == NULL ||
        mean == NULL ||
        covariance_weights == NULL ||
        S_out == NULL ||
        workspace == NULL ||
        dimension == 0 ||
        dimension > SRUKF_N ||
        point_count == 0 ||
        point_count > SRUKF_SIGMA_COUNT ||
        point_stride < dimension ||
        S_stride < dimension ||
        angle_index < -1 ||
        (angle_index >= 0 && (size_t)angle_index >= dimension) ||
        noise_columns > SRUKF_N ||
        (noise_columns > 0 &&
         (noise_factor == NULL || noise_stride < noise_columns))) {
        return SRUKF_INVALID_ARGUMENT;
    }

    for (size_t j = 0; j < dimension; ++j) {
        if (!isfinite(mean[j])) {
            return SRUKF_INVALID_ARGUMENT;
        }
    }

    /*
     * Store sqrt(abs(weight)) * residual for each point.
     * Negative-weight vectors will be applied after QR.
     */
    double deviations[SRUKF_SIGMA_COUNT][SRUKF_N] = {{0}};

    for (size_t i = 0; i < point_count; ++i) {
        if (!isfinite(covariance_weights[i])) {
            return SRUKF_INVALID_ARGUMENT;
        }

        for (size_t j = 0; j < dimension; ++j) {
            if (!isfinite(points[i * point_stride + j])) {
                return SRUKF_INVALID_ARGUMENT;
            }
        }

        /* Zero-weight points contribute nothing. */
        if (covariance_weights[i] == 0.0) {
            continue;
        }

        srukf_residual(
            &points[i * point_stride],
            mean,
            dimension,
            angle_index,
            deviations[i]);

        const double scale = sqrt(fabs(covariance_weights[i]));

        for (size_t j = 0; j < dimension; ++j) {
            deviations[i][j] *= scale;

            if (!isfinite(deviations[i][j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }
    }

    /*
     * B contains positive-weight deviation columns followed by
     * noise-factor columns. QR produces S with S*S^T = B*B^T.
     *
     * Keep B separate from workspace because the QR helper may
     * overwrite workspace's own QR buffers.
     */
    double B[SRUKF_N * SRUKF_QR_MAX_COLUMNS] = {0};
    size_t column_count = 0;

    for (size_t i = 0; i < point_count; ++i) {
        if (covariance_weights[i] <= 0.0) {
            continue;
        }

        if (column_count >= SRUKF_QR_MAX_COLUMNS) {
            return SRUKF_INVALID_ARGUMENT;
        }

        for (size_t j = 0; j < dimension; ++j) {
            B[j * SRUKF_QR_MAX_COLUMNS + column_count] =
                deviations[i][j];
        }

        ++column_count;
    }

    for (size_t k = 0; k < noise_columns; ++k) {
        if (column_count >= SRUKF_QR_MAX_COLUMNS) {
            return SRUKF_INVALID_ARGUMENT;
        }

        for (size_t j = 0; j < dimension; ++j) {
            const double value = noise_factor[j * noise_stride + k];

            if (!isfinite(value)) {
                return SRUKF_INVALID_ARGUMENT;
            }

            B[j * SRUKF_QR_MAX_COLUMNS + column_count] = value;
        }

        ++column_count;
    }

    /*
     * Fewer columns than rows cannot produce a positive-definite
     * covariance. The QR helper must also detect rank deficiency
     * when there are enough columns but they are dependent.
     */
    if (column_count < dimension) {
        return SRUKF_NUMERICAL_FAILURE;
    }

    double factor[SRUKF_N * SRUKF_N] = {0};

    SrukfStatus status = srmat_qr_lower_factor(
        B,
        dimension,
        column_count,
        SRUKF_QR_MAX_COLUMNS,
        factor,
        SRUKF_N,
        workspace);

    if (status != SRUKF_OK) {
        return status;
    }

    /*
     * Remove negative-weight contributions:
     *
     * P_new = P_old - u*u^T
     *
     * The rank-one helper may overwrite u.
     */
    for (size_t i = 0; i < point_count; ++i) {
        if (covariance_weights[i] >= 0.0) {
            continue;
        }

        double u[SRUKF_N] = {0};

        for (size_t j = 0; j < dimension; ++j) {
            u[j] = deviations[i][j];
        }

        status = srmat_cholesky_rank1(
            factor,
            dimension,
            SRUKF_N,
            u,
            -1);

        if (status != SRUKF_OK) {
            return status;
        }
    }

    /* Validate before modifying the caller's output. */
    for (size_t i = 0; i < dimension; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            if (!isfinite(factor[i * SRUKF_N + j])) {
                return SRUKF_NUMERICAL_FAILURE;
            }
        }

        if (factor[i * SRUKF_N + i] <= 0.0) {
            return SRUKF_NUMERICAL_FAILURE;
        }
    }

    /* Commit the lower triangle and explicitly zero the upper. */
    for (size_t i = 0; i < dimension; ++i) {
        for (size_t j = 0; j < dimension; ++j) {
            S_out[i * S_stride + j] =
                (j <= i) ? factor[i * SRUKF_N + j] : 0.0;
        }
    }

    return SRUKF_OK;
}
