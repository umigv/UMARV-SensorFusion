#ifndef SRUKF_IF
#define SRUKF_IF

#define SRUKF_N           5
#define SRUKF_SIGMA_COUNT (2 * SRUKF_N + 1)
#define SRUKF_MAX_MEAS    4

#include <stdbool.h>
#include <stddef.h>


typedef enum {
    SRUKF_X = 0,
    SRUKF_Y,
    SRUKF_HEADING,
    SRUKF_OMEGA,
    SRUKF_V
} SrukfStateIndex;

typedef enum {
    SRUKF_OK = 0,
    SRUKF_NOT_INITIALIZED,
    SRUKF_INVALID_ARGUMENT,
    SRUKF_INVALID_TIME,
    SRUKF_MEASUREMENT_REJECTED,
    SRUKF_NUMERICAL_FAILURE
} SrukfStatus;

/* VN observes four navigation states; encoder observes signed speed. */
typedef enum {
    SRUKF_SENSOR_VN,
    SRUKF_SENSOR_ENCODER
} SrukfSensor;

typedef struct {
    double timestamp;
    double x;
    double y;
    double heading;
    double angular_velocity;
    /* Full measurement covariance in state order: x, y, heading, omega. */
    double covariance[SRUKF_MAX_MEAS][SRUKF_MAX_MEAS];
} SrukfVnMeasurement;

typedef struct {
    double timestamp;
    double velocity;  /* Signed forward speed at timestamp, m/s. */
    double variance;  /* Measurement variance in (m/s)^2; strictly positive. */
} SrukfEncoderMeasurement;

typedef struct {
    /* Unscented-transform parameters. */
    double alpha;
    double beta;
    double kappa;

    /*
     * Continuous white-noise spectral densities:
     * linear acceleration:  m^2 / s^3
     * angular acceleration: rad^2 / s^3
     */
    double linear_accel_noise_density;
    double angular_accel_noise_density;

    /*
     * Four-dimensional VN squared Mahalanobis distance threshold.
     * A value <= 0 disables the corresponding sensor gate.
     */
    double vn_gate_threshold;
    double encoder_gate_threshold;

    /* False: retain predicted means for unmeasured states, as requested.
     * True: standard joint Kalman correction through cross-covariances. */
    bool correct_unmeasured_states;
} SrukfConfig;

typedef struct {
    double timestamp;
    double state[SRUKF_N];
    double sqrt_covariance[SRUKF_N][SRUKF_N];

    double mean_weights[SRUKF_SIGMA_COUNT];
    double covariance_weights[SRUKF_SIGMA_COUNT];
    double sigma_scale;

    SrukfConfig config;
    bool initialized;
} Srukf;

typedef struct {
    double timestamp;
    double x;
    double y;
    double heading;
    double angular_velocity;
    double velocity;

    double covariance[SRUKF_N][SRUKF_N];
} SrukfEstimate;

/*
 * Reusable temporary storage.
 * Define its fixed-size arrays in kalman_workspace.h.
 */
typedef struct SrukfWorkspace SrukfWorkspace;

/*
 * Fill configuration with documented starting values.
 * Noise settings still require tuning for the actual vehicle.
 */
void srukf_config_default(SrukfConfig *config);


/*
 * Check parameter ranges, finite values, nonnegative noise densities,
 * and whether the sigma-point scaling is valid.
 */
SrukfStatus srukf_config_validate(const SrukfConfig *config);


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
);


/*
 * Clear the filter and mark it uninitialized.
 * Call srukf_init() before using it again.
 */
void srukf_reset(Srukf *filter);


/*
 * Predict with the stored velocity and angular velocity over the elapsed time.
 * One midpoint-model step per event, used before either sensor correction.
 * Equal timestamps are no-ops; older timestamps are rejected.
 */
SrukfStatus srukf_predict_to(
    Srukf *filter,
    double timestamp,
    SrukfWorkspace *workspace
);

/*
 * Predict to the packet time, then correct x/y/heading/omega using VN.
 * The timestamp advances on success or innovation rejection. Other failures
 * leave the filter unchanged. Initialize explicitly with srukf_init first.
 */
SrukfStatus srukf_process_vn(
    Srukf *filter,
    const SrukfVnMeasurement *measurement,
    SrukfWorkspace *workspace
);

/*
 * Predict to the packet time, then correct velocity using the encoder.
 * A gated measurement retains the prediction; other errors leave the filter
 * unchanged. Equal-time packets correct without propagating motion twice.
 */
SrukfStatus srukf_process_encoder(
    Srukf *filter,
    const SrukfEncoderMeasurement *measurement,
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
const char *srukf_status_string(SrukfStatus status);

#endif
