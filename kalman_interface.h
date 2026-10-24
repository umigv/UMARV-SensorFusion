#ifndef SRUKF_IF
#define SRUKF_IF

#define SRUKF_N           5
#define SRUKF_SIGMA_COUNT 11
#define SRUKF_MAX_MEAS    3

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "kalman_internal.h"
#include "kalman_math.h"
#include "kalman_workspace.h"

typedef enum {
    SRUKF_X = 0,
    SRUKF_Y,
    SRUKF_V,
    SRUKF_HEADING,
    SRUKF_OMEGA
} SrukfStateIndex;

typedef enum {
    SRUKF_OK = 0,
    SRUKF_NOT_INITIALIZED,
    SRUKF_INVALID_ARGUMENT,
    SRUKF_INVALID_TIME,
    SRUKF_MEASUREMENT_REJECTED,
    SRUKF_NUMERICAL_FAILURE
} SrukfStatus;

typedef enum {
    SRUKF_SENSOR_GPS,
    SRUKF_SENSOR_ODOMETRY
} SrukfSensor;

typedef struct {
    double timestamp;
    double x;
    double y;

    /* Measurement covariance, not its square root. */
    double covariance[2][2];
} SrukfGpsMeasurement;

typedef struct {
    double timestamp;
    double velocity;
    double heading;
    double angular_velocity;

    /* Order: velocity, heading, angular velocity. */
    double covariance[3][3];
} SrukfOdometryMeasurement;

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

    /* Split long predictions into smaller steps. */
    double max_prediction_dt;

    /*
     * Thresholds for squared Mahalanobis distance.
     * A value <= 0 disables the corresponding gate.
     */
    double gps_gate_threshold;
    double odometry_gate_threshold;
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
    double velocity;
    double heading;
    double angular_velocity;

    double covariance[SRUKF_N][SRUKF_N];
} SrukfEstimate;

/*
 * Reusable temporary storage.
 * Define its fixed-size arrays in srukf_workspace.h.
 */
typedef struct SrukfWorkspace SrukfWorkspace;

/*
 * Fill configuration with documented starting values.
 * Noise settings still require tuning for the actual vehicle.
 */
void srukf_config_default(SrukfConfig *config);


/*
 * Check parameter ranges, finite values, positive time limits,
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
const char *srukf_status_string(SrukfStatus status);

#endif