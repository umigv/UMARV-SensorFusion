#include "kalman.h"
#include "kalman_internal.h"
#include "kalman_math.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


#include "kalman_workspace.h"

/*
 * Example uncertainties only: replace with sensor calibration
 * or receiver-provided values.
 */
#define GPS_POSITION_STD       1.5     /* meters */
#define ODOM_VELOCITY_STD       0.05    /* meters/second */
#define ODOM_HEADING_STD        0.035   /* radians, about 2 degrees */
#define ODOM_ANGULAR_RATE_STD   0.02    /* radians/second */

/*
 * Startup approximation: require GPS and odometry timestamps
 * to be close enough to treat them as simultaneous.
 */
#define INITIAL_SYNC_TOLERANCE  0.05    /* seconds */

static double square(double x)
{
    return x * x;
}

static void print_estimate(
    const Srukf *filter,
    const char *update_status)
{
    SrukfEstimate estimate;

    SrukfStatus status = srukf_get_estimate(filter, &estimate);

    if (status != SRUKF_OK) {
        fprintf(stderr, "Cannot read estimate: %s\n",
                srukf_status_string(status));
        return;
    }

    /*
     * Output:
     * E timestamp x y velocity heading angular_velocity status
     */
    printf("E %.6f %.6f %.6f %.6f %.6f %.6f %s\n",
           estimate.timestamp,
           estimate.x,
           estimate.y,
           estimate.velocity,
           estimate.heading,
           estimate.angular_velocity,
           update_status);

    fflush(stdout);
}


//Top level function, reads data packets from the terminal (stdin) in the form <G/O> <timestamp> <X/v> <Y/heading> <ang_velocity>
//and then outputs to the terminal (stdout) its best current filter estimate or whatever after reading each packet
void filter(void)
{
    Srukf estimator = {0};
    SrukfWorkspace workspace = {0};
    SrukfConfig config;

    SrukfGpsMeasurement initial_gps = {0};
    SrukfOdometryMeasurement initial_odometry = {0};

    bool have_initial_gps = false;
    bool have_initial_odometry = false;

    bool have_last_input_time = false;
    double last_input_time = 0.0;

    char line[512];

    srukf_config_default(&config);

    /*
     * Illustrative starting configuration.
     * Tune process noise and measurement uncertainties together.
     */
    config.alpha = 0.5;
    config.beta = 2.0;
    config.kappa = 0.0;

    config.linear_accel_noise_density = 0.25;
    config.angular_accel_noise_density = 0.01;

    config.max_prediction_dt = 0.02;

    config.gps_gate_threshold = 9.21;
    config.odometry_gate_threshold = 11.34;

    SrukfStatus status = srukf_config_validate(&config);

    if (status != SRUKF_OK) {
        fprintf(stderr, "Invalid configuration: %s\n",
                srukf_status_string(status));
        return;
    }

    fprintf(stderr,
            "Enter packets:\n"
            "  G timestamp x y [ignored_field]\n"
            "  O timestamp velocity heading angular_velocity\n"
            "Send EOF to stop.\n");

    while (fgets(line, sizeof line, stdin) != NULL) {
        /*
         * Reject lines that exceed the input buffer instead of
         * accidentally treating the remainder as another packet.
         */
        if (strchr(line, '\n') == NULL && !feof(stdin)) {
            int ch;

            while ((ch = getchar()) != '\n' && ch != EOF) {
                /* Discard remainder. */
            }

            fprintf(stderr, "ERROR: input line too long\n");

            if (estimator.initialized) {
                print_estimate(&estimator, "invalid_packet");
            }
            continue;
        }

        char *start = line;

        while (isspace((unsigned char)*start)) {
            ++start;
        }

        /* Allow blank lines and full-line comments. */
        if (*start == '\0' || *start == '#') {
            continue;
        }

        char type = '\0';
        char extra = '\0';

        double timestamp = 0.0;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;

        /*
         * GPS:      a=x,        b=y,       c=optional/ignored
         * Odometry: a=velocity, b=heading, c=angular_velocity
         *
         * The final %c detects unexpected trailing data.
         */
        int fields = sscanf(start, " %c %lf %lf %lf %lf %c",
                            &type, &timestamp, &a, &b, &c, &extra);

        type = (char)toupper((unsigned char)type);

        bool valid =
            (type == 'G' && fields == 4) ||
            (type == 'O' && fields == 5);

        if (valid) {
            valid = isfinite(timestamp) &&
                    isfinite(a) &&
                    isfinite(b) &&
                    (fields != 5 || isfinite(c));
        }

        if (!valid) {
            fprintf(stderr, "ERROR: malformed packet\n");

            if (estimator.initialized) {
                print_estimate(&estimator, "invalid_packet");
            }
            continue;
        }

        /*
         * This wrapper does not buffer and replay delayed data.
         * Equal timestamps are allowed.
         */
        if (have_last_input_time && timestamp < last_input_time) {
            fprintf(stderr,
                    "ERROR: out-of-order timestamp %.6f "
                    "(last input %.6f)\n",
                    timestamp, last_input_time);

            if (estimator.initialized) {
                print_estimate(&estimator, "stale_packet");
            }
            continue;
        }

        last_input_time = timestamp;
        have_last_input_time = true;

        SrukfGpsMeasurement gps = {0};
        SrukfOdometryMeasurement odometry = {0};

        if (type == 'G') {
            gps.timestamp = timestamp;
            gps.x = a;
            gps.y = b;

            gps.covariance[0][0] = square(GPS_POSITION_STD);
            gps.covariance[1][1] = square(GPS_POSITION_STD);
        } else {
            odometry.timestamp = timestamp;
            odometry.velocity = a;

            /* Normalize heading to a principal angular interval. */
            odometry.heading = atan2(sin(b), cos(b));

            odometry.angular_velocity = c;

            odometry.covariance[0][0] = square(ODOM_VELOCITY_STD);
            odometry.covariance[1][1] = square(ODOM_HEADING_STD);
            odometry.covariance[2][2] = square(ODOM_ANGULAR_RATE_STD);
        }

        /*
         * Initialization requires both position and motion data.
         * Retain the newest packet from each source.
         */
        if (!estimator.initialized) {
            if (type == 'G') {
                initial_gps = gps;
                have_initial_gps = true;
            } else {
                initial_odometry = odometry;
                have_initial_odometry = true;
            }

            if (!have_initial_gps || !have_initial_odometry) {
                printf("WAIT need_initial_%s\n",
                       have_initial_gps ? "odometry" : "gps");
                fflush(stdout);
                continue;
            }

            double separation = fabs(
                initial_gps.timestamp - initial_odometry.timestamp);

            if (separation > INITIAL_SYNC_TOLERANCE) {
                printf("WAIT initial_timestamps_too_far_apart\n");
                fflush(stdout);
                continue;
            }

            double initial_state[SRUKF_N] = {
                initial_gps.x,
                initial_gps.y,
                initial_odometry.velocity,
                initial_odometry.heading,
                initial_odometry.angular_velocity
            };

            double initial_covariance[SRUKF_N][SRUKF_N] = {{0}};

            initial_covariance[SRUKF_X][SRUKF_X] =
                square(GPS_POSITION_STD);

            initial_covariance[SRUKF_Y][SRUKF_Y] =
                square(GPS_POSITION_STD);

            initial_covariance[SRUKF_V][SRUKF_V] =
                square(ODOM_VELOCITY_STD);

            initial_covariance[SRUKF_HEADING][SRUKF_HEADING] =
                square(ODOM_HEADING_STD);

            initial_covariance[SRUKF_OMEGA][SRUKF_OMEGA] =
                square(ODOM_ANGULAR_RATE_STD);

            /*
             * Inputs are chronological, so timestamp is the
             * later of the two initialization timestamps.
             */
            status = srukf_init(
                &estimator,
                &config,
                timestamp,
                initial_state,
                initial_covariance);

            if (status != SRUKF_OK) {
                fprintf(stderr, "Initialization failed: %s\n",
                        srukf_status_string(status));
                return;
            }

            print_estimate(&estimator, "initialized");

            /*
             * Both startup measurements were used to initialize.
             * Do not fuse the current packet again.
             */
            continue;
        }

        if (type == 'G') {
            status = srukf_process_gps(
                &estimator, &gps, &workspace);
        } else {
            status = srukf_process_odometry(
                &estimator, &odometry, &workspace);
        }

        if (status == SRUKF_OK) {
            print_estimate(&estimator, "updated");
        } else if (status == SRUKF_MEASUREMENT_REJECTED) {
            /*
             * Per our API contract, successful prediction remains
             * committed even when the measurement is gated out.
             */
            print_estimate(&estimator, "measurement_rejected");
        } else {
            fprintf(stderr, "Filter error: %s\n",
                    srukf_status_string(status));

            /* Print the last valid estimate with its own timestamp. */
            print_estimate(&estimator, "update_failed");
        }
    }

    if (ferror(stdin)) {
        fprintf(stderr, "ERROR: failed to read stdin\n");
    }
}


int main() {
    filter();
}