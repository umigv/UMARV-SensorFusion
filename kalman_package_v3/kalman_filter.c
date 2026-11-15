#include "kalman_interface.h"
#include "kalman_internal.h"
#include "kalman_math.h"
#include "kalman_workspace.h"

#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Match simulate_buggy_packets / Maverick sensor defaults:
 * G = VN fused position; O = encoder speed, VN heading, encoder yaw rate.
 * Use white-noise variance + stationary drift variance. These are marginal
 * covariances; the five-state filter does not model temporal correlation.
 */
#define GPS_POSITION_NOISE_STD      0.05    /* m */
#define GPS_POSITION_DRIFT_STD      0.5     /* m */
#define ODOM_VELOCITY_NOISE_STD      0.01    /* m/s */
#define ODOM_VELOCITY_SCALE_STD      0.02    /* dimensionless */
#define ODOM_HEADING_NOISE_STD      0.008726646259971648 /* rad: 0.5 deg */
#define ODOM_HEADING_DRIFT_STD      0.005235987755982988 /* rad: 0.3 deg */
#define ODOM_ANGULAR_RATE_NOISE_STD  0.01    /* rad/s */
#define ODOM_ANGULAR_RATE_SCALE_STD  0.02    /* dimensionless */

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
    SrukfStatus status = srukf_config_validate(&config);

    if (status != SRUKF_OK) {
        fprintf(stderr, "Invalid configuration: %s\n",
                srukf_status_string(status));
        return;
    }

    fprintf(stderr,
            "Enter packets:\n"
            "  G timestamp x y\n"
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

        char type = (char)toupper((unsigned char)*start++);
        double values[4] = {0};
        size_t count = (type == 'G') ? 3 : 4;
        bool valid = (type == 'G' || type == 'O') &&
                     isspace((unsigned char)*start);

        /* Require complete whitespace-separated numeric tokens. */
        for (size_t i = 0; valid && i < count; ++i) {
            while (isspace((unsigned char)*start)) ++start;
            char *end;
            errno = 0;
            values[i] = strtod(start, &end);
            valid = end != start && errno != ERANGE &&
                    isfinite(values[i]) &&
                    (*end == '\0' || isspace((unsigned char)*end));
            start = end;
        }
        while (isspace((unsigned char)*start)) ++start;
        valid = valid && *start == '\0';
        double timestamp = values[0];
        double a = values[1], b = values[2], c = values[3];

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

        /* During startup track the buffered packets; after initialization
         * only committed filter timestamps advance the input watermark. */
        if (!estimator.initialized) {
            last_input_time = timestamp;
            have_last_input_time = true;
        }

        SrukfGpsMeasurement gps = {0};
        SrukfOdometryMeasurement odometry = {0};

        if (type == 'G') {
            gps.timestamp = timestamp;
            gps.x = a;
            gps.y = b;

            gps.covariance[0][0] = square(GPS_POSITION_NOISE_STD) +
                                   square(GPS_POSITION_DRIFT_STD);
            gps.covariance[1][1] = gps.covariance[0][0];
        } else {
            odometry.timestamp = timestamp;
            odometry.velocity = a;

            /* Normalize heading to a principal angular interval. */
            odometry.heading = srukf_wrap_angle(b);

            odometry.angular_velocity = c;

            /* Match Python's measurement-dependent encoder covariance. */
            odometry.covariance[0][0] = square(ODOM_VELOCITY_NOISE_STD) +
                square(ODOM_VELOCITY_SCALE_STD * odometry.velocity);
            odometry.covariance[1][1] = square(ODOM_HEADING_NOISE_STD) +
                                      square(ODOM_HEADING_DRIFT_STD);
            odometry.covariance[2][2] = square(ODOM_ANGULAR_RATE_NOISE_STD) +
                square(ODOM_ANGULAR_RATE_SCALE_STD * odometry.angular_velocity);
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
                initial_gps.covariance[0][0];

            initial_covariance[SRUKF_Y][SRUKF_Y] =
                initial_gps.covariance[1][1];

            initial_covariance[SRUKF_V][SRUKF_V] =
                initial_odometry.covariance[0][0];

            initial_covariance[SRUKF_HEADING][SRUKF_HEADING] =
                initial_odometry.covariance[1][1];

            initial_covariance[SRUKF_OMEGA][SRUKF_OMEGA] =
                initial_odometry.covariance[2][2];

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

        last_input_time = estimator.timestamp;
        have_last_input_time = true;

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


int main(void) {
    filter();
    return ferror(stdin) || ferror(stdout) ? EXIT_FAILURE : EXIT_SUCCESS;
}
