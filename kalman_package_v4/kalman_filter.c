#include "kalman_interface.h"
#include "kalman_workspace.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Maverick simulator output-error defaults, not raw IMU specifications.
 * VN = fused x/y, heading, compensated angular rate; E = encoder speed.
 * Drift variances are marginal approximations, not bias-state estimation.
 */
#define VN_POSITION_NOISE_STD 0.05
#define VN_POSITION_DRIFT_STD 0.5
#define VN_HEADING_NOISE_STD 0.008726646259971648 /* 0.5 deg */
#define VN_HEADING_DRIFT_STD 0.005235987755982988 /* 0.3 deg */
#define VN_ANGULAR_RATE_NOISE_STD 0.005
#define VN_ANGULAR_RATE_DRIFT_STD 0.002
#define ENCODER_VELOCITY_NOISE_STD 0.01
#define ENCODER_VELOCITY_SCALE_STD 0.02

static double square(double x) { return x * x; }

static void print_estimate(const Srukf *filter, const char *update_status)
{
    SrukfEstimate estimate;
    SrukfStatus status = srukf_get_estimate(filter, &estimate);
    if (status != SRUKF_OK) {
        fprintf(stderr, "Cannot read estimate: %s\n", srukf_status_string(status));
        return;
    }
    /* F avoids confusing estimates with E encoder input packets. */
    printf("F %.9f %.9f %.9f %.9f %.9f %s\n",
           estimate.timestamp, estimate.x, estimate.y,
           estimate.heading, estimate.angular_velocity, update_status);
    fflush(stdout);
}

/* Read VN/E text packets, report one estimate/status for each nonblank line. */
void filter(void)
{
    Srukf estimator = {0};
    SrukfWorkspace workspace = {0};
    SrukfConfig config;
    bool have_last_input_time = false;
    double last_input_time = 0.0;
    char line[512];
    srukf_config_default(&config);
    SrukfStatus status = srukf_config_validate(&config);
    if (status != SRUKF_OK) {
        fprintf(stderr, "Invalid configuration: %s\n", srukf_status_string(status));
        return;
    }
    fprintf(stderr, "Enter packets:\n"
                    "  VN timestamp x y heading angular_velocity\n"
                    "  E timestamp velocity\n"
                    "Use seconds, meters, radians; E before VN at equal times.\n"
                    "Send EOF to stop.\n");
    while (fgets(line, sizeof line, stdin) != NULL) {
        if (strchr(line, '\n') == NULL && !feof(stdin)) {
            int ch;
            while ((ch = getchar()) != '\n' && ch != EOF) { }
            fprintf(stderr, "ERROR: input line too long\n");
            if (estimator.initialized) print_estimate(&estimator, "invalid_packet");
            else puts("WAIT invalid_packet");
            fflush(stdout);
            continue;
        }
        char *start = line;
        while (isspace((unsigned char)*start)) ++start;
        if (*start == '\0' || *start == '#') continue;
        char *end_type = start;
        while (*end_type && !isspace((unsigned char)*end_type)) ++end_type;
        const size_t type_length = (size_t)(end_type - start);
        const bool is_vn = type_length == 2 &&
            toupper((unsigned char)start[0]) == 'V' &&
            toupper((unsigned char)start[1]) == 'N';
        const bool is_encoder = type_length == 1 &&
            toupper((unsigned char)start[0]) == 'E';
        bool valid = is_vn || is_encoder;
        const size_t count = is_vn ? 5 : 2;
        double values[5] = {0};
        start = end_type;
        for (size_t i = 0; valid && i < count; ++i) {
            while (isspace((unsigned char)*start)) ++start;
            char *end;
            errno = 0;
            values[i] = strtod(start, &end);
            valid = end != start && errno != ERANGE && isfinite(values[i]) &&
                    (*end == '\0' || isspace((unsigned char)*end));
            start = end;
        }
        while (isspace((unsigned char)*start)) ++start;
        valid = valid && *start == '\0';
        if (!valid) {
            fprintf(stderr, "ERROR: malformed packet\n");
            if (estimator.initialized) print_estimate(&estimator, "invalid_packet");
            else puts("WAIT invalid_packet");
            fflush(stdout);
            continue;
        }
        const double timestamp = values[0];
        if (have_last_input_time && timestamp < last_input_time) {
            fprintf(stderr, "ERROR: out-of-order timestamp %.9f (last %.9f)\n",
                    timestamp, last_input_time);
            if (estimator.initialized) print_estimate(&estimator, "stale_packet");
            else puts("WAIT stale_packet");
            fflush(stdout);
            continue;
        }
        SrukfVnMeasurement vn = {0};
        SrukfEncoderMeasurement encoder = {0};
        if (is_vn) {
            vn.timestamp = timestamp;
            vn.x = values[1]; vn.y = values[2];
            vn.heading = values[3]; vn.angular_velocity = values[4];
            vn.covariance[SRUKF_X][SRUKF_X] =
                square(VN_POSITION_NOISE_STD) + square(VN_POSITION_DRIFT_STD);
            vn.covariance[SRUKF_Y][SRUKF_Y] = vn.covariance[SRUKF_X][SRUKF_X];
            vn.covariance[SRUKF_HEADING][SRUKF_HEADING] =
                square(VN_HEADING_NOISE_STD) + square(VN_HEADING_DRIFT_STD);
            vn.covariance[SRUKF_OMEGA][SRUKF_OMEGA] =
                square(VN_ANGULAR_RATE_NOISE_STD) + square(VN_ANGULAR_RATE_DRIFT_STD);
        } else {
            encoder.timestamp = timestamp;
            encoder.velocity = values[1];
            encoder.variance = square(ENCODER_VELOCITY_NOISE_STD) +
                square(ENCODER_VELOCITY_SCALE_STD * encoder.velocity);
            if (!isfinite(encoder.variance)) {
                fprintf(stderr, "ERROR: encoder variance overflow\n");
                if (estimator.initialized) print_estimate(&estimator, "invalid_packet");
                else puts("WAIT invalid_packet");
                fflush(stdout);
                continue;
            }
        }
        if (!estimator.initialized) {
            if (!is_vn) {
                last_input_time = timestamp;
                have_last_input_time = true;
                puts("WAIT need_initial_vn");
                fflush(stdout);
                continue;
            }
            const double initial_state[SRUKF_N] = {
                vn.x, vn.y, vn.heading, vn.angular_velocity
            };
            status = srukf_init(&estimator, &config, timestamp,
                                initial_state, vn.covariance);
            if (status != SRUKF_OK) {
                fprintf(stderr, "Initialization failed: %s\n",
                        srukf_status_string(status));
                puts("WAIT initialization_failed");
                fflush(stdout);
                continue;
            }
            last_input_time = estimator.timestamp;
            have_last_input_time = true;
            print_estimate(&estimator, "initialized");
            continue; /* The first VN is used once, to initialize. */
        }
        status = is_vn ? srukf_process_vn(&estimator, &vn, &workspace)
                       : srukf_process_encoder(&estimator, &encoder, &workspace);
        last_input_time = estimator.timestamp;
        have_last_input_time = true;
        if (status == SRUKF_OK)
            print_estimate(&estimator, is_vn ? "vn_updated" : "encoder_predicted");
        else if (status == SRUKF_MEASUREMENT_REJECTED)
            print_estimate(&estimator, "measurement_rejected");
        else {
            fprintf(stderr, "Filter error: %s\n", srukf_status_string(status));
            print_estimate(&estimator, "update_failed");
        }
    }
    if (ferror(stdin)) fprintf(stderr, "ERROR: failed to read stdin\n");
}

int main(void)
{
    filter();
    return ferror(stdin) || ferror(stdout) ? EXIT_FAILURE : EXIT_SUCCESS;
}
