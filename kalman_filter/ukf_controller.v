`timescale 1ns/1ps

module ukf_controller (
    input wire clk,
    input wire resetn,

    // Scheduler / sensor interface
    input wire sensor_data_ready,
    input wire processor_enable,
    output wire sensor_data_accept,

    // Controller status
    output reg busy,
    output reg filter_done,

    // Sigma point generation
    output reg sigma_start,
    input wire sigma_done,

    // CORDIC / motion preparation
    output reg propagate_start,
    input wire propagate_done,

    // Sigma point state update
    output reg update_start,
    input wire update_done,

    // Predicted mean
    output reg mean_start,
    input wire mean_done,

    // Predicted covariance
    output reg covariance_start,
    input wire covariance_done,

    // Sigma point residuals
    output reg residual_start,
    input wire residual_done,

    // Debugging
    output wire [2:0] debug_state
);

    // State definitions
    localparam [2:0]
        S0_IDLE       = 3'd0,
        S1_SIGMA      = 3'd1,
        S2_PROPAGATE  = 3'd2,
        S3_UPDATE     = 3'd3,
        S4_MEAN       = 3'd4,
        S5_COVARIANCE = 3'd5,
        S6_RESIDUAL   = 3'd6;

    reg [2:0] state;

    // Tracks whether a start pulse has been issued
    reg launched;

    assign debug_state = state;

    // Accept sensor data only while idle and enabled
    assign sensor_data_accept =
        (state == S0_IDLE) && processor_enable;

    always @(posedge clk) begin
        if (!resetn) begin
            state <= S0_IDLE;
            launched <= 1'b0;
            busy <= 1'b0;
            filter_done <= 1'b0;

            sigma_start <= 1'b0;
            propagate_start <= 1'b0;
            update_start <= 1'b0;
            mean_start <= 1'b0;
            covariance_start <= 1'b0;
            residual_start <= 1'b0;
        end
        else begin
            // Default all one-cycle pulses to zero
            filter_done <= 1'b0;
            sigma_start <= 1'b0;
            propagate_start <= 1'b0;
            update_start <= 1'b0;
            mean_start <= 1'b0;
            covariance_start <= 1'b0;
            residual_start <= 1'b0;

            case (state)

                S0_IDLE: begin
                    busy <= 1'b0;
                    launched <= 1'b0;

                    if (sensor_data_ready &&
                        processor_enable) begin
                        busy <= 1'b1;
                        state <= S1_SIGMA;
                    end
                end

                S1_SIGMA: begin
                    if (!launched) begin
                        sigma_start <= 1'b1;
                        launched <= 1'b1;
                    end
                    else if (sigma_done) begin
                        launched <= 1'b0;
                        state <= S2_PROPAGATE;
                    end
                end

                S2_PROPAGATE: begin
                    if (!launched) begin
                        propagate_start <= 1'b1;
                        launched <= 1'b1;
                    end
                    else if (propagate_done) begin
                        launched <= 1'b0;
                        state <= S3_UPDATE;
                    end
                end

                S3_UPDATE: begin
                    if (!launched) begin
                        update_start <= 1'b1;
                        launched <= 1'b1;
                    end
                    else if (update_done) begin
                        launched <= 1'b0;
                        state <= S4_MEAN;
                    end
                end

                S4_MEAN: begin
                    if (!launched) begin
                        mean_start <= 1'b1;
                        launched <= 1'b1;
                    end
                    else if (mean_done) begin
                        launched <= 1'b0;
                        state <= S5_COVARIANCE;
                    end
                end

                S5_COVARIANCE: begin
                    if (!launched) begin
                        covariance_start <= 1'b1;
                        launched <= 1'b1;
                    end
                    else if (covariance_done) begin
                        launched <= 1'b0;
                        state <= S6_RESIDUAL;
                    end
                end

                S6_RESIDUAL: begin
                    if (!launched) begin
                        residual_start <= 1'b1;
                        launched <= 1'b1;
                    end
                    else if (residual_done) begin
                        launched <= 1'b0;
                        busy <= 1'b0;
                        filter_done <= 1'b1;
                        state <= S0_IDLE;
                    end
                end

                default: begin
                    state <= S0_IDLE;
                    launched <= 1'b0;
                    busy <= 1'b0;
                end

            endcase
        end
    end

endmodule
