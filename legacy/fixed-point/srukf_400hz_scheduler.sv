`timescale 1ns/1ps

// The phase counter measures execution time. state_ts tracks only committed
// filter results. All sensor timestamps are unsigned microseconds.
module srukf_400hz_scheduler #(
    parameter integer CLK_HZ = 50_000_000
)(
    input logic clk, resetn,
    input logic init,
    input logic [63:0] init_ts,
    input logic sample_valid,
    input logic [63:0] sample_ts,
    output logic sample_pop,
    output logic sample_rejected,
    output logic predict_start, correct_start,
    output logic signed [31:0] dt,
    output logic [63:0] prediction_ts,
    input logic predict_done, correct_done,
    input logic phase_fault,
    output logic [63:0] state_ts,
    output logic state_valid,
    output logic deadline_missed,
    output logic fault
);
    localparam integer SLOT_CYCLES = CLK_HZ / 800;
    localparam integer COUNT_WIDTH = SLOT_CYCLES > 1 ? $clog2(SLOT_CYCLES) : 1;
    localparam logic [63:0] PERIOD_US = 2500;

    typedef enum logic [1:0] {IDLE, PREDICT, CORRECT} operation_t;
    operation_t operation;
    logic armed, prediction_slot, prediction_ready;
    logic [COUNT_WIDTH-1:0] counter;
    logic [63:0] next_correction_ts;
    wire phase_complete = (operation == PREDICT && predict_done) ||
                          (operation == CORRECT && correct_done);

    initial begin
        if (CLK_HZ < 800 || CLK_HZ % 800 != 0)
            $fatal(1, "Clock must support an integral number of cycles per slot");
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            operation <= IDLE;
            armed <= 0;
            prediction_slot <= 1;
            prediction_ready <= 0;
            counter <= 0;
            next_correction_ts <= 0;
            state_ts <= 0;
            prediction_ts <= 0;
            state_valid <= 0;
            dt <= 0;
            sample_pop <= 0;
            sample_rejected <= 0;
            predict_start <= 0;
            correct_start <= 0;
            deadline_missed <= 0;
            fault <= 0;
        end else begin
            sample_pop <= 0;
            sample_rejected <= 0;
            predict_start <= 0;
            correct_start <= 0;
            state_valid <= 0;
            if (!armed) begin
                // Initialization is the correction epoch. The first prediction
                // starts half a period later and targets the next sample.
                if (init) begin
                    armed <= 1;
                    state_ts <= init_ts;
                    next_correction_ts <= init_ts + PERIOD_US;
                end
            end else if (!fault) begin
                if (phase_fault) fault <= 1;
                else begin
                    if (phase_complete) begin
                        operation <= IDLE;
                        state_valid <= 1;
                        if (operation == PREDICT) begin
                            state_ts <= prediction_ts;
                            prediction_ready <= 1;
                        end
                    end
                    if (counter == COUNT_WIDTH'(SLOT_CYCLES-1)) begin
                        counter <= 0;
                        prediction_slot <= !prediction_slot;
                        if (operation != IDLE && !phase_complete) begin
                            deadline_missed <= 1;
                            fault <= 1;
                        end else if (prediction_slot) begin
                            prediction_ready <= 0;
                            if (next_correction_ts <= state_ts || next_correction_ts - state_ts != PERIOD_US)
                                fault <= 1;
                            else begin
                                prediction_ts <= next_correction_ts;
                                dt <= 32'sd5368709;
                                predict_start <= 1;
                                operation <= PREDICT;
                            end
                        end else begin
                            next_correction_ts <= next_correction_ts + PERIOD_US;
                            prediction_ready <= 0;
                            // Future samples remain queued. Stale samples can
                            // be drained between slots without starting work.
                            if (sample_valid && sample_ts == next_correction_ts &&
                                ((prediction_ready && state_ts == sample_ts) ||
                                 (operation == PREDICT && predict_done &&
                                  prediction_ts == sample_ts))) begin
                                sample_pop <= 1;
                                correct_start <= 1;
                                operation <= CORRECT;
                            end
                        end
                    end else begin
                        counter <= counter + 1'b1;
                        if (!sample_pop && sample_valid && sample_ts < next_correction_ts) begin
                            sample_pop <= 1;
                            sample_rejected <= 1;
                        end
                    end
                end
            end
        end
    end
endmodule
