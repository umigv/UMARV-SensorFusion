module UKF_corr_engine #(
    parameter bit ENABLE_SECOND_UPDATE = 1'b1,
    parameter bit ENABLE_THIRD_UPDATE = 1'b1,
    parameter bit ENABLE_FOURTH_UPDATE = 1'b1
) (
    input  logic clock, reset,
    input  meas_regfile_t corr_packet,
    input  PRED_PACKET pred_packet,
    input  pred_s_factor_t pred_s,
    input  meas_s_factor_t measurement_s,
    output logic convert_start,
    output meas_regfile_t convert_input,
    input  logic convert_done,
    input  corr_measurement_t convert_result,
    output logic busy, done, x_update_valid, failed,
    output UKF_OUT_PACKET ukf_output,
    output pred_s_factor_t corrected_s,
    output logic xy_update_valid, xyz_update_valid
);

    typedef enum logic [2:0] {
        IDLE, WAIT_PRED, START_CONVERSION, WAIT_CONVERSION,
        LOAD_INPUTS, RUN_CORR_MICROCODE, CORR_DONE
    } corr_states;

    corr_states state, next_state;
    corr_measurement_t measurement;
    PRED_PACKET prediction;
    pred_s_factor_t prediction_s;
    logic accept_measurement, output_valid;
    UKF_OUT_PACKET microcode_result;
    logic microcode_done, microcode_x_valid, microcode_xy_valid, microcode_xyz_valid, microcode_failed;

    // The matching prediction may arrive with the measurement or later.
    assign accept_measurement = state == IDLE &&
                                corr_packet.ukf_pulse && corr_packet.ukf_vn_valid;
    assign busy = state != IDLE;
    assign done = state == CORR_DONE;
    assign convert_start = state == START_CONVERSION;

    corr_microcode #(
        .ENABLE_SECOND_UPDATE(ENABLE_SECOND_UPDATE),
        .ENABLE_THIRD_UPDATE(ENABLE_THIRD_UPDATE),
        .ENABLE_FOURTH_UPDATE(ENABLE_FOURTH_UPDATE)
    ) correction_program (
        .clock(clock),
        .reset(reset),
        .start(state == LOAD_INPUTS),
        .measurement(measurement),
        .prediction(prediction),
        .prediction_s(prediction_s),
        .measurement_s(measurement_s),
        .done(microcode_done),
        .x_update_valid(microcode_x_valid),
        .xy_update_valid(microcode_xy_valid),
        .xyz_update_valid(microcode_xyz_valid),
        .failed(microcode_failed),
        .result(microcode_result),
        .corrected_s(corrected_s)
    );

    // Delay the valid strobe to align with the engine's done pulse.
    assign ukf_output = {microcode_result.ukf_x, microcode_result.ukf_y,
                         microcode_result.ukf_theta, microcode_result.ukf_omega,
                         microcode_result.ukf_wheelB, output_valid};

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (accept_measurement) begin
                    if (pred_packet.pred_valid)
                        next_state = START_CONVERSION;
                    else
                        next_state = WAIT_PRED;
                end
            end
            WAIT_PRED: begin
                if (pred_packet.pred_valid)
                    next_state = START_CONVERSION;
            end
            START_CONVERSION, WAIT_CONVERSION: begin
                if (convert_done)
                    next_state = LOAD_INPUTS;
                else
                    next_state = WAIT_CONVERSION;
            end
            LOAD_INPUTS: next_state = RUN_CORR_MICROCODE;
            RUN_CORR_MICROCODE: begin
                if (microcode_done)
                    next_state = CORR_DONE;
            end
            CORR_DONE: next_state = IDLE;
            default: next_state = IDLE;
        endcase
    end

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            convert_input <= '0;
            measurement <= '0;
            prediction <= '0;
            prediction_s <= '0;
            x_update_valid <= 1'b0;
            xy_update_valid <= 1'b0;
            xyz_update_valid <= 1'b0;
            failed <= 1'b0;
            output_valid <= 1'b0;
        end else begin
            state <= next_state;
            if (accept_measurement)
                convert_input <= corr_packet;
            // The factor belongs to the state on the same pred_valid beat.
            if ((accept_measurement || state == WAIT_PRED) && pred_packet.pred_valid) begin
                prediction <= pred_packet;
                prediction_s <= pred_s;
            end

            // The converter presents the normalized result with its done pulse.
            if ((state == START_CONVERSION || state == WAIT_CONVERSION) && convert_done)
                measurement <= convert_result;

            x_update_valid <= state == RUN_CORR_MICROCODE && microcode_x_valid;
            xy_update_valid <= state == RUN_CORR_MICROCODE && microcode_xy_valid;
            xyz_update_valid <= state == RUN_CORR_MICROCODE && microcode_xyz_valid;
            failed <= state == RUN_CORR_MICROCODE && microcode_failed;
            output_valid <= state == RUN_CORR_MICROCODE && microcode_result.ukf_valid;
        end
    end

endmodule
