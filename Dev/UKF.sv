module UKF #(
    parameter logic [31:0] ORIGIN_LATITUDE = 32'b0,
    parameter logic [31:0] ORIGIN_LONGITUDE = 32'b0,
    parameter logic [31:0] EARTH_RADIUS = 32'h4ac26d70,
    parameter logic [31:0] WHEEL_VELOCITY_SCALE = 32'b0,
    parameter logic [160:0] INITIAL_STATE = '0,
    parameter logic [479:0] INITIAL_S = '0,
    parameter bit EXTERNAL_PREDICTION = 1'b0,
    parameter logic [31:0] PRED_DT = 32'h3b23d70a,
    parameter logic [31:0] SIGMA_SCALE = 32'h3f8f1bbd,
    parameter logic [31:0] W_NONCENTRAL = 32'h3ecccccd,
    parameter logic [31:0] SQRT_W_NONCENTRAL = 32'h3f21e89b,
    parameter logic [31:0] W_MEAN_CENTRAL = 32'hc0400000,
    parameter logic [31:0] SQRT_ABS_W_CENTRAL = 32'h3f000000,
    parameter bit CENTRAL_DOWNDATE = 1'b1,
    parameter logic [479:0] PROCESS_S = {
        32'h3c23d70a, 32'b0, 32'h3c23d70a, 64'b0, 32'h3c23d70a,
        96'b0, 32'h3c23d70a, 128'b0, 32'h3c23d70a
    }
) (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               ukf_beat,
    input  logic               ukf_pulse,
    input  logic        [31:0] ukf_beat_ts,
    input  logic               ukf_vn_valid,
    input  logic        [31:0] ukf_vn_yaw_f32,
    input  logic        [31:0] ukf_vn_rate_f32,
    input  logic        [63:0] ukf_vn_lat_f64,
    input  logic        [63:0] ukf_vn_lon_f64,
    input  logic               ukf_dw_valid,
    input  logic signed [27:0] ukf_dw_delta,

    input  meas_s_factor_t     measurement_s,

    // Prediction execution backend. Return state and factor on the same done beat.
    output logic               prediction_start,
    output UKF_OUT_PACKET      prediction_prior,
    output pred_s_factor_t     prediction_prior_s,
    output logic        [31:0] prediction_ts, prediction_wheel_velocity,
    output logic               prediction_wheel_valid,
    input  logic               prediction_done,
    input  PRED_PACKET         prediction_result,
    input  pred_s_factor_t     prediction_result_s,

    output logic               ukf_busy,
    output logic               ukf_pred_done,
    output logic               ukf_valid,
    output logic               ukf_failed,
    output logic               ukf_reinit_required,
    output logic               ukf_vn_overflow,
    output logic        [31:0] ukf_x,
    output logic        [31:0] ukf_y,
    output logic        [31:0] ukf_theta,
    output logic        [31:0] ukf_omega,
    output logic        [31:0] ukf_wheelB,
    output pred_s_factor_t     corrected_s,
    input  ukf_mac_request_t   prediction_mac_request,
    output ukf_mac_response_t  prediction_mac_response,
    output logic               prediction_mac_grant,
    input  ukf_divsqrt_request_t  prediction_div_request,
    output ukf_divsqrt_response_t prediction_div_response,
    output logic                  prediction_div_grant
);
    localparam logic [6:0] WHEEL_SCALE_ADDR = 7'd95;

    logic reset, prediction_busy, backend_active, correction_busy, prediction_failed, correction_failed;
    logic wheel_mac_owner, converter_busy, convert_request, convert_pending, convert_start, convert_done;
    logic [6:0] wheel_address;
    logic [31:0] wheel_read_data, wheel_write_data;
    logic wheel_write, wheel_clear;
    meas_regfile_t time_pack, incoming_packet, vn_pending_packet, correction_packet, convert_input;
    meas_s_factor_t active_measurement_s, noise_snapshot;
    logic load_shared_noise, load_shared_measurement, correction_update_failed;
    ukf_spad_request_t correction_spad_request, converter_scratch_request, shared_spad_request;
    ukf_spad_response_t correction_spad_response;
    logic vn_pending, vn_arrival, vn_dispatch, vn_store, late_dispatch, late_conversion;
    corr_measurement_t converted;
    PRED_PACKET pred_packet;
    UKF_OUT_PACKET result_packet, latest_state;
    pred_s_factor_t pred_s, latest_s, correction_s, shared_factor;
    logic [159:0] shared_state;
    ukf_mac_request_t wheel_mac_request, converter_mac_request, sensor_mac_request;
    ukf_mac_response_t sensor_mac_response;
    ukf_divsqrt_response_t shared_div_response;
    ukf_mac_request_t backend_mac_request;
    ukf_divsqrt_request_t backend_div_request;
    logic backend_done;
    PRED_PACKET backend_result;
    pred_s_factor_t backend_result_s;

    generate
        if (EXTERNAL_PREDICTION) begin : external_backend
            assign backend_done = prediction_done;
            assign backend_result = prediction_result;
            assign backend_result_s = prediction_result_s;
            assign backend_mac_request = prediction_mac_request;
            assign backend_div_request = prediction_div_request;
            assign correction_spad_response = '0;
            assign shared_state = '0;
            assign shared_factor = '0;
            assign active_measurement_s = noise_snapshot;
            assign ukf_reinit_required = 1'b0;
        end else begin : internal_backend
            // A failed in-place update has no rollback image. Reset is
            // the only recovery event and reloads the configured prior.
            always_ff @(posedge clk or negedge rst_n)
                if (!rst_n)
                    ukf_reinit_required <= 1'b0;
                else if (correction_update_failed)
                    ukf_reinit_required <= 1'b1;

            assign latest_state = {result_packet[160:1],
                                   !ukf_reinit_required && !correction_update_failed &&
                                   !correction_spad_request.active};
            assign latest_s = correction_s;
            // One global flop bank. Address 5 is the held wheel input.
            // The controller hands the read/write ports from prediction
            // to conversion and then to correction.
            localparam int SCRATCH_BASE = 21, NOISE_BASE = 37;
            wire [31:0] registers [0:51];
            wire [31:0] prior_bank [0:20];
            wire [31:0] scratch_bank [0:15];
            wire [31:0] noise_bank [0:14];
            logic [6:0] pred_read_a, pred_read_b, pred_write_addr;
            logic [31:0] pred_write_data, read_data_a, read_data_b;
            logic pred_write, pred_success;
            wire client_active = shared_spad_request.active;
            wire [6:0] addr_a = client_active ? shared_spad_request.read_addr_a : pred_read_a;
            wire [6:0] addr_b = client_active ? shared_spad_request.read_addr_b : pred_read_b;
            wire [6:0] write_addr = client_active ? shared_spad_request.write_addr : pred_write_addr;
            wire [31:0] write_data = client_active ? shared_spad_request.write_data : pred_write_data;
            wire write_enable = client_active ? shared_spad_request.write_enable : pred_write;
            wire [319:0] staged_noise_words = noise_snapshot;
            wire [479:0] staged_noise = {160'b0, staged_noise_words};
            wire [479:0] predicted_factor = pred_s;
            wire [479:0] retained_factor, working_factor;
            wire [127:0] converted_words = converted;
            wire [511:0] measurement_words = {converted_words, 384'b0};
            wire main_write = correction_spad_request.active && write_enable;
            wire [31:0] main_write_data = write_data & {32{!pred_packet.pred_valid}};

            assign registers[5] = prediction_wheel_velocity;
            for (genvar i = 0; i < 21; i++)
                assign prior_bank[i] = registers[i];
            assign shared_state = {registers[0], registers[1], registers[2],
                                   registers[3], registers[4]};
            assign shared_factor = retained_factor;
            assign backend_result = {registers[28], registers[29], registers[30],
                                     registers[31], registers[32], backend_done && pred_success};
            assign backend_result_s = working_factor;
            assign active_measurement_s = working_factor[319:0];

            for (genvar i = 0; i < 5; i++) begin : state_words
                logic [31:0] word;
                assign registers[i] = word;
                always_ff @(posedge clk or negedge rst_n)
                    if (!rst_n)
                        word <= INITIAL_STATE[160-32*i -: 32];
                    else if (pred_packet.pred_valid || (main_write && write_addr == 7'(i)))
                        word <= (pred_packet[160-32*i -: 32] & {32{pred_packet.pred_valid}}) |
                                main_write_data;
            end
            for (genvar i = 0; i < 15; i++) begin : factor_words
                logic [31:0] word;
                assign registers[6+i] = word;
                assign retained_factor[32*i +: 32] = word;
                always_ff @(posedge clk or negedge rst_n)
                    if (!rst_n)
                        word <= INITIAL_S[32*i +: 32];
                    else if (pred_packet.pred_valid || (main_write && write_addr == 7'(6+i)))
                        word <= (predicted_factor[32*i +: 32] & {32{pred_packet.pred_valid}}) |
                                main_write_data;
            end
            for (genvar i = 0; i < 16; i++) begin : scratch_words
                logic [31:0] word;
                assign registers[SCRATCH_BASE+i] = word;
                assign scratch_bank[i] = word;
                always_ff @(posedge clk)
                    if (rst_n) begin
                        if (i < 4 && load_shared_measurement)
                            word <= measurement_words[511-32*i -: 32];
                        else if (write_enable && write_addr == 7'(SCRATCH_BASE+i))
                            word <= write_data;
                    end
            end
            for (genvar i = 0; i < 15; i++) begin : noise_words
                logic [31:0] word;
                assign registers[NOISE_BASE+i] = word;
                assign noise_bank[i] = word;
                assign working_factor[32*i +: 32] = word;
                always_ff @(posedge clk)
                    if (rst_n) begin
                        if (prediction_start)
                            word <= PROCESS_S[32*i +: 32];
                        else if (i < 10 && load_shared_noise)
                            word <= staged_noise[32*i +: 32];
                        else if (!client_active && write_enable && write_addr == 7'(68+i))
                            word <= pred_write_data;
                    end
            end

            wire [3:0] scratch_index_a = addr_a[3:0] - 4'd5;
            wire [3:0] scratch_index_b = addr_b[3:0] - 4'd5;
            wire [3:0] noise_index_a = addr_a[3:0] - (client_active ? PROM_SR_BASE[3:0] : 4'd4);
            wire [3:0] noise_index_b = addr_b[3:0] - (client_active ? PROM_SR_BASE[3:0] : 4'd4);

            function automatic logic [31:0] read_register(
                input logic [6:0] addr,
                input logic correction_port,
                input logic [31:0] prior_word, scratch_word, noise_word
            );
                read_register = (prior_word & {32{addr < 21}}) |
                    (scratch_word & {32{addr >= 21 && addr <= 36}}) |
                    (noise_word & {32{
                        correction_port ? addr >= PROM_SR_BASE && addr < PROM_SR_BASE + 7'd10 :
                                          addr >= 68 && addr < 83}}) |
                    prom_math_read(addr);
            endfunction

            assign read_data_a = read_register(addr_a, client_active, prior_bank[addr_a[4:0]],
                                               scratch_bank[scratch_index_a], noise_bank[noise_index_a]);
            assign read_data_b = read_register(addr_b, client_active, prior_bank[addr_b[4:0]],
                                               scratch_bank[scratch_index_b], noise_bank[noise_index_b]);
            assign correction_spad_response = {read_data_a, read_data_b, registers[23][31]};

            UKF_pred_sequencer #(
                .DT(PRED_DT), .SIGMA_SCALE(SIGMA_SCALE), .W_NONCENTRAL(W_NONCENTRAL),
                .SQRT_W_NONCENTRAL(SQRT_W_NONCENTRAL), .W_MEAN_CENTRAL(W_MEAN_CENTRAL),
                .SQRT_ABS_W_CENTRAL(SQRT_ABS_W_CENTRAL), .CENTRAL_DOWNDATE(CENTRAL_DOWNDATE)
            ) sequencer (
                .clock(clk), .reset(reset), .start(prediction_start),
                .wheel_valid(prediction_wheel_valid),
                .read_addr_a(pred_read_a), .read_addr_b(pred_read_b),
                .read_data_a(read_data_a), .read_data_b(read_data_b),
                .wrap_word(registers[int'(PRED_ANGLE_ARG)]),
                .write_enable(pred_write), .write_addr(pred_write_addr), .write_data(pred_write_data),
                .mac_grant(prediction_mac_grant), .div_grant(prediction_div_grant),
                .mac_response(prediction_mac_response), .div_response(prediction_div_response),
                .mac_request(backend_mac_request), .div_request(backend_div_request),
                .done(backend_done), .success(pred_success)
            );
        end
    endgenerate

    assign reset = ~rst_n;
    assign ukf_busy = prediction_busy || correction_busy || converter_busy || convert_pending || ukf_valid;
    assign vn_arrival = rst_n && !ukf_reinit_required && ukf_pulse && ukf_vn_valid;
    assign late_dispatch = prediction_busy && !correction_busy;
    assign vn_dispatch = rst_n && !ukf_reinit_required && (vn_pending || vn_arrival) &&
                         (late_dispatch || !ukf_busy);
    assign vn_store = vn_arrival && (!vn_dispatch || vn_pending) &&
                      (!vn_pending || vn_dispatch);
    assign incoming_packet = {ukf_pulse, ukf_beat_ts, ukf_vn_valid,
                              ukf_vn_yaw_f32, ukf_vn_rate_f32, ukf_vn_lat_f64,
                              ukf_vn_lon_f64, ukf_dw_valid, ukf_dw_delta};
    always_comb begin
        time_pack = incoming_packet;
        time_pack.ukf_pulse = rst_n && !ukf_reinit_required && !ukf_busy &&
                             ((ukf_pulse && ukf_dw_valid) || vn_dispatch);
        time_pack.ukf_dw_valid = ukf_pulse && ukf_dw_valid;
        time_pack.ukf_vn_valid = vn_dispatch;
        if (!time_pack.ukf_dw_valid && vn_pending)
            time_pack.ukf_beat_ts = vn_pending_packet.ukf_beat_ts;
        correction_packet = vn_pending ? vn_pending_packet : incoming_packet;
        correction_packet.ukf_pulse = vn_dispatch;
        correction_packet.ukf_vn_valid = vn_dispatch;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vn_pending <= 1'b0;
            ukf_vn_overflow <= 1'b0;
            late_conversion <= 1'b0;
        end else begin
            ukf_vn_overflow <= vn_arrival && vn_pending && !vn_dispatch;
            if (correction_update_failed && !EXTERNAL_PREDICTION)
                vn_pending <= 1'b0;
            else if (vn_store)
                vn_pending <= 1'b1;
            else if (vn_dispatch)
                vn_pending <= 1'b0;
            if (vn_dispatch)
                late_conversion <= late_dispatch;
        end
    end

    // Both noise factors can be live during prediction. A queued dispatch
    // swaps ownership instead of copying 320 bits. On a simultaneous refill
    // the incoming noise goes into the old active bank.
    meas_s_factor_t noise_bank_0, noise_bank_1;
    logic active_noise_bank;
    wire take_direct_noise = vn_dispatch && !vn_pending;
    wire store_noise = vn_store || take_direct_noise;
    wire store_noise_bank = vn_dispatch ? active_noise_bank : !active_noise_bank;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n)
            active_noise_bank <= 1'b0;
        else if (vn_dispatch && vn_pending)
            active_noise_bank <= !active_noise_bank;

    always_ff @(posedge clk) begin
        if (store_noise && !store_noise_bank)
            noise_bank_0 <= measurement_s;
        if (store_noise && store_noise_bank)
            noise_bank_1 <= measurement_s;
    end
    assign noise_snapshot = active_noise_bank ? noise_bank_1 : noise_bank_0;
    wire [319:0] selected_noise = vn_pending ? (active_noise_bank ? noise_bank_0 : noise_bank_1) : measurement_s;

    // The queued packet is read only after its corresponding valid event.
    always_ff @(posedge clk) begin
        if (vn_store) begin
            vn_pending_packet <= incoming_packet;
        end
    end

    assign wheel_read_data = wheel_address == WHEEL_SCALE_ADDR ? WHEEL_VELOCITY_SCALE : '0;
    // The converter and prediction never interleave MAC accumulator use.
    // Correction starts after both complete.
    assign prediction_mac_grant = backend_active && !wheel_mac_owner &&
                                  !converter_busy && !convert_start;
    assign prediction_div_grant = backend_active;
    assign sensor_mac_request = wheel_mac_owner ? wheel_mac_request :
                                prediction_mac_grant ? backend_mac_request : converter_mac_request;
    assign prediction_mac_response = prediction_mac_grant ? sensor_mac_response : '0;
    assign prediction_div_response = prediction_div_grant ? shared_div_response : '0;
    // The built-in path defers conversion until prediction releases SPAD.
    // An external backend retains its early-conversion handshake.
    assign convert_start = (convert_request || convert_pending) && !wheel_mac_owner &&
                           (EXTERNAL_PREDICTION ? (!late_conversion || !backend_active) : !prediction_busy) &&
                           !converter_busy;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            convert_pending <= 1'b0;
        else if (!EXTERNAL_PREDICTION && prediction_failed)
            convert_pending <= 1'b0;
        else
            convert_pending <= (convert_request || convert_pending) && !convert_start;
    end

    // The wheel value is held in global register 5 for prediction.
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n)
            prediction_wheel_velocity <= '0;
        else if (wheel_write)
            prediction_wheel_velocity <= wheel_write_data;
        else if (wheel_clear)
            prediction_wheel_velocity <= '0;

    UKF_pred_engine prediction (
        .clock(clk), .reset(reset),
        .corr_packet(latest_state), .corr_s(latest_s), .timestamp_pack(time_pack),
        .backend_start(prediction_start), .backend_active(backend_active),
        .backend_prior(prediction_prior),
        .backend_prior_s(prediction_prior_s), .backend_ts(prediction_ts),
        .clear_wheel_velocity(wheel_clear),
        .backend_wheel_valid(prediction_wheel_valid),
        .backend_done(backend_done), .backend_prediction(backend_result),
        .backend_prediction_s(backend_result_s),
        .velocity_scale_addr(WHEEL_SCALE_ADDR), .mem_rdata(wheel_read_data),
        .mem_addr(wheel_address), .mem_we(wheel_write), .mem_wdata(wheel_write_data),
        .mac_start(wheel_mac_request.start), .mac_clear(wheel_mac_request.clear),
        .mac_last(wheel_mac_request.last), .mac_a(wheel_mac_request.a), .mac_b(wheel_mac_request.b),
        .mac_busy(sensor_mac_response.busy), .mac_result_valid(sensor_mac_response.valid),
        .mac_result(sensor_mac_response.result), .wheel_mac_owner(wheel_mac_owner),
        .busy(prediction_busy), .done(ukf_pred_done), .failed(prediction_failed),
        .pred_pack(pred_packet), .pred_s(pred_s)
    );

    // Conversion starts after prediction commits and finishes before
    // correction starts. Its four scratch words become correction residuals.
    assign shared_spad_request = converter_scratch_request.active ?
                                 converter_scratch_request : correction_spad_request;

    vn_to_correction #(
        .SHARED_SCRATCH(!EXTERNAL_PREDICTION),
        .ORIGIN_LATITUDE(ORIGIN_LATITUDE), .ORIGIN_LONGITUDE(ORIGIN_LONGITUDE),
        .EARTH_RADIUS(EARTH_RADIUS)
    ) converter (
        .clock(clk), .reset(reset), .start(convert_start), .packet(convert_input),
        .busy(converter_busy), .done(convert_done), .measurement(converted),
        .mac_request(converter_mac_request), .mac_response(sensor_mac_response),
        .scratch_request(converter_scratch_request), .scratch_read(correction_spad_response.a)
    );

    UKF_corr_engine #(
        .SHARE_MAC(1'b1), .REPORT_PRED_FAILURE(1'b1),
        .HELD_PREDICTION(!EXTERNAL_PREDICTION), .SHARED_STORAGE(!EXTERNAL_PREDICTION)
    ) correction (
        .clock(clk), .reset(reset), .corr_packet(correction_packet),
        .pred_packet(pred_packet), .pred_s(pred_s), .measurement_s(active_measurement_s),
        .load_shared_noise(load_shared_noise), .load_shared_measurement(load_shared_measurement),
        .update_failed(correction_update_failed),
        .shared_state(shared_state), .shared_factor(shared_factor),
        .spad_request(correction_spad_request), .spad_response(correction_spad_response),
        .convert_start(convert_request), .convert_input(convert_input),
        .convert_done(convert_done), .convert_result(converted),
        .busy(correction_busy), .done(), .failed(correction_failed),
        .x_update_valid(), .xy_update_valid(), .xyz_update_valid(),
        .ukf_output(result_packet), .corrected_s(correction_s),
        .sensor_mac_owner(wheel_mac_owner || converter_busy || prediction_mac_grant),
        .prediction_div_owner(prediction_div_grant),
        .prediction_div_request(backend_div_request),
        .prediction_div_response(shared_div_response),
        .sensor_mac_request(sensor_mac_request), .sensor_mac_response(sensor_mac_response),
        .prediction_failed(prediction_failed)
    );

    // On the built-in path these wires reflect the live bank. Sample
    // correction results on ukf_valid; the bank can change while busy.
    assign {ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_wheelB} =
        {latest_state.ukf_x, latest_state.ukf_y, latest_state.ukf_theta,
         latest_state.ukf_omega, latest_state.ukf_wheelB};
    assign corrected_s = latest_s;
    assign ukf_failed = correction_failed || (prediction_failed && !correction_busy);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ukf_valid <= 1'b0;
        else
            ukf_valid <= result_packet.ukf_valid && !ukf_reinit_required;
    end

    // External backends retain the legacy publish/rollback contract.
    generate
        if (EXTERNAL_PREDICTION) begin : external_retained
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    latest_state <= INITIAL_STATE;
                    latest_s <= INITIAL_S;
                end else if (result_packet.ukf_valid) begin
                    latest_state <= result_packet;
                    latest_s <= correction_s;
                end else if (pred_packet.pred_valid) begin
                    latest_state <= {pred_packet[160:1], 1'b1};
                    latest_s <= pred_s;
                end
            end
        end
    endgenerate
endmodule
