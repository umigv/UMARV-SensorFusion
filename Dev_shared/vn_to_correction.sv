// Adapt the raw VN packet to the existing FP32 east/north converter,
// sharing UKF_pred_sequencer's SPAD[0..3] and a single FP64->FP32 narrower.
module vn_to_correction #(
    parameter logic [31:0] ORIGIN_LATITUDE = 32'b0,
    parameter logic [31:0] ORIGIN_LONGITUDE = 32'b0,
    parameter logic [31:0] EARTH_RADIUS = 32'h4ac26d70
) (
    input logic clock, reset, start,
    input ukf_defs::meas_regfile_t packet,
    output logic busy, done,
    output ukf_defs::corr_measurement_t measurement,
    output ukf_defs::ukf_mac_request_t mac_request,
    input ukf_defs::ukf_mac_response_t mac_response,
    output ukf_defs::ukf_spad_request_t spad_request,
    input ukf_defs::ukf_spad_response_t spad_response
);
    localparam logic [31:0] ONE = 32'h3f800000;
    localparam logic [31:0] DEG_TO_RAD = 32'h3c8efa35;
    localparam logic [31:0] HALF_PI = 32'h3fc90fdb;

    // Round the double input to nearest FP32, with ties to even.
    function automatic logic [31:0] narrow(input logic [63:0] word);
        logic [10:0] exponent64;
        logic [7:0] exponent32;
        logic [52:0] significand;
        logic [53:0] rounded;
        integer unbiased, shift;
        exponent64 = word[62:52];
        unbiased = {21'b0, exponent64} - 1023;
        significand = {1'b1, word[51:0]};
        if (exponent64 == 11'h7ff)
            narrow = {word[63], 8'hff, (|word[51:0] ? 23'h400000 : 23'b0)};
        else if (unbiased > 127)
            narrow = {word[63], 8'hff, 23'b0};
        else if (unbiased < -150)
            narrow = {word[63], 31'b0};
        else begin
        shift = unbiased < -126 ? -unbiased - 97 : 29;
        rounded = {1'b0, significand} >> shift;
        if (significand[shift-1] &&
            (rounded[0] || (|(significand << (54-shift)))))
            rounded = rounded + 1'b1;
        if (unbiased < -126)
            narrow = {word[63], 7'b0, rounded[23:0]};
        else begin
        exponent32 = 8'(unbiased + 127 + rounded[24]);
        narrow = {word[63], exponent32, rounded[22:0]};
        end
        end
    endfunction

    typedef enum logic [2:0] {
        IDLE, START_XY, WAIT_XY, SEED_HEADING, WAIT_SEED, ISSUE_HEADING, WAIT_HEADING
    } state_t;
    state_t state;
    ukf_defs::ukf_mac_request_t xy_mac_request;
    logic [6:0] address;
    logic write_enable, xy_valid;
    logic [31:0] read_data, write_data, x, y, narrow_word;
    logic [63:0] narrow_in;

    assign busy = state != IDLE;
    assign done = state == WAIT_HEADING && mac_response.valid;
    assign measurement = {x, y, mac_response.result,
                          ~packet.ukf_vn_rate_f32[31], packet.ukf_vn_rate_f32[30:0]};

    assign narrow_in = (state == IDLE) ? packet.ukf_vn_lat_f64 : packet.ukf_vn_lon_f64;
    assign narrow_word = narrow(narrow_in);

    always_comb begin
        spad_request = '0;
        if (state == IDLE && start) begin
            spad_request = {1'b1, 1'b1, 4'd0, 4'd0, 4'd0, narrow_word};
        end else if (state == START_XY) begin
            spad_request = {1'b1, 1'b0, 4'd0, 4'd0, 4'd0, 32'b0};
        end else if (state == WAIT_XY) begin
            spad_request = {
                1'b1,
                write_enable && address >= 7'd21 && address <= 7'd24,
                address[3:0] - 4'd5,
                4'd0,
                address[3:0] - 4'd5,
                write_data
            };
        end
    end

    always_comb begin
        case (address)
            7'd70: read_data = ORIGIN_LONGITUDE;
            7'd71: read_data = ORIGIN_LATITUDE;
            7'd96: read_data = ONE;
            7'd107: read_data = DEG_TO_RAD;
            7'd108: read_data = EARTH_RADIUS;
            7'd109: read_data = 32'hbf000000;
            7'd110: read_data = 32'h3d2aaaab;
            7'd111: read_data = 32'hbab60b61;
            7'd112: read_data = 32'h37d00d01;
            7'd113: read_data = 32'hb493f27e;
            7'd114: read_data = 32'h310f76c7;
            default: read_data = address >= 7'd21 && address <= 7'd24 ? spad_response.a : '0;
        endcase
    end

    lat_to_xy coordinates (
        .clk(clock), .rst_n(!reset), .start(state == START_XY),
        .latitude(spad_response.a), .longitude(narrow_word),
        .mem_rdata(read_data), .mem_addr(address), .mem_we(write_enable), .mem_wdata(write_data),
        .mac_start(xy_mac_request.start), .mac_clear(xy_mac_request.clear),
        .mac_last(xy_mac_request.last), .mac_a(xy_mac_request.a), .mac_b(xy_mac_request.b),
        .mac_busy(mac_response.busy), .mac_result_valid(mac_response.valid),
        .mac_result(mac_response.result), .busy(), .xy_valid(xy_valid), .x_f32(x), .y_f32(y)
    );

    // ENU heading is counterclockwise from east. VN yaw is clockwise from north.
    always_comb begin
        mac_request = xy_mac_request;
        case (state)
            SEED_HEADING: mac_request = {!mac_response.busy, 1'b1, 1'b1, HALF_PI, ONE};
            ISSUE_HEADING: mac_request = {!mac_response.busy, 1'b0, 1'b1,
                                          packet.ukf_vn_yaw_f32, {1'b1, DEG_TO_RAD[30:0]}};
            default: begin end
        endcase
    end

    always_ff @(posedge clock or posedge reset) begin
        if (reset)
            state <= IDLE;
        else begin
            case (state)
                IDLE: if (start) state <= START_XY;
                START_XY: state <= WAIT_XY;
                WAIT_XY: if (xy_valid) state <= SEED_HEADING;
                SEED_HEADING: if (mac_request.start) state <= WAIT_SEED;
                WAIT_SEED: if (mac_response.valid) state <= ISSUE_HEADING;
                ISSUE_HEADING: if (mac_request.start) state <= WAIT_HEADING;
                WAIT_HEADING: if (done) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
