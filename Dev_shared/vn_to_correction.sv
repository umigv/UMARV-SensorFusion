// Narrow raw FP64 lat/lon to FP32 and stage (lat, lon, yaw, -rate)
// directly into UKF_pred_sequencer's shared SPAD[0..3].
module vn_to_correction (
    input logic clock, reset, start,
    input ukf_defs::meas_regfile_t packet,
    output logic busy, done,
    output ukf_defs::ukf_spad_request_t spad_request
);
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

    typedef enum logic [1:0] {IDLE, WRITE_LON, WRITE_YAW, WRITE_RATE} state_t;
    state_t state;
    logic [63:0] narrow_in;
    logic [31:0] narrow_word, write_data;
    logic [1:0] write_idx;
    logic write_en;

    assign busy = state != IDLE;
    assign done = state == WRITE_RATE;
    assign narrow_in = (state == IDLE) ? packet.ukf_vn_lat_f64 : packet.ukf_vn_lon_f64;
    assign narrow_word = narrow(narrow_in);

    always_comb begin
        write_en = 1'b0;
        write_idx = 2'd0;
        write_data = narrow_word;
        case (state)
            IDLE: if (start) begin
                write_en = 1'b1;
                write_idx = 2'd0;
                write_data = narrow_word;
            end
            WRITE_LON: begin
                write_en = 1'b1;
                write_idx = 2'd1;
                write_data = narrow_word;
            end
            WRITE_YAW: begin
                write_en = 1'b1;
                write_idx = 2'd2;
                write_data = packet.ukf_vn_yaw_f32;
            end
            WRITE_RATE: begin
                write_en = 1'b1;
                write_idx = 2'd3;
                write_data = {~packet.ukf_vn_rate_f32[31], packet.ukf_vn_rate_f32[30:0]};
            end
        endcase
        spad_request = {write_en, write_en, 4'd0, 4'd0, {2'b0, write_idx}, write_data};
    end

    always_ff @(posedge clock or posedge reset) begin
        if (reset)
            state <= IDLE;
        else begin
            case (state)
                IDLE: if (start) state <= WRITE_LON;
                WRITE_LON: state <= WRITE_YAW;
                WRITE_YAW: state <= WRITE_RATE;
                WRITE_RATE: state <= IDLE;
            endcase
        end
    end
endmodule
