// Lidar stage 4: how far the robot has turned and moved since the current sweep started.
//
// For every firing of the lidar this block produces what the transform needs to put that
// firing's points into the robot's frame at the start of the sweep:
//
//   sin0..3, cos0..3   sine and cosine of the firing's corrected angle
//                          a' = lidar azimuth + yaw_offset + laser offset - turned
//                      a' is measured the way the lidar measures azimuth: clockwise from straight
//                      ahead. Index 0..3 is the laser's horizontal offset class (+4.2, +1.4, -1.4,
//                      -4.2 degrees) when four_offsets = 1 (VLP-32C). Otherwise the offset is zero
//                      and all four indexes hold the same values.
//   shift_x, shift_y   how far the robot has moved since the sweep started, in output counts,
//                      in the sweep-start frame: x forward, y to the left.
//
// With ground = distance * cos(elevation), the transform is then
//       x_forward = ground * cos(a') + shift_x
//       y_left    = -ground * sin(a') + shift_y
//
// Three running totals are kept, cleared on the first firing of each sweep and advanced once per
// firing by a fixed time step (the lidar's own firing period, not the time packets arrive):
//       turned  += omega * T                              radians, 29 fractional bits
//       moved_x += v * T * (1 - turned^2 / 2)             output counts, 16 fractional bits
//       moved_y += v * T * turned
// The last two use the small-angle forms of cos and sin; a sweep turns the robot a fraction of
// a radian at most.
//
// omega and v arrive as IEEE single-precision values and are converted to fixed point here,
// clamped to +/-8 (rad/s, m/s). omega is positive counter-clockwise, v positive forward.
//
// One multiplier is shared by everything; a firing takes about a dozen clocks plus one CORDIC
// run per offset class. firing_start may arrive while a state update is being processed; it is
// remembered and served next. overrun pulses if a firing arrives before the previous one was served.
//
// Written in the plain SystemVerilog subset the Tiny Tapeout tools accept.
module lidar_motion (
    input  logic               clk,
    input  logic               rst,                 // synchronous, active high

    // Settings
    input  logic               corr_rot_en,         // 0: leave 'turned' out of the angle
    input  logic               corr_shift_en,       // 0: shift_x and shift_y stay zero
    input  logic               scale_4mm,           // output step: 0 = 2 mm per count, 1 = 4 mm
    input  logic               four_offsets,        // 1: VLP-32C, four horizontal laser offsets
    input  logic        [15:0] firing_period_ns,    // time for one firing of all lasers
    input  logic signed [15:0] yaw_offset,          // lidar mounting angle, 0.01 degree, added to the azimuth

    // Filter state (IEEE single precision), held steady between state_valid pulses
    input  logic               state_valid,
    input  logic        [31:0] state_omega,         // rad/s
    input  logic        [31:0] state_v,             // m/s

    // One pulse per firing, with that firing's block azimuth
    input  logic               firing_start,
    input  logic               firing_second,       // second firing of a data block: azimuth is interpolated
    input  logic               sweep_start,         // first firing of a sweep: totals are cleared
    input  logic        [15:0] azimuth,             // 0.01 degree, 0..35999

    // CORDIC (lidar_cordic)
    output logic               cordic_start,
    output logic signed [17:0] cordic_theta,        // radians, 15 fractional bits, within +/- pi
    input  logic               cordic_done,
    input  logic signed [17:0] cordic_sin,
    input  logic signed [17:0] cordic_cos,

    // Results for the firing, steady from fire_done until the next fire_done
    output logic               fire_done,           // one clock
    output logic               fire_sweep_start,    // this firing started a sweep
    output logic signed [15:0] sin0, cos0, sin1, cos1, sin2, cos2, sin3, cos3,   // 15 fractional bits
    output logic signed [15:0] shift_x, shift_y,

    output logic               state_seen,          // a filter state has been received since reset
    output logic               overrun              // one clock: a firing was missed
);

    // ---- IEEE single precision to signed fixed point with 12 fractional bits, clamped --------
    function automatic signed [15:0] fp_to_q12;
        input [31:0] f;
        reg [7:0]  e;
        reg [24:0] man, rounded;
        reg [4:0]  sh;
        reg [15:0] mag;
        begin
            e   = f[30:23];
            man = {2'b01, f[22:0]};
            if (e == 8'd255)      mag = (f[22:0] != 23'd0) ? 16'd0 : 16'd32767;    // not-a-number: 0, infinity: clamp
            else if (e >= 8'd130) mag = 16'd32767;                                 // 8 or more: clamp
            else if (e <= 8'd113) mag = 16'd0;                                     // under half a step
            else begin
                sh      = 8'd138 - e;                                              // 9..24
                rounded = (man + (25'd1 << (sh - 5'd1))) >> sh;
                mag     = (rounded > 25'd32767) ? 16'd32767 : rounded[15:0];
            end
            fp_to_q12 = f[31] ? -$signed(mag) : $signed(mag);
        end
    endfunction

    // ---- constants ------------------------------------------------------------------------------
    localparam signed [16:0] K_TURN    = 17'sd17592;    // ns -> radians(Q29) per rad/s(Q12):  x 0.53687
    localparam signed [16:0] K_MOVE    = 17'sd17180;    // ns -> counts(Q16) per m/s(Q12), 2 mm: x 0.52429
    localparam signed [16:0] K_DEG2RAD = 17'sd46851;    // 0.01 degree -> radians(Q15), with 13 fractional bits
    localparam signed [18:0] PI_Q15    = 19'sd102944;
    localparam signed [18:0] TWO_PI_Q15 = 19'sd205887;
    localparam signed [18:0] OFF_4P2   = 19'sd2402;     // 4.2 degrees
    localparam signed [18:0] OFF_1P4   = 19'sd801;      // 1.4 degrees

    localparam [4:0] IDLE = 5'd0,
                     U_KT = 5'd1,  U_KT2 = 5'd2,  U_KM = 5'd3,  U_KM2 = 5'd4,
                     U_DT = 5'd5,  U_DT2 = 5'd6,  U_DS = 5'd7,  U_DS2 = 5'd8,
                     F_ACC = 5'd9, F_T2 = 5'd10,  F_T2B = 5'd11, F_COS = 5'd12,
                     F_MX = 5'd13, F_MX2 = 5'd14, F_MY = 5'd15, F_MY2 = 5'd16,
                     F_AZ = 5'd17, F_AZ2 = 5'd18, F_CORD = 5'd19, F_WAIT = 5'd20, F_OUT = 5'd21;
    logic [4:0] st;

    // ---- shared multiplier: operands registered, product registered ----------------------------
    logic signed [16:0] ma, mb;
    logic signed [33:0] mp;
    always_ff @(posedge clk) mp <= ma * mb;

    // ---- per-state-update values ---------------------------------------------------------------
    logic signed [15:0] omega_q, v_q;       // rad/s and m/s, 12 fractional bits
    logic signed [16:0] kt, km;
    logic signed [23:0] dturn;              // added to 'turned' per firing
    logic signed [16:0] ds;                 // output counts moved per firing, 16 fractional bits

    // ---- running totals ------------------------------------------------------------------------
    logic signed [31:0] turned;             // radians, 29 fractional bits
    logic signed [31:0] moved_x, moved_y;   // output counts, 16 fractional bits
    wire  signed [17:0] t15 = turned[31:14];                               // radians, 15 fractional bits
    wire  signed [16:0] t17 = (t15 >  18'sd65535) ?  17'sd65535 :
                              (t15 < -18'sd65535) ? -17'sd65535 : t15[16:0];   // clamp to +/-2 rad

    // ---- a firing waiting to be served, and the one being served --------------------------------
    logic        fire_req, upd_req;
    logic [15:0] f_az;
    logic        f_second, f_sweep;
    logic [15:0] w_az;
    logic        w_second, w_sweep;
    logic [15:0] az_last, az_step;          // previous block's azimuth and the step between blocks

    // azimuth of this firing, with interpolation, mounting offset, wrapped to -18000..17999
    wire [15:0] step_raw   = w_az - az_last;
    wire [15:0] step_now   = (w_az >= az_last) ? step_raw : (step_raw + 16'd36000);
    wire [16:0] az_eff     = w_second ? ({1'b0, w_az} + {2'b00, az_step[15:1]}) : {1'b0, w_az};
    wire signed [18:0] az_sum = $signed({2'b00, az_eff}) + $signed({{3{yaw_offset[15]}}, yaw_offset});
    wire signed [18:0] az_w1  = (az_sum >= 19'sd18000)  ? (az_sum - 19'sd36000) :
                                (az_sum < -19'sd18000)  ? (az_sum + 19'sd36000) : az_sum;
    wire signed [18:0] az_w2  = (az_w1  >= 19'sd18000)  ? (az_w1  - 19'sd36000) : az_w1;
    logic signed [16:0] az_c;

    wire signed [16:0] cos_small = 17'sd32768 - $signed({1'b0, mp[31:16]});   // 1 - turned^2/2, 15 fractional bits

    // ---- angle for the CORDIC --------------------------------------------------------------------
    logic signed [18:0] base;               // radians, 15 fractional bits
    logic [1:0] cls;
    logic signed [18:0] off;
    always_comb begin
        case ({four_offsets, cls})
            3'b100:  off =  OFF_4P2;
            3'b101:  off =  OFF_1P4;
            3'b110:  off = -OFF_1P4;
            3'b111:  off = -OFF_4P2;
            default: off =  19'sd0;
        endcase
    end
    wire signed [18:0] th_raw  = base + off;
    wire signed [18:0] th_wrap = (th_raw >=  PI_Q15) ? (th_raw - TWO_PI_Q15) :
                                 (th_raw <  -PI_Q15) ? (th_raw + TWO_PI_Q15) : th_raw;

    function automatic signed [15:0] sat16;
        input signed [17:0] x;
        begin
            sat16 = (x > 18'sd32767) ? 16'sd32767 : (x < -18'sd32767) ? -16'sd32767 : x[15:0];
        end
    endfunction

    // moved totals rounded to whole counts and clamped to 16 bits
    function automatic signed [15:0] to_counts;
        input signed [31:0] m;
        reg signed [32:0] r;
        reg signed [16:0] whole;
        begin
            r     = $signed({m[31], m}) + 33'sd32768;
            whole = r[32:16];               // a part-select is unsigned, so it goes through a signed variable
            to_counts = (whole > 17'sd32767) ? 16'sd32767 : (whole < -17'sd32767) ? -16'sd32767 : whole[15:0];
        end
    endfunction

    wire signed [15:0] sin_s = sat16(cordic_sin);
    wire signed [15:0] cos_s = sat16(cordic_cos);

    always_ff @(posedge clk) begin
        if (rst) begin
            st <= IDLE;
            ma <= 17'sd0; mb <= 17'sd0;
            omega_q <= 16'sd0; v_q <= 16'sd0; kt <= 17'sd0; km <= 17'sd0; dturn <= 24'sd0; ds <= 17'sd0;
            turned <= 32'sd0; moved_x <= 32'sd0; moved_y <= 32'sd0;
            fire_req <= 1'b0; upd_req <= 1'b0;
            f_az <= 16'd0; f_second <= 1'b0; f_sweep <= 1'b0;
            w_az <= 16'd0; w_second <= 1'b0; w_sweep <= 1'b0;
            az_last <= 16'd0; az_step <= 16'd0; az_c <= 17'sd0;
            base <= 19'sd0; cls <= 2'd0;
            cordic_start <= 1'b0; cordic_theta <= 18'sd0;
            fire_done <= 1'b0; fire_sweep_start <= 1'b0;
            sin0 <= 16'sd0; sin1 <= 16'sd0; sin2 <= 16'sd0; sin3 <= 16'sd0;
            cos0 <= 16'sd32767; cos1 <= 16'sd32767; cos2 <= 16'sd32767; cos3 <= 16'sd32767;
            shift_x <= 16'sd0; shift_y <= 16'sd0;
            state_seen <= 1'b0; overrun <= 1'b0;
        end else begin
            cordic_start <= 1'b0;
            fire_done    <= 1'b0;
            overrun      <= 1'b0;

            // requests are remembered until the sequencer is free
            if (state_valid) upd_req <= 1'b1;
            if (firing_start) begin
                if (fire_req) overrun <= 1'b1;
                fire_req <= 1'b1;
                f_az     <= azimuth;
                f_second <= firing_second;
                f_sweep  <= sweep_start;
            end

            case (st)
                IDLE: begin
                    if (fire_req) begin
                        if (!firing_start) fire_req <= 1'b0;
                        w_az <= f_az; w_second <= f_second; w_sweep <= f_sweep;
                        st <= F_ACC;
                    end else if (upd_req) begin
                        if (!state_valid) upd_req <= 1'b0;
                        omega_q <= fp_to_q12(state_omega);
                        v_q     <= fp_to_q12(state_v);
                        ma <= $signed({1'b0, firing_period_ns}); mb <= K_TURN;
                        st <= U_KT;
                    end
                end

                // ---- new filter state: work out the per-firing steps -------------------------
                U_KT:  st <= U_KT2;
                U_KT2: begin kt <= mp[31:15];
                             ma <= $signed({1'b0, firing_period_ns}); mb <= K_MOVE; st <= U_KM; end
                U_KM:  st <= U_KM2;
                U_KM2: begin km <= scale_4mm ? mp[32:16] : mp[31:15];
                             ma <= {omega_q[15], omega_q}; mb <= kt; st <= U_DT; end
                U_DT:  st <= U_DT2;
                U_DT2: begin dturn <= {{2{mp[33]}}, mp[33:12]};     // omega(Q12) x kt, back to 29 fractional bits
                             ma <= {v_q[15], v_q}; mb <= km; st <= U_DS; end
                U_DS:  st <= U_DS2;
                U_DS2: begin ds <= mp[32:16]; state_seen <= 1'b1; st <= IDLE; end

                // ---- a firing --------------------------------------------------------------
                F_ACC: begin
                    if (w_sweep) begin
                        turned <= 32'sd0; moved_x <= 32'sd0; moved_y <= 32'sd0;
                    end else begin
                        turned <= turned + {{8{dturn[23]}}, dturn};
                    end
                    if (!w_second) begin az_step <= step_now; az_last <= w_az; end
                    az_c <= az_w2[16:0];
                    st <= F_T2;
                end
                F_T2:  begin ma <= t17; mb <= t17; st <= F_T2B; end
                F_T2B: st <= F_COS;
                F_COS: begin ma <= ds; mb <= cos_small; st <= F_MX; end
                F_MX:  st <= F_MX2;
                F_MX2: begin if (!w_sweep) moved_x <= moved_x + {{13{mp[33]}}, mp[33:15]};
                             ma <= ds; mb <= t17; st <= F_MY; end
                F_MY:  st <= F_MY2;
                F_MY2: begin if (!w_sweep) moved_y <= moved_y + {{13{mp[33]}}, mp[33:15]};
                             ma <= az_c; mb <= K_DEG2RAD; st <= F_AZ; end
                F_AZ:  st <= F_AZ2;
                F_AZ2: begin base <= mp[31:13] - (corr_rot_en ? {t15[17], t15} : 19'sd0);
                             cls <= 2'd0; st <= F_CORD; end
                F_CORD: begin cordic_theta <= th_wrap[17:0]; cordic_start <= 1'b1; st <= F_WAIT; end
                F_WAIT: begin
                    if (cordic_done) begin
                        if (!four_offsets) begin
                            sin0 <= sin_s; sin1 <= sin_s; sin2 <= sin_s; sin3 <= sin_s;
                            cos0 <= cos_s; cos1 <= cos_s; cos2 <= cos_s; cos3 <= cos_s;
                            st <= F_OUT;
                        end else begin
                            case (cls)
                                2'd0:    begin sin0 <= sin_s; cos0 <= cos_s; end
                                2'd1:    begin sin1 <= sin_s; cos1 <= cos_s; end
                                2'd2:    begin sin2 <= sin_s; cos2 <= cos_s; end
                                default: begin sin3 <= sin_s; cos3 <= cos_s; end
                            endcase
                            cls <= cls + 2'd1;
                            st  <= (cls == 2'd3) ? F_OUT : F_CORD;
                        end
                    end
                end
                F_OUT: begin
                    shift_x <= corr_shift_en ? to_counts(moved_x) : 16'sd0;
                    shift_y <= corr_shift_en ? to_counts(moved_y) : 16'sd0;
                    fire_sweep_start <= w_sweep;
                    fire_done <= 1'b1;
                    st <= IDLE;
                end
                default: st <= IDLE;
            endcase
        end
    end

endmodule
