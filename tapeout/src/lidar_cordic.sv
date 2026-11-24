// Iterative CORDIC for the lidar block: sine and cosine of an angle in radians.
//
// This is the team's shared CORDIC (cordic.sv on the Sigma branch) fixed to one
// configuration and rewritten in the plain SystemVerilog subset the Tiny Tapeout tools accept.
// Behaviour is bit-for-bit the same as the original built with
//   WIDTH = 18, FRAC = 15, ITERATIONS = 17
// and with the original's 34-bit floor on internal precision removed (the lidar's angle is
// always within a turn or two, so the extra bits for reducing very large angles are not needed).
//
// Differences from the original, all mechanical:
//   - parameters and the functions that derived constants from them are replaced by the
//     constants themselves
//   - enum states become localparams; size casts and typed function arguments are gone
//
// theta, sin_theta and cos_theta are signed Q3.15: one count = 1/32768.
// theta is accepted over the full +/-4 rad range. A result takes 46 clocks.
// sin_theta, cos_theta, overflow and error update only on completion or reset.
module lidar_cordic (
    input  logic               clk,
    input  logic               resetn,
    input  logic               start,
    input  logic        [1:0]  mode,        // 0: rotate the unit vector by theta. others: unsupported
    input  logic signed [17:0] theta,
    output logic signed [17:0] sin_theta,
    output logic signed [17:0] cos_theta,
    output logic               busy,
    output logic               done,
    output logic               overflow,
    output logic               error
);
    // Working precision: 23 fractional bits (8 guard bits below the 15 that are output)
    localparam GUARD        = 8;
    localparam REDUCE_STEPS = 26;        // WIDTH + GUARD
    localparam ITERATIONS   = 17;

    localparam [1:0] CIRCULAR_ROTATION = 2'd0;

    localparam [2:0] IDLE = 3'd0, REDUCE_ANGLE = 3'd1, PREPARE = 3'd2, ITERATE = 3'd3,
                     PUBLISH = 3'd4, UNSUPPORTED = 3'd5;
    logic [2:0] phase;
    logic [1:0] active_mode;
    logic [4:0] count;
    logic [4:0] step;
    logic [17:0] angle_bits;
    logic input_negative, flip_quadrant;
    logic signed [25:0] x, y;     // Q3.23
    logic signed [27:0] z;        // Q5.23

    // Angle constants in Q5.23 radians
    localparam signed [27:0] HALF_PI       = 28'sd13176795;
    localparam signed [27:0] PI            = 28'sd26353589;
    localparam signed [27:0] THREE_HALF_PI = 28'sd39530384;
    localparam signed [27:0] TWO_PI        = 28'sd52707179;

    // 1 / (CORDIC gain for 17 steps), Q3.23
    localparam signed [25:0] ROTATION_SEED = 26'sd5094007;

    localparam signed [26:0] MAX_OUT =  27'sd131071;
    localparam signed [26:0] MIN_OUT = -27'sd131072;

    wire [17:0] theta_magnitude = theta[17] ? -theta : theta;

    // atan(2^-step) in Q5.23 radians
    logic signed [27:0] micro_angle;
    always_comb begin
        case (step)
            5'd0 : micro_angle = 28'sd6588397;
            5'd1 : micro_angle = 28'sd3889358;
            5'd2 : micro_angle = 28'sd2055030;
            5'd3 : micro_angle = 28'sd1043165;
            5'd4 : micro_angle = 28'sd523607;
            5'd5 : micro_angle = 28'sd262059;
            5'd6 : micro_angle = 28'sd131061;
            5'd7 : micro_angle = 28'sd65535;
            5'd8 : micro_angle = 28'sd32768;
            5'd9 : micro_angle = 28'sd16384;
            5'd10: micro_angle = 28'sd8192;
            5'd11: micro_angle = 28'sd4096;
            5'd12: micro_angle = 28'sd2048;
            5'd13: micro_angle = 28'sd1024;
            5'd14: micro_angle = 28'sd512;
            5'd15: micro_angle = 28'sd256;
            5'd16: micro_angle = 28'sd128;
            default: micro_angle = 28'sd0;
        endcase
    end

    wire direction = (active_mode == CIRCULAR_ROTATION) && (z >= 0);
    wire [4:0] shift_index = (active_mode == CIRCULAR_ROTATION) ? step : 5'd0;
    wire signed [27:0] micro_sel = (active_mode == CIRCULAR_ROTATION) ? micro_angle : 28'sd0;

    wire signed [25:0] x_shifted = x >>> shift_index;
    wire signed [25:0] y_shifted = y >>> shift_index;
    wire signed [25:0] x_next = direction ? (x - y_shifted) : (x + y_shifted);
    wire signed [25:0] y_next = direction ? (y + x_shifted) : (y - x_shifted);

    // The same adder handles modulo reduction, quadrant folding and the residual-angle
    // updates. The reduction consumes one magnitude bit per clock.
    wire signed [27:0] trial = {z[26:0], angle_bits[17]};
    logic signed [27:0] phase_base, phase_delta;
    always_comb begin
        phase_base  = z;
        phase_delta = direction ? -micro_sel : micro_sel;
        case (phase)
            REDUCE_ANGLE: begin
                phase_base  = trial;
                phase_delta = -TWO_PI;
            end
            PREPARE: begin
                if (z > THREE_HALF_PI) phase_delta = -TWO_PI;
                else if (z > HALF_PI)  phase_delta = -PI;
                else                   phase_delta = 28'sd0;
            end
            default: begin end
        endcase
    end
    wire signed [27:0] phase_sum = phase_base + phase_delta;

    // Round the working value to the output step (round half away from zero), apply the sign
    wire signed [26:0] y_ext = {y[25], y};
    wire signed [26:0] x_ext = {x[25], x};
    wire        [26:0] y_mag = y_ext[26] ? -y_ext : y_ext;
    wire        [26:0] x_mag = x_ext[26] ? -x_ext : x_ext;
    wire        [26:0] y_rnd = (y_mag >> GUARD) + {26'd0, y_mag[GUARD-1]};
    wire        [26:0] x_rnd = (x_mag >> GUARD) + {26'd0, x_mag[GUARD-1]};
    wire sin_negate = y_ext[26] ^ input_negative ^ flip_quadrant;
    wire cos_negate = x_ext[26] ^ flip_quadrant;
    wire signed [26:0] sin_rounded = sin_negate ? -$signed(y_rnd) : $signed(y_rnd);
    wire signed [26:0] cos_rounded = cos_negate ? -$signed(x_rnd) : $signed(x_rnd);

    wire sin_over = (sin_rounded > MAX_OUT) || (sin_rounded < MIN_OUT);
    wire cos_over = (cos_rounded > MAX_OUT) || (cos_rounded < MIN_OUT);
    wire signed [17:0] sin_sat = (sin_rounded > MAX_OUT) ? MAX_OUT[17:0] :
                                 (sin_rounded < MIN_OUT) ? MIN_OUT[17:0] : sin_rounded[17:0];
    wire signed [17:0] cos_sat = (cos_rounded > MAX_OUT) ? MAX_OUT[17:0] :
                                 (cos_rounded < MIN_OUT) ? MIN_OUT[17:0] : cos_rounded[17:0];

    always_ff @(posedge clk) begin
        if (!resetn) begin
            phase <= IDLE;
            active_mode <= CIRCULAR_ROTATION;
            busy <= 1'b0;
            done <= 1'b0;
            overflow <= 1'b0;
            error <= 1'b0;
            sin_theta <= 18'sd0;
            cos_theta <= 18'sd0;
            count <= 5'd0;
            step <= 5'd0;
            angle_bits <= 18'd0;
            input_negative <= 1'b0;
            flip_quadrant <= 1'b0;
            x <= 26'sd0;
            y <= 26'sd0;
            z <= 28'sd0;
        end else begin
            done <= 1'b0;
            case (phase)
                IDLE: if (start) begin
                    busy <= 1'b1;
                    active_mode <= mode;
                    if (mode == CIRCULAR_ROTATION) begin
                        angle_bits <= theta_magnitude;
                        input_negative <= theta[17];
                        count <= REDUCE_STEPS - 1;
                        z <= 28'sd0;
                        phase <= REDUCE_ANGLE;
                    end else begin
                        phase <= UNSUPPORTED;
                    end
                end
                REDUCE_ANGLE: begin
                    z <= (phase_sum < 0) ? trial : phase_sum;
                    // After WIDTH clocks, the shift register supplies GUARD zeros.
                    angle_bits <= angle_bits << 1;
                    count <= count - 1'b1;
                    if (count == 5'd0) phase <= PREPARE;
                end
                PREPARE: begin
                    x <= ROTATION_SEED;
                    y <= 26'sd0;
                    z <= phase_sum;
                    flip_quadrant <= (z > HALF_PI) && (z <= THREE_HALF_PI);
                    step <= 5'd0;
                    phase <= ITERATE;
                end
                ITERATE: begin
                    x <= x_next;
                    y <= y_next;
                    z <= phase_sum;
                    step <= step + 1'b1;
                    if (step == ITERATIONS - 1) phase <= PUBLISH;
                end
                PUBLISH: begin
                    sin_theta <= sin_sat;
                    cos_theta <= cos_sat;
                    overflow <= sin_over || cos_over;
                    error <= 1'b0;
                    busy <= 1'b0;
                    done <= 1'b1;
                    phase <= IDLE;
                end
                UNSUPPORTED: begin
                    error <= 1'b1;
                    overflow <= 1'b0;
                    busy <= 1'b0;
                    done <= 1'b1;
                    phase <= IDLE;
                end
                default: begin
                    busy <= 1'b0;
                    phase <= IDLE;
                end
            endcase
        end
    end
endmodule
