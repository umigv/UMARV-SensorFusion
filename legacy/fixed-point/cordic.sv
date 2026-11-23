`timescale 1ns/1ps
// Shared iterative CORDIC interface. Mode 0 rotates the unit vector by theta
// (radians); modes 1 and 2 are reserved for circular and hyperbolic vectoring.
// An unsupported request completes with error and holds the previous outputs.
// sin_theta, cos_theta, overflow and error update only on completion or reset.
module cordic #(
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16,
    parameter integer ITERATIONS = FRAC + 4
)(
    input logic clk, resetn, start,
    input logic [1:0] mode,
    input logic signed [WIDTH-1:0] theta,
    output logic signed [WIDTH-1:0] sin_theta, cos_theta,
    output logic busy, done, overflow, error
);
    localparam logic [1:0] CIRCULAR_ROTATION = 2'd0;
    // Include guard bits for angular reduction across the full input range,
    // coordinate rounding, and the smallest micro-rotation.
    localparam integer ROT_FRAC = FRAC + 8 > ITERATIONS + 2 ?
                                  FRAC + 8 : ITERATIONS + 2;
    localparam integer WORK_FRAC = ROT_FRAC > 34 ? ROT_FRAC : 34;
    localparam integer GUARD = WORK_FRAC - FRAC;
    localparam integer XY_WIDTH = WORK_FRAC + 3;
    localparam integer Z_WIDTH = WORK_FRAC + 5;
    localparam integer REDUCE_STEPS = WIDTH + GUARD;
    localparam integer COUNT_WIDTH = $clog2(REDUCE_STEPS + 1);
    localparam integer SHIFT_WIDTH = $clog2(ITERATIONS + 1);

    typedef enum logic [2:0] {IDLE, REDUCE_ANGLE, PREPARE, ITERATE,
                              PUBLISH, UNSUPPORTED} phase_t;
    phase_t phase;
    logic [1:0] active_mode;
    logic [COUNT_WIDTH-1:0] count;
    logic [SHIFT_WIDTH-1:0] step;
    logic [WIDTH-1:0] angle_bits;
    logic input_negative, flip_quadrant;
    logic signed [XY_WIDTH-1:0] x, y;
    logic signed [Z_WIDTH-1:0] z;

    // Constants rounded to Q46, then rounded to the selected working precision.
    localparam integer CONSTANT_SHIFT = 46 - WORK_FRAC;
    function automatic logic signed [Z_WIDTH-1:0] rescale_constant(
        input logic [49:0] value
    );
        rescale_constant = Z_WIDTH'((value + (50'd1 << (CONSTANT_SHIFT-1))) >> CONSTANT_SHIFT);
    endfunction
    localparam logic signed [Z_WIDTH-1:0] HALF_PI = rescale_constant(50'd110534964875444);
    localparam logic signed [Z_WIDTH-1:0] PI = rescale_constant(50'd221069929750889);
    localparam logic signed [Z_WIDTH-1:0] THREE_HALF_PI = rescale_constant(50'd331604894626333);
    localparam logic signed [Z_WIDTH-1:0] TWO_PI = rescale_constant(50'd442139859501778);

    function automatic logic signed [Z_WIDTH-1:0] circular_angle(input integer index);
        case (index)
             0: circular_angle = rescale_constant(50'd55267482437722);
             1: circular_angle = rescale_constant(50'd32626299986363);
             2: circular_angle = rescale_constant(50'd17238840874560);
             3: circular_angle = rescale_constant(50'd8750704798476);
             4: circular_angle = rescale_constant(50'd4392333272480);
             5: circular_angle = rescale_constant(50'd2198307846807);
             6: circular_angle = rescale_constant(50'd1099422162396);
             7: circular_angle = rescale_constant(50'd549744629487);
             8: circular_angle = rescale_constant(50'd274876508855);
             9: circular_angle = rescale_constant(50'd137438778710);
            10: circular_angle = rescale_constant(50'd68719454891);
            11: circular_angle = rescale_constant(50'd34359735637);
            12: circular_angle = rescale_constant(50'd17179868843);
            13: circular_angle = rescale_constant(50'd8589934549);
            14: circular_angle = rescale_constant(50'd4294967291);
            15: circular_angle = rescale_constant(50'd2147483647);
            16: circular_angle = rescale_constant(50'd1073741824);
            17: circular_angle = rescale_constant(50'd536870912);
            18: circular_angle = rescale_constant(50'd268435456);
            19: circular_angle = rescale_constant(50'd134217728);
            20: circular_angle = rescale_constant(50'd67108864);
            21: circular_angle = rescale_constant(50'd33554432);
            22: circular_angle = rescale_constant(50'd16777216);
            23: circular_angle = rescale_constant(50'd8388608);
            24: circular_angle = rescale_constant(50'd4194304);
            25: circular_angle = rescale_constant(50'd2097152);
            26: circular_angle = rescale_constant(50'd1048576);
            27: circular_angle = rescale_constant(50'd524288);
            28: circular_angle = rescale_constant(50'd262144);
            29: circular_angle = rescale_constant(50'd131072);
            30: circular_angle = rescale_constant(50'd65536);
            31: circular_angle = rescale_constant(50'd32768);
            32: circular_angle = rescale_constant(50'd16384);
            33: circular_angle = rescale_constant(50'd8192);
            34: circular_angle = rescale_constant(50'd4096);
            35: circular_angle = rescale_constant(50'd2048);
            36: circular_angle = rescale_constant(50'd1024);
            37: circular_angle = rescale_constant(50'd512);
            38: circular_angle = rescale_constant(50'd256);
            39: circular_angle = rescale_constant(50'd128);
            default: circular_angle = '0;
        endcase
    endfunction

    // Reciprocal gain for this finite rotation schedule. Constant-folded.
    function automatic logic signed [XY_WIDTH-1:0] circular_seed(input integer steps);
        logic [49:0] seed;
        case (steps)
             4: seed = 50'd42842877836648;
             5: seed = 50'd42759444694494;
             6: seed = 50'd42738581338924;
             7: seed = 50'd42733365182032;
             8: seed = 50'd42732061122916;
             9: seed = 50'd42731735106894;
            10: seed = 50'd42731653602810;
            11: seed = 50'd42731633226785;
            12: seed = 50'd42731628132778;
            13: seed = 50'd42731626859276;
            14: seed = 50'd42731626540901;
            15: seed = 50'd42731626461307;
            16: seed = 50'd42731626441408;
            17: seed = 50'd42731626436434;
            18: seed = 50'd42731626435190;
            19: seed = 50'd42731626434879;
            20: seed = 50'd42731626434801;
            21: seed = 50'd42731626434782;
            22: seed = 50'd42731626434777;
            // At Q46 all schedules of 23 or more steps share this value.
            default: seed = 50'd42731626434776;
        endcase
        circular_seed = XY_WIDTH'(rescale_constant(seed));
    endfunction

    localparam logic signed [XY_WIDTH-1:0] ROTATION_SEED = circular_seed(ITERATIONS);
    localparam logic signed [XY_WIDTH:0] MAX_OUT =
        (XY_WIDTH+1)'($signed({1'b0, {(WIDTH-1){1'b1}}}));
    localparam logic signed [XY_WIDTH:0] MIN_OUT =
        (XY_WIDTH+1)'($signed({1'b1, {(WIDTH-1){1'b0}}}));

    wire [WIDTH-1:0] theta_magnitude = theta[WIDTH-1] ? $unsigned(-theta) : $unsigned(theta);
    // Active mode selects the schedule; the x/y shifters and adders are shared.
    logic [SHIFT_WIDTH-1:0] shift_index;
    logic signed [Z_WIDTH-1:0] micro_angle;
    logic direction;
    always_comb begin
        shift_index = '0;
        micro_angle = '0;
        direction = 1'b0;
        case (active_mode)
            CIRCULAR_ROTATION: begin
                shift_index = step;
                micro_angle = circular_angle(int'(step));
                direction = z >= 0;
            end
            default: begin end
        endcase
    end

    wire signed [XY_WIDTH-1:0] x_shifted = x >>> shift_index;
    wire signed [XY_WIDTH-1:0] y_shifted = y >>> shift_index;
    wire signed [XY_WIDTH-1:0] x_next = x + (direction ? -y_shifted : y_shifted);
    wire signed [XY_WIDTH-1:0] y_next = y + (direction ? x_shifted : -x_shifted);

    // The same phase update handles modulo reduction, quadrant folding and
    // residual-angle updates. The reduction consumes one magnitude bit/clock.
    wire signed [Z_WIDTH-1:0] trial = $signed({z[Z_WIDTH-2:0], angle_bits[WIDTH-1]});
    logic signed [Z_WIDTH-1:0] phase_base, phase_delta;
    always_comb begin
        phase_base = z;
        phase_delta = direction ? -micro_angle : micro_angle;
        case (phase)
            REDUCE_ANGLE: begin
                phase_base = trial;
                phase_delta = -TWO_PI;
            end
            PREPARE: begin
                if (z > THREE_HALF_PI) phase_delta = -TWO_PI;
                else if (z > HALF_PI) phase_delta = -PI;
                else phase_delta = '0;
            end
            default: begin end
        endcase
    end
    wire signed [Z_WIDTH-1:0] phase_sum = phase_base + phase_delta;

    function automatic logic signed [XY_WIDTH:0] round_output(
        input logic signed [XY_WIDTH-1:0] value,
        input logic negate
    );
        logic [XY_WIDTH:0] magnitude, rounded;
        logic signed [XY_WIDTH:0] extended;
        begin
            extended = (XY_WIDTH+1)'(value);
            magnitude = extended < 0 ? $unsigned(-extended) : $unsigned(extended);
            rounded = (magnitude >> GUARD) + (XY_WIDTH+1)'(magnitude[GUARD-1]);
            round_output = ((extended < 0) ^ negate) ? -$signed(rounded) : $signed(rounded);
        end
    endfunction

    function automatic logic signed [WIDTH-1:0] saturate(
        input logic signed [XY_WIDTH:0] value
    );
        if (value > MAX_OUT) saturate = WIDTH'(MAX_OUT);
        else if (value < MIN_OUT) saturate = WIDTH'(MIN_OUT);
        else saturate = WIDTH'(value);
    endfunction

    wire signed [XY_WIDTH:0] sin_rounded = round_output(y, input_negative ^ flip_quadrant);
    wire signed [XY_WIDTH:0] cos_rounded = round_output(x, flip_quadrant);

    initial begin
        if (WIDTH < 2 || WIDTH > 32 || FRAC < 0 || FRAC >= WIDTH)
            $fatal(1, "Require 2 <= WIDTH <= 32 and 0 <= FRAC < WIDTH");
        if (ITERATIONS < 4 || ITERATIONS > 40)
            $fatal(1, "Require 4 <= ITERATIONS <= 40");
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            phase <= IDLE;
            active_mode <= CIRCULAR_ROTATION;
            busy <= 0;
            done <= 0;
            overflow <= 0;
            error <= 0;
            sin_theta <= '0;
            cos_theta <= '0;
            count <= '0;
            step <= '0;
            angle_bits <= '0;
            input_negative <= 0;
            flip_quadrant <= 0;
            x <= '0;
            y <= '0;
            z <= '0;
        end else begin
            done <= 0;
            case (phase)
                IDLE: if (start) begin
                    busy <= 1;
                    active_mode <= mode;
                    if (mode == CIRCULAR_ROTATION) begin
                        angle_bits <= theta_magnitude;
                        input_negative <= theta[WIDTH-1];
                        count <= COUNT_WIDTH'(REDUCE_STEPS-1);
                        z <= '0;
                        phase <= REDUCE_ANGLE;
                    end else begin
                        phase <= UNSUPPORTED;
                    end
                end
                REDUCE_ANGLE: begin
                    z <= phase_sum < 0 ? trial : phase_sum;
                    // After WIDTH clocks, the shift register supplies GUARD zeros.
                    angle_bits <= angle_bits << 1;
                    count <= count - 1'b1;
                    if (count == 0) phase <= PREPARE;
                end
                PREPARE: begin
                    x <= ROTATION_SEED;
                    y <= '0;
                    z <= phase_sum;
                    flip_quadrant <= z > HALF_PI && z <= THREE_HALF_PI;
                    step <= '0;
                    phase <= ITERATE;
                end
                ITERATE: begin
                    x <= x_next;
                    y <= y_next;
                    z <= phase_sum;
                    step <= step + 1'b1;
                    if (step == SHIFT_WIDTH'(ITERATIONS-1)) phase <= PUBLISH;
                end
                PUBLISH: begin
                    sin_theta <= saturate(sin_rounded);
                    cos_theta <= saturate(cos_rounded);
                    overflow <= sin_rounded > MAX_OUT || sin_rounded < MIN_OUT ||
                                cos_rounded > MAX_OUT || cos_rounded < MIN_OUT;
                    error <= 0;
                    busy <= 0;
                    done <= 1;
                    phase <= IDLE;
                end
                UNSUPPORTED: begin
                    error <= 1;
                    overflow <= 0;
                    busy <= 0;
                    done <= 1;
                    phase <= IDLE;
                end
                default: begin
                    busy <= 0;
                    phase <= IDLE;
                end
            endcase
        end
    end
endmodule
