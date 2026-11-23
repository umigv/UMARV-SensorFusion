`timescale 1ns/1ps
// Sequential Euler propagation of nine [x, y, theta, omega] sigma points.
// State and velocity use WIDTH/FRAC; dt uses signed Q1.(WIDTH-1) seconds.
// At WIDTH=32, drive dt=2147484 for 1 ms. Valid timesteps are in [0, 1).
// Headings remain unwrapped in radians. The CORDIC reduces its input angle.
// Each arbiter grants a drained unit and holds ownership until request falls.
// A timeout or unit error is sticky and retains the affected unit until reset.
// Watchdogs cover issued operations; this module waits for grants indefinitely.
module process_model #(
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16,
    parameter integer ACC_WIDTH = 2*WIDTH + 8,
    parameter integer MAC_TIMEOUT_CYCLES = 16,
    parameter integer CORDIC_TIMEOUT_CYCLES = 128
)(
    input logic clk, resetn, start,
    input logic signed [WIDTH-1:0] sigma [0:8][0:3],
    input logic signed [WIDTH-1:0] v_wheel, dt,
    output logic signed [WIDTH-1:0] propagated [0:8][0:3],
    output logic busy, done, overflow, error,

    output logic mac_request,
    input logic mac_grant,
    output wire mac_start, mac_clear, mac_last,
    output logic signed [WIDTH-1:0] mac_a, mac_b,
    input logic mac_busy, mac_result_valid, mac_overflow, mac_acc_overflow,
    input logic signed [WIDTH-1:0] mac_result,
    input logic signed [ACC_WIDTH-1:0] mac_full_result,

    output logic cordic_request,
    input logic cordic_grant,
    output wire cordic_start,
    output wire [1:0] cordic_mode,
    output wire signed [WIDTH-1:0] cordic_theta,
    input logic cordic_busy, cordic_done, cordic_overflow, cordic_error,
    input logic signed [WIDTH-1:0] cordic_sin, cordic_cos
);
    localparam integer DT_FRAC = WIDTH-1;
    localparam integer MAX_TIMEOUT = MAC_TIMEOUT_CYCLES > CORDIC_TIMEOUT_CYCLES ?
                                     MAC_TIMEOUT_CYCLES : CORDIC_TIMEOUT_CYCLES;
    localparam integer WAIT_WIDTH = MAX_TIMEOUT > 1 ? $clog2(MAX_TIMEOUT) : 1;
    localparam logic [WAIT_WIDTH-1:0] MAC_LIMIT = WAIT_WIDTH'(MAC_TIMEOUT_CYCLES-1);
    localparam logic [WAIT_WIDTH-1:0] CORDIC_LIMIT = WAIT_WIDTH'(CORDIC_TIMEOUT_CYCLES-1);

    typedef enum logic [2:0] {IDLE, REQUEST_CORDIC, WAIT_CORDIC_RESULT, MAC_LAUNCH,
                              WAIT_MAC_RESULT, ADVANCE_POINT, PUBLISH, FAULT} phase_t;
    typedef enum logic [1:0] {WHEEL_STEP, POSITION_X, POSITION_Y, HEADING} operation_t;
    phase_t phase;
    operation_t operation;
    logic [3:0] point;
    logic [WAIT_WIDTH-1:0] wait_cycles;
    logic saw_busy;

    // The private bank holds the captured inputs, updated in place as each
    // point is propagated. Its fourth component never changes after capture.
    logic signed [WIDTH-1:0] work [0:8][0:3];
    logic signed [WIDTH-1:0] velocity, timestep, distance, sine, cosine;

    localparam logic signed [WIDTH:0] MAX_POS = {2'b00, {(WIDTH-1){1'b1}}};
    localparam logic signed [WIDTH:0] MIN_NEG = {2'b11, {(WIDTH-1){1'b0}}};
    function automatic logic signed [WIDTH-1:0] saturate(input logic signed [WIDTH:0] value);
        if (value > MAX_POS) saturate = MAX_POS[WIDTH-1:0];
        else if (value < MIN_NEG) saturate = MIN_NEG[WIDTH-1:0];
        else saturate = value[WIDTH-1:0];
    endfunction

    // Single-product full_result is a signed 2*WIDTH-bit product. For the
    // mixed-Q timestep products, discard DT_FRAC bits with the MAC's
    // round-to-nearest, ties-away-from-zero convention.
    wire signed [2*WIDTH-1:0] time_product = mac_full_result[2*WIDTH-1:0];
    wire [2*WIDTH-1:0] magnitude =
        time_product < 0 ? $unsigned(-time_product) : $unsigned(time_product);
    wire [WIDTH:0] rounded_magnitude =
        (WIDTH+1)'(magnitude >> DT_FRAC) + (WIDTH+1)'(magnitude[DT_FRAC-1]);
    wire signed [WIDTH:0] time_delta =
        time_product < 0 ? -$signed(rounded_magnitude) : $signed(rounded_magnitude);

    wire [1:0] component = operation == POSITION_X ? 2'd0 :
                           operation == POSITION_Y ? 2'd1 : 2'd2;
    wire signed [WIDTH:0] delta = operation == HEADING ? time_delta : (WIDTH+1)'(mac_result);
    wire signed [WIDTH:0] updated = (WIDTH+1)'(work[point][component]) + delta;

    assign cordic_mode = 2'd0;
    assign cordic_theta = work[point][2];
    assign cordic_start = resetn && phase == REQUEST_CORDIC && cordic_grant &&
                          !cordic_busy && !cordic_done;
    assign mac_start = resetn && phase == MAC_LAUNCH && mac_grant &&
                       !mac_busy && !mac_result_valid;
    assign mac_clear = mac_start;
    assign mac_last = mac_start;

    always_comb begin
        case (operation)
            WHEEL_STEP: begin mac_a = velocity; mac_b = timestep; end
            POSITION_X: begin mac_a = distance; mac_b = cosine; end
            POSITION_Y: begin mac_a = distance; mac_b = sine; end
            HEADING: begin mac_a = work[point][3]; mac_b = timestep; end
            default: begin mac_a = '0; mac_b = '0; end
        endcase
    end

    initial begin
        if (WIDTH < 2 || FRAC < 0 || FRAC >= WIDTH || ACC_WIDTH < 2*WIDTH)
            $fatal(1, "Invalid process-model fixed-point parameters");
        if (MAC_TIMEOUT_CYCLES < 1 || CORDIC_TIMEOUT_CYCLES < 1)
            $fatal(1, "Process-model watchdog limits must be positive");
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            phase <= IDLE;
            operation <= WHEEL_STEP;
            point <= '0;
            busy <= 0;
            done <= 0;
            error <= 0;
            overflow <= 0;
            mac_request <= 0;
            cordic_request <= 0;
            wait_cycles <= '0;
            saw_busy <= 0;
            velocity <= '0;
            timestep <= '0;
            distance <= '0;
            sine <= '0;
            cosine <= '0;
            for (integer i=0; i<9; i=i+1)
                for (integer j=0; j<4; j=j+1) begin
                    work[i][j] <= '0;
                    propagated[i][j] <= '0;
                end
        end else begin
            done <= 0;
            case (phase)
                IDLE: if (start) begin
                    for (integer i=0; i<9; i=i+1)
                        for (integer j=0; j<4; j=j+1)
                            work[i][j] <= sigma[i][j];
                    velocity <= v_wheel;
                    timestep <= dt;
                    point <= 0;
                    overflow <= 0;
                    busy <= 1;
                    if (dt[WIDTH-1]) begin
                        error <= 1;
                        phase <= FAULT;
                    end else begin
                        cordic_request <= 1;
                        phase <= REQUEST_CORDIC;
                    end
                end
                REQUEST_CORDIC: if (cordic_start) begin
                    saw_busy <= 0;
                    wait_cycles <= 0;
                    phase <= WAIT_CORDIC_RESULT;
                end
                WAIT_CORDIC_RESULT: begin
                    if (cordic_busy) saw_busy <= 1;
                    if (cordic_grant && saw_busy && !cordic_busy && cordic_done) begin
                        if (cordic_error) begin
                            error <= 1;
                            phase <= FAULT;
                        end else begin
                            sine <= cordic_sin;
                            cosine <= cordic_cos;
                            if (cordic_overflow) overflow <= 1;
                            cordic_request <= 0;
                            mac_request <= 1;
                            operation <= point == 0 ? WHEEL_STEP : POSITION_X;
                            phase <= MAC_LAUNCH;
                        end
                    end else if (wait_cycles == CORDIC_LIMIT) begin
                        error <= 1;
                        phase <= FAULT;
                    end else wait_cycles <= wait_cycles + 1'b1;
                end
                MAC_LAUNCH: if (mac_start) begin
                    saw_busy <= 0;
                    wait_cycles <= 0;
                    phase <= WAIT_MAC_RESULT;
                end
                WAIT_MAC_RESULT: begin
                    if (mac_busy) saw_busy <= 1;
                    if (mac_grant && saw_busy && !mac_busy && mac_result_valid) begin
                        if (mac_acc_overflow) begin
                            error <= 1;
                            overflow <= 1;
                            phase <= FAULT;
                        end else if (operation == WHEEL_STEP) begin
                            // Nonnegative dt < 1 keeps distance in range.
                            // The native FRAC-scaled result is unused here.
                            distance <= WIDTH'(time_delta);
                            operation <= POSITION_X;
                            phase <= MAC_LAUNCH;
                        end else begin
                            work[point][component] <= saturate(updated);
                            if (updated > MAX_POS || updated < MIN_NEG ||
                                (operation != HEADING && mac_overflow))
                                overflow <= 1;
                            case (operation)
                                POSITION_X: begin operation <= POSITION_Y; phase <= MAC_LAUNCH; end
                                POSITION_Y: begin operation <= HEADING; phase <= MAC_LAUNCH; end
                                HEADING: begin mac_request <= 0; phase <= ADVANCE_POINT; end
                                default: begin error <= 1; phase <= FAULT; end
                            endcase
                        end
                    end else if (wait_cycles == MAC_LIMIT) begin
                        error <= 1;
                        phase <= FAULT;
                    end else wait_cycles <= wait_cycles + 1'b1;
                end
                ADVANCE_POINT: begin
                    if (point == 8) phase <= PUBLISH;
                    else begin
                        point <= point + 1'b1;
                        cordic_request <= 1;
                        phase <= REQUEST_CORDIC;
                    end
                end
                PUBLISH: begin
                    for (integer i=0; i<9; i=i+1)
                        for (integer j=0; j<4; j=j+1)
                            propagated[i][j] <= work[i][j];
                    busy <= 0;
                    done <= 1;
                    phase <= IDLE;
                end
                FAULT: begin
                    error <= 1;
                end
                default: begin
                    error <= 1;
                    phase <= FAULT;
                end
            endcase
        end
    end
endmodule
