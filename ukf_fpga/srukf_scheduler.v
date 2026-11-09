`timescale 1ns/1ps
// Sits between the sensor FIFO and the shared datapath (MAC, CORDIC, SQRT).
// Pops one measurement at a time, predicts the state forward to that
// measurement's timestamp, applies the matching correction, and publishes.
// If the FIFO is empty when a tick arrives it runs a predict-only step so the
// estimate keeps updating at the tick rate (dead reckoning).
//
// Each S-state from the state diagram is a short sequence of micro-ops. A
// micro-op fires one datapath unit and waits for its done. The datapath
// decodes uop_id to know which RAM/register addresses and muxes to drive;
// this module only sequences.

module srukf_scheduler #(
    parameter TICK_DT = 32'd1000      // dt in microseconds for a predict-only tick
)(
    input  wire        clk,
    input  wire        resetn,

    // sensor FIFO (one entry per measurement, pushed by the UART/Ethernet receivers)
    input  wire        fifo_empty,
    input  wire        fifo_type,     // 0 = GPS (x,y), 1 = ODOM (v,theta,omega)
    input  wire [31:0] fifo_ts,       // measurement timestamp, microseconds
    output reg         fifo_rd,       // 1-cycle pop; payload is latched by the datapath on this pulse

    input  wire        tick_1khz,     // predict-only trigger when the FIFO is empty
    input  wire        init_done,     // state params RAM holds x0, S0, weights

    // datapath unit handshakes
    output reg         start_mac,    input wire done_mac,
    output reg         start_cordic, input wire done_cordic,
    output reg         start_sqrt,   input wire done_sqrt,

    // micro-op identity for the datapath decoder: {state, step}
    output reg  [3:0]  uop_state,
    output reg  [3:0]  uop_step,
    output reg         uop_valid,     // high while a micro-op is in flight

    // dt for this prediction, written to state params RAM before S1
    output reg  [31:0] dt,
    output reg         dt_wr,

    output reg         meas_select,   // which measurement model the correction uses
    output reg         estimate_valid,
    output reg         busy,
    output reg         ts_reject      // sticky: a FIFO entry was older than the filter time
);

    // ---- states (S0..S10 from the diagram, plus cold start) ----
    localparam [3:0]
        WAIT_FIX = 4'd0,
        S0_IDLE  = 4'd1,
        S1_SIGMA = 4'd2,   // sigma points: x +/- gamma*S columns  (MAC)
        S2_PROP  = 4'd3,   // cos/sin theta (CORDIC), v*dt (MAC), store
        S3_PRED  = 4'd4,   // x' = x + cos*vdt, y' = y + sin*vdt, th' = th + w*dt (MAC)
        S4_MEAN  = 4'd5,   // weighted sums of sigma points (MAC adder tree)
        S5_XHAT  = 4'd6,   // predicted mean x_hat = a + b*W_i (MAC)
        S6_DEV   = 4'd7,   // d_i = X_i - x_hat (MAC subtract)
        S7_MEAS  = 4'd8,   // z_i = h(X_i), z_hat, S_z, P_xz (MAC, SQRT)
        S8_GAIN  = 4'd9,   // K via triangular solves (MAC)
        S9_UPD   = 4'd10,  // x += K*(z - z_hat), S downdate (MAC, SQRT)
        S10_OUT  = 4'd11;

    // ---- datapath unit codes ----
    localparam [1:0] U_NONE = 2'd0, U_MAC = 2'd1, U_CORDIC = 2'd2, U_SQRT = 2'd3;

    // ---- micro-op table: which unit each (state, step) fires ----
    // Steps for S1-S6 follow pages 6-7 of the design doc. S7-S9 are placeholders
    // with a plausible unit sequence; the doc has not detailed them yet.
    function [1:0] uop_unit(input [3:0] s, input [3:0] i);
        case (s)
            S1_SIGMA: uop_unit = U_MAC;                                   // 1 step
            S2_PROP:  uop_unit = (i==0) ? U_CORDIC : (i==1) ? U_MAC : U_NONE; // 0: sin/cos  1: v*dt  2: store (move)
            S3_PRED:  uop_unit = U_MAC;                                   // 0: x'  1: y'  2: theta'
            S4_MEAN:  uop_unit = (i==1) ? U_NONE : U_MAC;                 // 0: sum x',y',th'  1: store (move)  2: W0 term
            S5_XHAT:  uop_unit = U_MAC;                                   // 1 step
            S6_DEV:   uop_unit = U_MAC;                                   // 1 step
            S7_MEAS:  uop_unit = (i==3) ? U_SQRT : U_MAC;                 // 0: h(X)  1: z_hat  2: P_xz  3: S_z
            S8_GAIN:  uop_unit = U_MAC;                                   // 0: fwd solve  1: back solve
            S9_UPD:   uop_unit = (i==2) ? U_SQRT : U_MAC;                 // 0: innovation  1: x+=K*r  2: S downdate
            default:  uop_unit = U_NONE;
        endcase
    endfunction

    function [3:0] uop_count(input [3:0] s);
        case (s)
            S1_SIGMA: uop_count = 4'd1;
            S2_PROP:  uop_count = 4'd3;
            S3_PRED:  uop_count = 4'd3;
            S4_MEAN:  uop_count = 4'd3;
            S5_XHAT:  uop_count = 4'd1;
            S6_DEV:   uop_count = 4'd1;
            S7_MEAS:  uop_count = 4'd4;
            S8_GAIN:  uop_count = 4'd2;
            S9_UPD:   uop_count = 4'd3;
            default:  uop_count = 4'd0;
        endcase
    endfunction

    // S2 step 2 and S4 step 1 are pure register moves with no unit; treat as done next cycle.
    function uop_is_move(input [3:0] s, input [3:0] i);
        uop_is_move = (s==S2_PROP && i==2) || (s==S4_MEAN && i==1);
    endfunction

    reg [3:0]  state, next_state;
    reg [3:0]  step;
    reg        has_meas;       // this pass has a correction to run
    reg [31:0] filter_ts;      // time the state estimate currently represents
    reg        uop_done;       // the current micro-op's unit reported done

    wire [1:0] cur_unit = uop_unit(state, step);
    wire       last_step = (step == uop_count(state) - 1);

    always @(*) begin
        case (cur_unit)
            U_MAC:    uop_done = done_mac;
            U_CORDIC: uop_done = done_cordic;
            U_SQRT:   uop_done = done_sqrt;
            default:  uop_done = uop_is_move(state, step);
        endcase
    end

    // ---- next-state logic ----
    // Math states advance when their last micro-op finishes; see the step
    // counter below for advancing within a state.
    always @(*) begin
        next_state = state;
        case (state)
            WAIT_FIX: if (init_done) next_state = S0_IDLE;
            S0_IDLE:  if (!fifo_empty || tick_1khz) next_state = S1_SIGMA;
            S1_SIGMA: if (uop_done && last_step) next_state = S2_PROP;
            S2_PROP:  if (uop_done && last_step) next_state = S3_PRED;
            S3_PRED:  if (uop_done && last_step) next_state = S4_MEAN;
            S4_MEAN:  if (uop_done && last_step) next_state = S5_XHAT;
            S5_XHAT:  if (uop_done && last_step) next_state = S6_DEV;
            S6_DEV:   if (uop_done && last_step) next_state = has_meas ? S7_MEAS : S10_OUT;
            S7_MEAS:  if (uop_done && last_step) next_state = S8_GAIN;
            S8_GAIN:  if (uop_done && last_step) next_state = S9_UPD;
            S9_UPD:   if (uop_done && last_step) next_state = S10_OUT;
            S10_OUT:  next_state = S0_IDLE;
            default:  next_state = WAIT_FIX;
        endcase
    end

    wire entering = (next_state != state);
    wire [31:0] ts_delta = fifo_ts - filter_ts;

    always @(posedge clk) begin
        if (!resetn) begin
            state <= WAIT_FIX; step <= 4'd0;
            start_mac <= 1'b0; start_cordic <= 1'b0; start_sqrt <= 1'b0;
            uop_state <= 4'd0; uop_step <= 4'd0; uop_valid <= 1'b0;
            fifo_rd <= 1'b0; dt <= 32'd0; dt_wr <= 1'b0;
            meas_select <= 1'b0; has_meas <= 1'b0; filter_ts <= 32'd0;
            estimate_valid <= 1'b0; busy <= 1'b0; ts_reject <= 1'b0;
        end else begin
            state <= next_state;
            start_mac <= 1'b0; start_cordic <= 1'b0; start_sqrt <= 1'b0;
            fifo_rd <= 1'b0; dt_wr <= 1'b0; estimate_valid <= 1'b0;

            // ---- leaving IDLE: pop the FIFO (or dead-reckon), set dt ----
            if (state == S0_IDLE && next_state == S1_SIGMA) begin
                busy <= 1'b1;
                if (!fifo_empty) begin
                    fifo_rd     <= 1'b1;
                    has_meas    <= 1'b1;
                    meas_select <= fifo_type;
                    // an entry older than the filter time gets dt = 0 (no predict, still corrects)
                    if (ts_delta[31]) begin dt <= 32'd0; ts_reject <= 1'b1; end
                    else begin dt <= ts_delta; filter_ts <= fifo_ts; end
                end else begin
                    has_meas  <= 1'b0;
                    dt        <= TICK_DT;
                    filter_ts <= filter_ts + TICK_DT;
                end
                dt_wr <= 1'b1;
            end

            // ---- micro-op stepping ----
            if (entering) begin
                step <= 4'd0;
            end else if (uop_valid && uop_done && !last_step) begin
                step <= step + 4'd1;
            end

            // fire the unit for the micro-op being entered (new state or next step)
            if ((entering && uop_count(next_state) != 0) ||
                (!entering && uop_valid && uop_done && !last_step)) begin
                uop_state <= entering ? next_state : state;
                uop_step  <= entering ? 4'd0 : step + 4'd1;
                uop_valid <= 1'b1;
                case (uop_unit(entering ? next_state : state, entering ? 4'd0 : step + 4'd1))
                    U_MAC:    start_mac    <= 1'b1;
                    U_CORDIC: start_cordic <= 1'b1;
                    U_SQRT:   start_sqrt   <= 1'b1;
                    default: ;
                endcase
            end else if (entering || (uop_done && last_step)) begin
                uop_valid <= 1'b0;
            end

            if (entering && next_state == S10_OUT) begin
                estimate_valid <= 1'b1;
                busy <= 1'b0;
            end
        end
    end

endmodule
