// SystemVerilog Cholesky Downdate Module (5x5 Lower Triangular S, 5x1 Vector U)
// Algorithm: Rank-1 Downdate: S_new * S_new^T = S * S^T - u * u^T
//
// For each column k (0 to N-1):
//   1. Diagonal: S_kk_new = sqrt(S_kk^2 - u_k^2)
//   2. Rotation parameters:
//        c = S_kk_new / S_kk_old
//        h = u_k / S_kk_old
//   3. For each row j below diagonal (j = k+1 to N-1):
//        S_jk_new = (S_jk_old - h * u_j_old) / c
//        u_j_new  = c * u_j_old - h * S_jk_new
//
// Hardware primitives used:
//   - MAC (Multiply-Accumulate via MAC.sv)
//   - Carry Lookahead Adder / Subtractor (via cla.sv)
//   - External Division (with div_by_zero and overflow detection via division.sv)
//   - Multi-cycle Fixed-Point Square Root (SQRT module via SQRT.sv)

`timescale 1ns/1ps

module cholDownDate #(
    parameter int DATA_WIDTH = 32, // Total width of signed fixed-point word
    parameter int FRAC_WIDTH = 16, // Number of fractional bits (e.g., Q16.16)
    parameter int N          = 5   // Matrix dimension (5x5)
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // --- Loading Interface for Matrix S ---
    input  logic                          s_load_en,
    input  logic [$clog2(N)-1:0]          s_load_row,
    input  logic [$clog2(N)-1:0]          s_load_col,
    input  logic signed [DATA_WIDTH-1:0]  s_load_data,

    // --- Loading Interface for Vector u ---
    input  logic                          u_load_en,
    input  logic [$clog2(N)-1:0]          u_load_idx,
    input  logic signed [DATA_WIDTH-1:0]  u_load_data,

    // --- Control and Status ---
    input  logic                          start_downdate,
    output logic                          busy,
    output logic                          done,
    output logic                          error_indefinite, // High if (S_kk^2 - u_k^2) <= 0 (matrix not positive definite)
    output logic                          sqrt_by_neg,      // High if SQRT encountered a negative radicand
    output logic                          negative_input,   // Mirror of sqrt_by_neg (SQRT fault flag)
    output logic                          error_divide,     // High if divisor reports error (div_by_zero or overflow)

    // --- Readout Interface for Updated S and U ---
    input  logic [$clog2(N)-1:0]          s_read_row,
    input  logic [$clog2(N)-1:0]          s_read_col,
    output logic signed [DATA_WIDTH-1:0]  s_read_data,

    input  logic [$clog2(N)-1:0]          u_read_idx,
    output logic signed [DATA_WIDTH-1:0]  u_read_data
);

    // =========================================================================
    // Parameters and Signal Sizing
    // =========================================================================
    localparam int DOUBLE_WIDTH = DATA_WIDTH * 2;

    assign negative_input = sqrt_by_neg;

    // =========================================================================
    // Internal Registers for S Matrix and U Vector
    // =========================================================================
    logic signed [DATA_WIDTH-1:0] S_reg [0:N-1][0:N-1];
    logic signed [DATA_WIDTH-1:0] u_reg [0:N-1];

    assign s_read_data = S_reg[s_read_row][s_read_col];
    assign u_read_data = u_reg[u_read_idx];

    // =========================================================================
    // Algorithm State Machine
    // =========================================================================
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_DIAG_MAC,     // Compute S_kk^2 and u_k^2
        ST_DIAG_SUB,     // Compute diff = S_kk^2 - u_k^2 (LUT sub) & start SQRT
        ST_DIAG_SQRT,    // S_kk_new = sqrt(diff)
        ST_CALC_C,       // c = S_kk_new / S_kk_old
        ST_CALC_H,       // h = u_k / S_kk_old
        ST_ROW_CHECK,    // Check if j < N
        ST_ROW_MAC_HU,   // h * u_j (MAC)
        ST_ROW_SUB,      // num = S_jk_old - (h * u_j) (LUT sub)
        ST_ROW_DIV_S,    // S_jk_new = num / c
        ST_ROW_MAC_U,    // u_j_new = c * u_j_old - h * S_jk_new (MAC)
        ST_ROW_NEXT,     // j++
        ST_COL_NEXT,     // k++
        ST_DONE,
        ST_ERROR
    } state_t;

    state_t state;

    // Loop indices
    logic [$clog2(N):0] k; // Column index (0 to N-1)
    logic [$clog2(N):0] j; // Row index (k+1 to N-1)

    // Temporary algorithm variables (signed fixed-point)
    logic signed [DATA_WIDTH-1:0] s_kk_old;
    logic signed [DATA_WIDTH-1:0] s_kk_new;
    logic signed [DATA_WIDTH-1:0] u_k_val;
    logic signed [DATA_WIDTH-1:0] c_val;
    logic signed [DATA_WIDTH-1:0] h_val;
    logic signed [DATA_WIDTH-1:0] s_jk_old;
    logic signed [DATA_WIDTH-1:0] s_jk_new;
    logic signed [DATA_WIDTH-1:0] u_j_old;
    logic signed [DATA_WIDTH-1:0] u_j_new;
    logic signed [DATA_WIDTH-1:0] h_u_prod;
    logic signed [DATA_WIDTH-1:0] prod1_reg;
    logic [1:0]                   mac_u_step;

    // Intermediate DOUBLE_WIDTH storage for products and differences
    logic signed [DOUBLE_WIDTH-1:0] diag_diff;
    logic signed [DOUBLE_WIDTH-1:0] diag_s_sq;
    logic signed [DOUBLE_WIDTH-1:0] diag_u_sq;
    logic signed [DOUBLE_WIDTH-1:0] diag_diff_cla;

    // =========================================================================
    // Math Primitive: Multiply-Accumulate (MAC.sv)
    // =========================================================================
    logic                         mac_start;
    logic signed [DATA_WIDTH-1:0] mac_a, mac_b, mac_c;
    logic signed [DATA_WIDTH-1:0] mac_out;
    logic                         mac_busy;
    logic                         mac_done;
    logic                         mac_overflow;
    logic                         mac_started;

    MAC #(
        .WIDTH (DATA_WIDTH),
        .FRAC  (FRAC_WIDTH)
    ) u_mac (
        .clk      (clk),
        .resetn   (rst_n),
        .start    (mac_start),
        .a        (mac_a),
        .b        (mac_b),
        .c        (mac_c),
        .result   (mac_out),
        .busy     (mac_busy),
        .done     (mac_done),
        .overflow (mac_overflow)
    );

    // =========================================================================
    // Math Primitive: Carry Lookahead Adder / Subtractor (cla.sv)
    // =========================================================================
    logic signed [DATA_WIDTH-1:0] lut_add_a, lut_add_b;
    logic                         lut_sub_mode; // 0: add, 1: sub
    logic signed [DATA_WIDTH-1:0] lut_add_out;
    wire  [DATA_WIDTH-1:0]        cla_b_in = lut_sub_mode ? ~lut_add_b : lut_add_b;
    wire                          cla_cin  = lut_sub_mode;

    cla #(
        .WIDTH (DATA_WIDTH)
    ) u_cla (
        .a        (lut_add_a),
        .b        (cla_b_in),
        .cin      (cla_cin),
        .sum      (lut_add_out),
        .cout     (),
        .overflow ()
    );

    // 64-bit CLA for diagonal difference (S_kk^2 - u_k^2)
    cla #(
        .WIDTH (DOUBLE_WIDTH)
    ) u_cla_diag (
        .a        (diag_s_sq),
        .b        (~diag_u_sq),
        .cin      (1'b1),
        .sum      (diag_diff_cla),
        .cout     (),
        .overflow ()
    );

    // =========================================================================
    // Math Primitive: External Division Module
    // =========================================================================
    logic                         div_start;
    logic signed [DATA_WIDTH-1:0] div_a;
    logic signed [DATA_WIDTH-1:0] div_b;
    logic signed [DATA_WIDTH-1:0] div_quot;
    logic                         div_busy;
    logic                         div_done;
    logic                         div_by_zero;
    logic                         div_overflow;
    logic                         div_started;

    division #(
        .WIDTH (DATA_WIDTH),
        .FRAC  (FRAC_WIDTH)
    ) u_div (
        .clk         (clk),
        .resetn      (rst_n),
        .start       (div_start),
        .a           (div_a),
        .b           (div_b),
        .c           (div_quot),
        .busy        (div_busy),
        .done        (div_done),
        .div_by_zero (div_by_zero),
        .overflow    (div_overflow)
    );

    // =========================================================================
    // Math Primitive: Fixed-Point Square Root
    // Preserves full accuracy: diag_diff has 2*FRAC_WIDTH fractional bits.
    // SQRT with FRAC=0 computes floor(sqrt(diag_diff)), naturally producing
    // an output with FRAC_WIDTH fractional bits without discarding precision.
    // =========================================================================
    logic                           sqrt_start;
    logic signed [DOUBLE_WIDTH-1:0] sqrt_in;
    logic signed [DOUBLE_WIDTH-1:0] sqrt_out;
    logic                           sqrt_busy;
    logic                           sqrt_done;
    logic                           sqrt_negative;

    SQRT #(
        .WIDTH (DOUBLE_WIDTH),
        .FRAC  (0)
    ) u_sqrt (
        .clk            (clk),
        .resetn         (rst_n),
        .start          (sqrt_start),
        .a              (sqrt_in),
        .result         (sqrt_out),
        .busy           (sqrt_busy),
        .done           (sqrt_done),
        .negative_input (sqrt_negative)
    );

    // =========================================================================
    // Main FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= ST_IDLE;
            k                <= '0;
            j                <= '0;
            div_start        <= 1'b0;
            sqrt_start       <= 1'b0;
            mac_start        <= 1'b0;
            mac_started      <= 1'b0;
            mac_u_step       <= 2'd0;
            prod1_reg        <= '0;
            diag_s_sq        <= '0;
            diag_u_sq        <= '0;
            diag_diff        <= '0;
            error_indefinite <= 1'b0;
            sqrt_by_neg      <= 1'b0;
            error_divide     <= 1'b0;
            div_started      <= 1'b0;
            done             <= 1'b0;
            busy             <= 1'b0;

            // Reset Matrix and Vector
            for (int r = 0; r < N; r++) begin
                for (int c = 0; c < N; c++) begin
                    S_reg[r][c] <= '0;
                end
                u_reg[r] <= '0;
            end
        end else begin
            // Default pulse signals
            div_start  <= 1'b0;
            sqrt_start <= 1'b0;
            mac_start  <= 1'b0;

            // Allow external loading when IDLE
            if (state == ST_IDLE) begin
                if (s_load_en) begin
                    S_reg[s_load_row][s_load_col] <= s_load_data;
                end
                if (u_load_en) begin
                    u_reg[u_load_idx] <= u_load_data;
                end
            end

            case (state)
                ST_IDLE: begin
                    done <= 1'b0;
                    if (start_downdate) begin
                        busy             <= 1'b1;
                        error_indefinite <= 1'b0;
                        sqrt_by_neg      <= 1'b0;
                        error_divide     <= 1'b0;
                        div_started      <= 1'b0;
                        mac_started      <= 1'b0;
                        mac_u_step       <= 2'd0;
                        k                <= '0;
                        state            <= ST_DIAG_MAC;
                    end
                end

                // --- Column Diagonal Step ---
                ST_DIAG_MAC: begin
                    s_kk_old  <= S_reg[k][k];
                    u_k_val   <= u_reg[k];
                    // Compute S_kk^2 and u_k^2 with DOUBLE_WIDTH precision (2*FRAC_WIDTH fractional bits)
                    diag_s_sq <= DOUBLE_WIDTH'(S_reg[k][k]) * DOUBLE_WIDTH'(S_reg[k][k]);
                    diag_u_sq <= DOUBLE_WIDTH'(u_reg[k])    * DOUBLE_WIDTH'(u_reg[k]);
                    state     <= ST_DIAG_SUB;
                end

                ST_DIAG_SUB: begin
                    // Feed full-precision product difference to SQRT (computed via 64-bit CLA)
                    diag_diff  <= diag_diff_cla;
                    sqrt_in    <= diag_diff_cla;
                    sqrt_start <= 1'b1;
                    state      <= ST_DIAG_SQRT;
                end

                ST_DIAG_SQRT: begin
                    if (sqrt_done) begin
                        if (sqrt_negative) begin
                            error_indefinite <= 1'b1;
                            sqrt_by_neg      <= 1'b1;
                            state            <= ST_ERROR;
                        end else if (sqrt_out == '0) begin
                            // Radicand was zero: downdated matrix is singular (not positive definite)
                            error_indefinite <= 1'b1;
                            state            <= ST_ERROR;
                        end else begin
                            s_kk_new        <= DATA_WIDTH'(sqrt_out);
                            S_reg[k][k]     <= DATA_WIDTH'(sqrt_out); // Update diagonal S_kk in reg
                            // Kick off division: c = S_kk_new / S_kk_old
                            div_a           <= DATA_WIDTH'(sqrt_out);
                            div_b           <= s_kk_old;
                            div_start       <= 1'b1;
                            state           <= ST_CALC_C;
                        end
                    end
                end

                ST_CALC_C: begin
                    if (div_done) begin
                        if (div_by_zero || div_overflow) begin
                            error_divide <= 1'b1;
                            state        <= ST_ERROR;
                        end else begin
                            c_val     <= div_quot;
                            // Kick off division: h = u_k / S_kk_old
                            div_a     <= u_k_val;
                            div_b     <= s_kk_old;
                            div_start <= 1'b1;
                            state     <= ST_CALC_H;
                        end
                    end
                end

                ST_CALC_H: begin
                    if (div_done) begin
                        if (div_by_zero || div_overflow) begin
                            error_divide <= 1'b1;
                            state        <= ST_ERROR;
                        end else begin
                            h_val <= div_quot;
                            j     <= k + 1; // First row below diagonal
                            state <= ST_ROW_CHECK;
                        end
                    end
                end

                // --- Rows Below Diagonal Loop ---
                ST_ROW_CHECK: begin
                    if (j < N) begin
                        s_jk_old <= S_reg[j][k];
                        u_j_old  <= u_reg[j];
                        state    <= ST_ROW_MAC_HU;
                    end else begin
                        state    <= ST_COL_NEXT;
                    end
                end

                // Step A: h * u_j using MAC
                ST_ROW_MAC_HU: begin
                    if (!mac_started) begin
                        if (!mac_busy) begin
                            mac_a       <= h_val;
                            mac_b       <= u_j_old;
                            mac_c       <= '0;
                            mac_start   <= 1'b1;
                            mac_started <= 1'b1;
                        end
                    end else if (mac_done) begin
                        mac_started  <= 1'b0;
                        h_u_prod     <= mac_out;
                        state        <= ST_ROW_SUB;
                    end
                end

                // Step B: S_jk_old - (h * u_j) using CLA subtractor
                ST_ROW_SUB: begin
                    lut_add_a    <= s_jk_old;
                    lut_add_b    <= h_u_prod;
                    lut_sub_mode <= 1'b1; // subtract
                    div_started  <= 1'b0;
                    state        <= ST_ROW_DIV_S;
                end

                // Step C: S_jk_new = (S_jk_old - h * u_j) / c
                ST_ROW_DIV_S: begin
                    if (!div_started) begin
                        if (!div_busy) begin
                            div_a       <= lut_add_out;
                            div_b       <= c_val;
                            div_start   <= 1'b1;
                            div_started <= 1'b1;
                        end
                    end else if (div_done) begin
                        div_started <= 1'b0;
                        if (div_by_zero || div_overflow) begin
                            error_divide <= 1'b1;
                            state        <= ST_ERROR;
                        end else begin
                            s_jk_new    <= div_quot;
                            S_reg[j][k] <= div_quot; // Store updated S_jk
                            mac_started <= 1'b0;
                            mac_u_step  <= 2'd0;
                            state       <= ST_ROW_MAC_U;
                        end
                    end
                end

                // Step D: u_j_new = c * u_j_old - h * S_jk_new (using MAC & CLA)
                ST_ROW_MAC_U: begin
                    case (mac_u_step)
                        2'd0: begin
                            // Sub-step 1: compute c * u_j_old using MAC
                            if (!mac_started) begin
                                if (!mac_busy) begin
                                    mac_a       <= c_val;
                                    mac_b       <= u_j_old;
                                    mac_c       <= '0;
                                    mac_start   <= 1'b1;
                                    mac_started <= 1'b1;
                                end
                            end else if (mac_done) begin
                                mac_started <= 1'b0;
                                prod1_reg   <= mac_out;
                                mac_u_step  <= 2'd1;
                            end
                        end

                        2'd1: begin
                            // Sub-step 2: compute h * s_jk_new using MAC
                            if (!mac_started) begin
                                if (!mac_busy) begin
                                    mac_a       <= h_val;
                                    mac_b       <= s_jk_new;
                                    mac_c       <= '0;
                                    mac_start   <= 1'b1;
                                    mac_started <= 1'b1;
                                end
                            end else if (mac_done) begin
                                mac_started  <= 1'b0;
                                // Sub-step 3: Setup CLA subtractor: prod1 - prod2
                                lut_add_a    <= prod1_reg;
                                lut_add_b    <= mac_out;
                                lut_sub_mode <= 1'b1; // subtract
                                mac_u_step   <= 2'd2;
                            end
                        end

                        2'd2: begin
                            // Sub-step 4: Store CLA difference: u_j_new = prod1 - prod2
                            u_j_new    <= lut_add_out;
                            u_reg[j]   <= lut_add_out; // Store updated u_j
                            mac_u_step <= 2'd0;
                            state      <= ST_ROW_NEXT;
                        end

                        default: mac_u_step <= 2'd0;
                    endcase
                end

                ST_ROW_NEXT: begin
                    j     <= j + 1;
                    state <= ST_ROW_CHECK;
                end

                ST_COL_NEXT: begin
                    if (k + 1 < N) begin
                        k     <= k + 1;
                        state <= ST_DIAG_MAC;
                    end else begin
                        state <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                ST_ERROR: begin
                    busy  <= 1'b0;
                    done  <= 1'b1; // Completed with error flag
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
