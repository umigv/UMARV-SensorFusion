// Signed 16-bit fixed point -> IEEE-754 single precision, with scaling.
//
//   fout = (negate ? -1 : 1) * din * (SCALE_Q31 / 2^31) * (x2 ? 2 : 1)
//
// Default SCALE_Q31 is 0.002 * 2^31, i.e. din is in VLP-16 distance units
// (2 mm / LSB) and fout is in metres.  Mantissa is truncated (error < 1.2e-7
// relative, far below the sensor resolution).
//
// Latency: 3 clocks.
module pc_fix2float #(
    parameter int unsigned SCALE_Q31 = 4294967    // round(0.002 * 2^31)
) (
    input  logic               clk,
    input  logic signed [15:0] din,
    input  logic               x2,
    input  logic               negate,
    output logic [31:0]        fout
);

    localparam int SW = 23;          // bits needed for SCALE_Q31 (< 2^23)
    localparam int PW = 16 + SW;     // product width (39)

    // ---- stage 1: magnitude * scale --------------------------------------
    logic [15:0]   mag;
    logic [PW-1:0] prod_q;
    logic          sign_q1, x2_q1;

    assign mag = din[15] ? 16'(-din) : 16'(din);   // -32768 -> 32768, fits

    always_ff @(posedge clk) begin
        prod_q  <= mag * SW'(SCALE_Q31);
        sign_q1 <= din[15] ^ negate;
        x2_q1   <= x2;
    end

    // ---- stage 2: find leading one ---------------------------------------
    logic [5:0]    msb;
    logic [5:0]    msb_q;
    logic [PW-1:0] prod_q2;
    logic          sign_q2, x2_q2, zero_q2;

    always_comb begin
        msb = '0;
        for (int i = 0; i < PW; i++)
            if (prod_q[i]) msb = 6'(i);
    end

    always_ff @(posedge clk) begin
        msb_q   <= msb;
        prod_q2 <= prod_q;
        sign_q2 <= sign_q1;
        x2_q2   <= x2_q1;
        zero_q2 <= (prod_q == '0);
    end

    // ---- stage 3: normalise and pack -------------------------------------
    logic [PW-1:0] norm;
    logic [7:0]    exp;

    always_comb begin
        norm = prod_q2 << (6'(PW-1) - msb_q);          // leading one at bit PW-1
        exp  = 8'(127 - 31) + 8'(msb_q) + 8'(x2_q2);   // value = prod * 2^-31
    end

    always_ff @(posedge clk) begin
        if (zero_q2) fout <= 32'd0;
        else         fout <= {sign_q2, exp, norm[PW-2 -: 23]};
    end

endmodule
