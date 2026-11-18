`include "sys_defs.svh"
module stage_t (
    input clk,
    input rst_n,
    input stall,
    input INGRESS_PACKET data_in,
    output TRANSFORM_PACKET t2f_pipe
);
// NOTE: This stage is 2 clock cycles, since the BRAM memory read is 1 cycle
TRANSFORM_PACKET next_t2f_pipe;

DATA [1:0] x_coeff;
DATA [1:0] y_coeff;
DATA [1:0] z_coeff;

trig_lut lut(
    .clk,
    .stall,
    .azimuth(data_in.azimuth),
    .laser_id(data_in.laser_id),
    .x_coeff,
    .y_coeff,
    .z_coeff
);
// Registers to hold the data (lut takes 1 cycle to read)
logic   [1:0] valid_q;
DATA    [1:0] distance_q;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        valid_q       <= '0;
        distance_q[0] <= '0;
        distance_q[1] <= '0;
    end else if (!stall) begin
        valid_q       <= data_in.valid;
        distance_q[0] <= data_in.distance[0];
        distance_q[1] <= data_in.distance[1];
    end
end

always_comb begin
    next_t2f_pipe.valid = valid_q;
    for(int i = 0; i < 2; i++) begin
        // 1. Declare variables cleanly within loop scope
        logic signed [31:0] math_x;
        logic signed [31:0] math_y;
        logic signed [31:0] math_z;

        // 2. Perform operational assignments dynamically
        math_x = signed'(32'(distance_q[i])) * signed'(32'(x_coeff[i]));
        math_y = signed'(32'(distance_q[i])) * signed'(32'(y_coeff[i]));
        math_z = signed'(32'(distance_q[i])) * signed'(32'(z_coeff[i]));

        // Shift and cast back down to 16 bits
        next_t2f_pipe.x[i] = DATA'(math_x >>> 15);
        next_t2f_pipe.y[i] = DATA'(math_y >>> 15);
        next_t2f_pipe.z[i] = DATA'(math_z >>> 15);
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin // Reset asserted
        t2f_pipe <= '0;
    end else if (!stall) begin
        t2f_pipe <= next_t2f_pipe;
    end
end

endmodule
