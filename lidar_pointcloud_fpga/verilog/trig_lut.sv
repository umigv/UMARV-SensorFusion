`include "sys_defs.svh"

module trig_lut #(
    // there are only 360x16 = 5760 entries, but there must
    // be 8192 to support the 13-bit concatenated address space
    parameter DEPTH = 8192, 
    parameter INIT_FILE = "azimuth_sincos.mem"
) (
    input  logic clk,
    input  logic stall,
    input  DATA         [1:0] azimuth,
    input  ELEVATION    [1:0] laser_id,
    output DATA         [1:0] x_coeff,
    output DATA         [1:0] y_coeff,
    output DATA         [1:0] z_coeff
);

    // 9 bit wire for the truncated 0-359 degree value
    logic [1:0][8:0] az_degree;
    
    // Dividing by 100 using reciprocal multiplication
    assign az_degree[0] = 9'( ({16'b0, azimuth[0]} * 32'd655) >> 16 );
    assign az_degree[1] = 9'( ({16'b0, azimuth[1]} * 32'd655) >> 16 );
    
    // Declared [0:DEPTH-1] for standard $readmemh parsing
    // 48 bit lines so each azimuth + elevation indexes x, y, and z
    logic [47:0] rom [0:DEPTH-1]; 

    // Load the pre-computed trig values during synthesis
    initial begin
        $readmemh(INIT_FILE, rom);
    end

    // Standard synchronous ROM read
    // 13 bits = {4 (elevation), 9 (azimuth)}
    always_ff @(posedge clk) begin
        if(!stall) begin
            {x_coeff[0], y_coeff[0], z_coeff[0]} <= rom[{laser_id[0], az_degree[0]}];
            {x_coeff[1], y_coeff[1], z_coeff[1]} <= rom[{laser_id[1], az_degree[1]}];
        end
    end

endmodule
