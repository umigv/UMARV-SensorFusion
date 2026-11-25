
module int_to_fp32 #(
    parameter integer WIDTH = 28,
    parameter integer FRACTION_BITS = 0
) (
    input logic signed [WIDTH-1:0] int_in,
    output logic [31:0] fp_out
);
logic sign;
logic [WIDTH-1:0] magnitude;
logic [WIDTH-1:0] shifted;
logic [WIDTH-1:0] remainder;
logic [WIDTH-1:0] halfway;
logic [24:0] rounded;
integer msb_pos;
integer shift_amt;
logic round_up;
always_comb begin 
    sign = int_in[WIDTH-1];
    magnitude = sign ? (~int_in + 1'b1): int_in;
    fp_out = 32'b0;
    shifted = '0;
    remainder = '0;
    halfway = '0;
    rounded = 25'b0;
    round_up = 1'b0;
    msb_pos = 0;
    shift_amt = 0;
    if (magnitude!=0) begin
        for(int i = 0; i<WIDTH; i=i+1) begin
            if (magnitude[i])
            msb_pos = i;
        end 
        if(msb_pos>23) begin
            shift_amt=msb_pos-23;
            shifted = magnitude>>shift_amt;
            remainder = magnitude & ((WIDTH'(1)<<shift_amt)-1'b1);
            halfway = WIDTH'(1)<<(shift_amt-1);
            round_up = (remainder>halfway)||((remainder==halfway)&&shifted[0]);
        end
        else begin
            shifted = magnitude << (23-msb_pos);
        end
        rounded = {1'b0,shifted[23:0]} + round_up;
        if (rounded[24]) begin
            fp_out = {sign, 8'(msb_pos+128-FRACTION_BITS),23'b0};
        end
        else begin
            fp_out = {sign, 8'(msb_pos+127-FRACTION_BITS),rounded[22:0]};
        end 
    end 
end 
endmodule

