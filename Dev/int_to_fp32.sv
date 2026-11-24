module int_to_fp32(
    input logic signed [27:0] int_in,
    output logic [31:0] fp_out
);
logic sign;
logic [27:0] magnitude;
logic [27:0] shifted;
logic [27:0] remainder;
logic [27:0] halfway;
logic [24:0] rounded;
integer msb_pos;
integer shift_amt;
integer i;
logic round_up;
always_comb begin 
    sign = int_in[27];
    magnitude = sign ? (~int_in + 28'd1): int_in;
    fp_out = 32'b0;
    shifted = 28'b0;
    remainder = 28'b0;
    halfway = 28'b0;
    rounded = 25'b0;
    round_up = 1'b0;
    msb_pos = 0;
    shift_amt = 0;
    if (magnitude!=0) begin
        for(i = 0; i<28; i=i+1) begin
            if (magnitude[i])
            msb_pos = i;
        end 
        if(msb_pos>23) begin
            shift_amt=msb_pos-23;
            shifted = magnitude>>shift_amt;
            remainder = magnitude & ((28'd1<<shift_amt)-1'b1);
            halfway = 28'd1<<(shift_amt-1);
            round_up = (remainder>halfway)||((remainder==halfway)&&shifted[0]);
        end
        else begin
            shifted = magnitude << (23-msb_pos);
        end
        rounded = {1'b0,shifted[23:0]} + round_up;
        if (rounded[24]) begin
            fp_out = {sign, 8'(msb_pos+128),23'b0};
        end
        else begin
            fp_out = {sign, 8'(msb_pos+127),rounded[22:0]};
        end 
    end 
end 
endmodule