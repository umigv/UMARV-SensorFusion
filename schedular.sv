module schedular (
    input  wire        clk,
    input  wire        resetn,

    output logic processor_on,
    
);


always_ff @(posedge clk) begin : FIFO
    
end

endmodule