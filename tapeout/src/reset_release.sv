// Brought over unchanged from the EECS 427 final project (FINAL/verilog/accelerator/reset_release.sv).
// Asynchronous assertion, two local rising edges before reset release.
module reset_release(input logic clk, arst, output logic rst);
    (* ASYNC_REG = "TRUE" *) logic [1:0] release_pipe;
    always_ff @(posedge clk or posedge arst) begin
        if (arst) release_pipe <= 2'b11;
        else release_pipe <= {release_pipe[0],1'b0};
    end
    assign rst = release_pipe[1];
endmodule
