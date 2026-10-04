module v16PacketParser (
    input wire  clk,
    input wire  resetn,

    // Our parser is the receiver/Slave and the Ethernet/UDP is the Master that pushes data to us

    // AXI-Stream Input from UDP 
    input wire [7:0] s_AXIstream_tdata, // Actual data on conveyor belt
    input wire s_AXIstream_tvalid,  // Ethernet's signal for whether or not its data is valid
    input wire s_AXIstream_tlast,   // End of packet, high on same clock cycle as last packet byte
    output wire s_AXIstream_tready, // Signal if we're ready for data   

    // Parsed outputs 
    output reg [15:0] out_azimuth,
    output reg [15:0] out_distance,
    output reg [7:0] out_reflectivity,
    output reg out_data_valid
);

reg [10:0] byte_counter;

reg [7:0] distance_low;
reg [7:0] azimuth_low;


// State parameters 
localparam init = 3'b000;
localparam header = 3'b001;
localparam data = 3'b010;


// State transitions ???
// always @* begin
    
// end

// Synchronous update of state register
always @(posedge clk) begin

    if (!resetn)
        byte_counter <= 11'd0;

    else if (s_AXIstream_tvalid && s_AXIstream_tready) begin

        case(byte_counter)

            // Replace these byte numbers with the correct
            // offsets from the Velodyne manual. ***********************************

            11'd0:

            
            begin
                out_reflectivity <= s_AXIstream_tdata;
                out_data_valid <= 1'b1;
            end

        endcase

        if (s_AXIstream_tlast)
            byte_counter <= 11'd0;
        else
            byte_counter <= byte_counter + 11'd1;

    end

end

// Output logic
assign xxx = xxxx;

endmodule