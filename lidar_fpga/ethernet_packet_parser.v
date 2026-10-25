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

// Intermediate wires
wire [3:0] block_number; // Which block we're on: 0-11
wire [6:0] block_offset; // What byte are we on in that block: 0-99

assign block_number = (byte_counter - 42) / 100;
assign block_offset = (byte_counter - 42) % 100;

// Synchronous update of state register
always @(posedge clk) begin
    if (!resetn)
        byte_counter <= 11'd0;

    else if (s_AXIstream_tvalid && s_AXIstream_tready) begin

        if (byte_counter < 42) begin
            // Ethernet/IP/UDP header
        end
        else if (byte_counter < 1242) begin
            // Velodyne data blocks
        end
        else if (byte_counter < 1246) begin
            // Timestamp
        end
        else begin
            // Factory bytes
        end

        if (s_AXIstream_tlast) // Reset our byte counter if we get flag that we're on last byte
            byte_counter <= 11'd0;
        else
            byte_counter <= byte_counter + 11'd1; // Otherwise, increment

    end
end

// Output logic
assign xxx = xxxx;

endmodule