module v16PacketParser #(
    parameter integer HEADER_BYTES = 42  // 42: full Ethernet frame; 0: UDP payload only
)(
    input  wire        clk,
    input  wire        resetn,
    input  wire [7:0]  s_AXIstream_tdata,
    input  wire        s_AXIstream_tvalid,
    input  wire        s_AXIstream_tlast,
    output wire        s_AXIstream_tready,
    output reg  [15:0] out_azimuth,
    output reg  [15:0] out_distance,
    output reg  [7:0]  out_reflectivity,
    output reg         out_data_valid
);
    localparam integer BLOCK_START = HEADER_BYTES;
    localparam integer BLOCK_END   = HEADER_BYTES + 1200;

    reg [10:0] byte_counter;
    reg [6:0]  block_offset;  // 0..99, reset at every measurement block
    reg [1:0]  point_phase;   // 0: distance low, 1: distance high, 2: reflectivity
    reg [7:0]  distance_low;
    reg [7:0]  azimuth_low;
    reg        flag_first_ok;
    reg        block_valid;

    // No buffering or backpressure: one accepted byte per clock cycle.
    assign s_AXIstream_tready = 1'b1;

    always @(posedge clk) begin
        if (!resetn) begin
            byte_counter    <= 11'd0;
            block_offset    <= 7'd0;
            point_phase     <= 2'd0;
            distance_low    <= 8'd0;
            azimuth_low     <= 8'd0;
            flag_first_ok  <= 1'b0;
            block_valid    <= 1'b0;
            out_azimuth    <= 16'd0;
            out_distance   <= 16'd0;
            out_reflectivity <= 8'd0;
            out_data_valid <= 1'b0;
        end else begin
            // A pulse is asserted for one cycle per completed point.
            out_data_valid <= 1'b0;

            // An AXI-stream transfer happens ONLY when valid and ready are high.
            if (s_AXIstream_tvalid && s_AXIstream_tready) begin
                if ((byte_counter >= BLOCK_START) &&
                    (byte_counter <  BLOCK_END)) begin
                    case (block_offset)
                        7'd0: begin
                            flag_first_ok <= (s_AXIstream_tdata == 8'hFF);
                            block_valid   <= 1'b0;
                            point_phase   <= 2'd0;
                        end
                        7'd1: begin
                            // Bad block markers suppress all 32 point outputs.
                            block_valid <= flag_first_ok &&
                                           (s_AXIstream_tdata == 8'hEE);
                        end
                        7'd2: azimuth_low <= s_AXIstream_tdata;
                        7'd3: out_azimuth <= {s_AXIstream_tdata, azimuth_low}; // Use this format because VLP-16 sends azimuth in little endian format
                        default: begin
                            case (point_phase)
                                2'd0: begin
                                    distance_low <= s_AXIstream_tdata;
                                    point_phase  <= 2'd1;
                                end
                                2'd1: begin
                                    out_distance <= {s_AXIstream_tdata, distance_low};
                                    point_phase  <= 2'd2;
                                end
                                2'd2: begin
                                    out_reflectivity <= s_AXIstream_tdata;
                                    out_data_valid   <= block_valid;
                                    point_phase      <= 2'd0;
                                end
                                default: point_phase <= 2'd0;
                            endcase
                        end
                    endcase

                    if (block_offset == 7'd99)
                        block_offset <= 7'd0;
                    else
                        block_offset <= block_offset + 7'd1;
                end

                // tlast marks the packet boundary, whatever its actual length.
                if (s_AXIstream_tlast) begin
                    byte_counter <= 11'd0;
                    block_offset <= 7'd0;
                    point_phase  <= 2'd0;
                    block_valid  <= 1'b0;
                end else if (byte_counter < HEADER_BYTES + 1206) begin
                    byte_counter <= byte_counter + 11'd1;
                end
            end
        end
    end
endmodule
