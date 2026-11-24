// MII transmitter: bytes in from an asynchronous FIFO, 4-bit nibbles out to the PHY.
//
// Everything here is in the PHY's transmit clock domain (TX_CLK, 25 MHz at 100 Mbit/s).
// It reads the read side of an async_fifo whose words are {bad, last, byte}:
//   byte   one byte of the frame, starting with the destination address
//   last   this is the final byte of the frame
//   bad    with last: send a wrong checksum, so the receiver throws the frame away
//
// For each frame it sends the preamble and start marker, the bytes (low nibble first), and the
// 4-byte checksum (CRC-32), then keeps the line idle for 12 byte times.
//
// The FIFO must be kept supplied faster than one byte every two clocks. If it runs dry in the
// middle of a frame, the frame is cut short (the receiver sees a damaged frame), the rest of it
// is read and discarded, and underrun pulses.
module mii_send (
    input  logic       TX_CLK,
    input  logic       TX_RST,          // active high, asserted asynchronously, released in step with TX_CLK

    // read side of the FIFO (async_fifo: data arrives one clock after rd_en, with rd_valid)
    output logic       rd_en,
    input  logic [9:0] rd_data,
    input  logic       rd_valid,
    input  logic       rd_empty,

    output logic       TX_EN,
    output logic [3:0] TXD,

    output logic       frame_sent,      // one clock: a complete frame went out
    output logic       underrun         // one clock: a frame was cut short
);

    localparam [2:0] T_IDLE = 3'd0, T_PRE = 3'd1, T_DATA = 3'd2, T_FCS = 3'd3, T_IFG = 3'd4, T_FLUSH = 3'd5;
    logic [2:0] st;

    logic [4:0] cnt;                // nibble or gap counter
    logic       phase;              // 0: low nibble, 1: high nibble
    logic [9:0] cur;                // the byte being sent
    logic       cur_ok;             // cur holds a byte
    logic       asked;              // a byte was requested last clock

    // Ethernet CRC-32, four bits per clock, least significant bit first
    function automatic [31:0] crc_bit;
        input [31:0] c;
        input        d;
        begin
            crc_bit = (c[0] ^ d) ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
        end
    endfunction
    function automatic [31:0] crc_nibble;
        input [31:0] c;
        input [3:0]  d;
        begin
            crc_nibble = crc_bit(crc_bit(crc_bit(crc_bit(c, d[0]), d[1]), d[2]), d[3]);
        end
    endfunction
    logic [31:0] crc;
    wire  [31:0] fcs = cur[9] ? crc : ~crc;        // a 'bad' frame gets the checksum un-inverted, which is wrong
    logic [31:0] fcs_sr;

    wire [3:0] nib = phase ? cur[7:4] : cur[3:0];

    always_comb begin
        rd_en = 1'b0;
        case (st)
            T_IDLE:  rd_en = !rd_empty;                                 // fetch the first byte
            T_DATA:  rd_en = !phase && !cur[8] && !rd_empty;            // fetch the next one while the low nibble goes out
            T_FLUSH: rd_en = !rd_empty && !asked;
            default: rd_en = 1'b0;
        endcase
    end

    always_ff @(posedge TX_CLK or posedge TX_RST) begin
        if (TX_RST) begin
            st <= T_IDLE; cnt <= 5'd0; phase <= 1'b0; cur <= 10'd0; cur_ok <= 1'b0; asked <= 1'b0;
            crc <= 32'hFFFFFFFF; fcs_sr <= 32'd0;
            TX_EN <= 1'b0; TXD <= 4'h0; frame_sent <= 1'b0; underrun <= 1'b0;
        end else begin
            frame_sent <= 1'b0;
            underrun   <= 1'b0;
            asked      <= rd_en;

            case (st)
                T_IDLE: begin
                    TX_EN <= 1'b0; TXD <= 4'h0;
                    if (rd_en) begin
                        cnt <= 5'd0; cur_ok <= 1'b0; crc <= 32'hFFFFFFFF;
                        st <= T_PRE;
                    end
                end

                // 15 nibbles of 5, then D. The first byte arrives from the FIFO meanwhile.
                T_PRE: begin
                    TX_EN <= 1'b1;
                    TXD   <= (cnt == 5'd15) ? 4'hD : 4'h5;
                    cnt   <= cnt + 5'd1;
                    if (rd_valid) begin cur <= rd_data; cur_ok <= 1'b1; end
                    if (cnt == 5'd15) begin phase <= 1'b0; st <= T_DATA; end
                end

                T_DATA: begin
                    TXD   <= nib;
                    crc   <= crc_nibble(crc, nib);
                    phase <= !phase;
                    if (!phase) begin
                        // low nibble. The next byte must be on its way unless this is the last.
                        if (!cur[8] && rd_empty) begin
                            underrun <= 1'b1;
                            TX_EN <= 1'b0; TXD <= 4'h0;
                            st <= T_FLUSH;
                        end
                    end else begin
                        // high nibble: the byte is done
                        if (cur[8]) begin
                            fcs_sr <= cur[9] ? crc_nibble(crc, nib) : ~crc_nibble(crc, nib);
                            cnt <= 5'd0;
                            st  <= T_FCS;
                        end else begin
                            cur <= rd_data;         // requested during the low nibble, valid now
                        end
                    end
                end

                T_FCS: begin
                    TXD    <= fcs_sr[3:0];
                    fcs_sr <= {4'h0, fcs_sr[31:4]};
                    cnt    <= cnt + 5'd1;
                    if (cnt == 5'd7) begin cnt <= 5'd0; frame_sent <= 1'b1; st <= T_IFG; end
                end

                // idle line between frames: 12 byte times
                T_IFG: begin
                    TX_EN <= 1'b0; TXD <= 4'h0;
                    cnt <= cnt + 5'd1;
                    if (cnt == 5'd23) st <= T_IDLE;
                end

                // after an underrun: read and discard up to and including the frame's last byte
                T_FLUSH: begin
                    TX_EN <= 1'b0; TXD <= 4'h0;
                    if (rd_valid && rd_data[8]) begin cnt <= 5'd0; st <= T_IFG; end
                end

                default: st <= T_IDLE;
            endcase
        end
    end

    wire _unused = &{cur_ok, fcs, 1'b0};

endmodule
