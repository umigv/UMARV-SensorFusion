// MII receiver: 4-bit nibbles from the PHY in, whole bytes out.
//
// Everything here is in the PHY's receive clock domain (RX_CLK, 25 MHz at 100 Mbit/s).
// There is no clock crossing in this block; the outputs are meant to be written straight
// into an asynchronous FIFO: {rx_data, rx_sop, rx_eop, rx_error} with rx_valid as the write strobe.
//
// Output stream
//   rx_valid  one clock per byte
//   rx_sop    with the first byte of a frame (the first byte after the start marker)
//   rx_eop    with the last byte of a frame
//   rx_error  with rx_eop only: the frame is damaged (bad checksum, RX_ER seen, ended half-way
//             through a byte, or longer than MAX_BYTES)
//
// The preamble and start marker are removed. The four checksum bytes at the end of the frame
// are NOT removed; they come out as the last four bytes. Each byte is held back until the
// next one arrives (or the frame ends), so that rx_eop can go out with the real last byte.
//
// Written in the plain SystemVerilog subset the Tiny Tapeout tools accept: no enums, structs,
// size casts or 'return'.
module mii_recv (
    input logic RX_CLK,
    input logic RX_RST,             // active high, asserted asynchronously, released in step with RX_CLK
    input logic [3:0] RXD,
    input logic RX_DV, RX_ER,       // tie RX_ER low if the pin is not connected

    output logic [7:0] rx_data,
    output logic rx_valid,
    output logic rx_sop, rx_eop, rx_error
);

localparam [1:0] S_IDLE = 2'd0, S_PREAMBLE = 2'd1, S_DATA = 2'd2, S_DROP = 2'd3;
logic [1:0] s, ns;

localparam [10:0] MAX_BYTES = 11'd1536;        // longest frame passed on, checksum included

// Inputs are registered once, on every clock, and only the registered copies are used below.
logic [3:0] rxd_reg;
logic rx_dv_reg, rx_er_reg;

logic sfd_found;                                // the start marker: a D nibble after the 5s
assign sfd_found = rx_dv_reg && (rxd_reg == 4'hD);

logic nibble_sel;                               // 0: low nibble arriving, 1: high nibble
logic [3:0] low_nibble;
logic byte_done;                                // a whole byte is on {rxd_reg, low_nibble}
assign byte_done = (s == S_DATA) && rx_dv_reg && nibble_sel;

logic [7:0] held_data;                          // the most recent byte, waiting to go out
logic held_valid, held_first;
logic [10:0] byte_count;
logic er_seen;
logic too_long;
assign too_long = byte_done && (byte_count == MAX_BYTES);

logic frame_end;                                // RX_DV has dropped, or the frame is cut for length
assign frame_end = (s == S_DATA) && (!rx_dv_reg || too_long);

// Ethernet CRC-32, four bits per clock, least significant bit first.
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
logic crc_ok;                                   // running the checksum bytes through leaves this value
assign crc_ok = (crc == 32'hDEBB20E3);

always_comb begin
    ns = s;
    case(s)
        S_IDLE: begin
            if(rx_dv_reg) begin
                if(sfd_found)             ns = S_DATA;
                else if(rxd_reg == 4'h5)  ns = S_PREAMBLE;
                else                      ns = S_DROP;      // not a preamble
            end
        end
        S_PREAMBLE: begin
            if(!rx_dv_reg)                ns = S_IDLE;
            else if(sfd_found)            ns = S_DATA;
            else if(rxd_reg != 4'h5)      ns = S_DROP;
        end
        S_DATA: begin
            if(!rx_dv_reg)                ns = S_IDLE;
            else if(too_long)             ns = S_DROP;
        end
        S_DROP: ns = (!rx_dv_reg) ? S_IDLE : S_DROP;        // ignore the rest of this frame
    endcase
end

always_ff @(posedge RX_CLK or posedge RX_RST) begin
    if(RX_RST) begin
        s <= S_IDLE;
        rxd_reg <= '0;
        rx_dv_reg <= '0;
        rx_er_reg <= '0;
        nibble_sel <= '0;
        low_nibble <= '0;
        held_data <= '0;
        held_valid <= '0;
        held_first <= '0;
        byte_count <= '0;
        er_seen <= '0;
        crc <= '1;
        rx_data <= '0;
        rx_valid <= '0;
        rx_sop <= '0;
        rx_eop <= '0;
        rx_error <= '0;
    end else begin
        s <= ns;
        rxd_reg <= RXD;
        rx_dv_reg <= RX_DV;
        rx_er_reg <= RX_ER;

        rx_valid <= 1'b0;
        rx_sop <= 1'b0;
        rx_eop <= 1'b0;
        rx_error <= 1'b0;

        if(s != S_DATA) begin
            // get ready for the next frame
            nibble_sel <= 1'b0;
            held_valid <= 1'b0;
            byte_count <= '0;
            er_seen <= 1'b0;
            crc <= '1;
        end else if(rx_dv_reg) begin
            nibble_sel <= ~nibble_sel;
            crc <= crc_nibble(crc, rxd_reg);
            if(rx_er_reg) er_seen <= 1'b1;
            if(!nibble_sel) begin
                low_nibble <= rxd_reg;
            end else begin
                // a byte is complete: send the one that was waiting, keep this one
                held_data <= {rxd_reg, low_nibble};
                held_valid <= 1'b1;
                held_first <= !held_valid;
                byte_count <= byte_count + 1'b1;
            end
        end

        // send the waiting byte when the next one arrives, or when the frame ends
        if(held_valid && (byte_done || frame_end)) begin
            rx_data <= held_data;
            rx_valid <= 1'b1;
            rx_sop <= held_first;
            if(frame_end) begin
                rx_eop <= 1'b1;
                rx_error <= !crc_ok || er_seen || rx_er_reg || nibble_sel || too_long;
            end
        end
    end
end

endmodule
