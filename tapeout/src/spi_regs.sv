// Host SPI port and register file.
//
// The microcontroller is the SPI master; this block is the slave. It holds every setting of the
// chip in byte-wide registers and lets the host read back status and the latest measurements.
//
// SPI format: mode 0 (clock idles low, data sampled on the rising edge), most significant bit
// first, chip select active low. SCK may be up to 5 MHz: the pins are sampled with the 50 MHz
// chip clock, so there is no second clock domain.
//
//   byte 0        {write, address[6:0]}       write = 1 to write, 0 to read
//   byte 1, 2...  data. The address goes up by one after each byte, so several registers can be
//                 written or read in one transfer. Raising chip select ends the transfer.
//
// Numbers that span several registers have their low byte at the lowest address. MAC and IP
// addresses are in the order they appear on the wire (first byte at the lowest address).
//
// Register map (reset value in brackets)
//   00       ID, reads A7
//   01       version, reads 01
//   02       control [43]: bit 0 timestamper on, 1 lidar on, 2 lidar bypass, 3 output step 4 mm,
//            4 correct for turning, 5 correct for moving, 6 check encoder turn bits,
//            7 lidar state comes from the Kalman filter (0: from registers 24-3B)
//   03       lidar model [00]: 0 VLP-16 / Puck LITE, 1 Puck Hi-Res, 2 VLP-32C, 3 HDL-32E
//   04-05    beat period, microseconds [2500]
//   06-07    measurement deadline, microseconds [2000]
//   08-09    UDP port the lidar sends to, 0 = any [2368]
//   0A-0B    lidar mounting angle, 0.01 degree, signed [0]
//   0C-11    source MAC [02:00:00:00:00:01]
//   12-17    destination MAC [FF:FF:FF:FF:FF:FF]
//   18-1B    source IP [192.168.1.50]
//   1C-1F    destination IP [255.255.255.255]
//   20-21    source UDP port [2370]
//   22-23    destination UDP port [2370]
//   24-3B    state supplied by the host, six 32-bit values [0]: time (us), x, y, theta, omega, v.
//            The last five are IEEE single precision. This stands in for the Kalman filter.
//   3C       write any value: the state above is handed to the lidar block (one pulse)
//   3D       write any value: the event counters at 40-46 are cleared
//   40-46    event counters, read only, stop at 255: lidar packets, other frames, bad received
//            checksums, frames sent, buffer overflows, motion overruns, transmit underruns
//   48-4B    time of the last beat, microseconds (read only, as are all below)
//   4C       bit 0: VN-300 measurement valid, bit 1: encoder measurement valid
//   50-53    VN-300 yaw, single precision           54-57  VN-300 turn rate, single precision
//   58-5F    VN-300 latitude, double precision      60-67  VN-300 longitude, double precision
//   68-6B    encoder travel since the last beat, counts, signed
//   6C-7F    not held here. Every write, to any address, also appears on bus_we / bus_addr /
//            bus_wdata, so another block (the Kalman filter) can keep its own registers there.
//
// The measurement registers change only at a beat (meas_pulse) and then stay put until the next
// one, 2.5 ms later. Read them after the pulse and they cannot change under you.
//
// Written in the plain SystemVerilog subset the Tiny Tapeout tools accept.
module spi_regs (
    input  logic        clk,
    input  logic        rst_n,              // active low, released in step with clk

    // SPI pins
    input  logic        spi_sck,
    input  logic        spi_cs_n,
    input  logic        spi_mosi,
    output logic        spi_miso,

    // Settings out
    output logic        ts_enable,
    output logic        lidar_enable,
    output logic        lidar_bypass,
    output logic        scale_4mm,
    output logic        corr_rot_en,
    output logic        corr_shift_en,
    output logic        dw_check_turns,
    output logic        state_from_filter,
    output logic [1:0]  lidar_model,
    output logic [15:0] period_us,
    output logic [15:0] deadline_us,
    output logic [15:0] lidar_port,
    output logic signed [15:0] yaw_offset,
    output logic [47:0] src_mac, dst_mac,
    output logic [31:0] src_ip, dst_ip,
    output logic [15:0] src_port, dst_port,

    // State supplied by the host (stand-in for the Kalman filter)
    output logic        host_state_valid,   // one clock
    output logic [31:0] host_state_ts, host_state_x, host_state_y, host_state_theta, host_state_omega, host_state_v,

    // Every host write, for registers kept in other blocks
    output logic        bus_we,             // one clock
    output logic [6:0]  bus_addr,
    output logic [7:0]  bus_wdata,

    // Status in
    input  logic [6:0]  stat_pulse,         // one-clock event pulses, in the order of registers 40-46
    input  logic [31:0] beat_ts,
    input  logic        vn_valid,
    input  logic        dw_valid,
    input  logic [31:0] vn_yaw_f32, vn_rate_f32,
    input  logic [63:0] vn_lat_f64, vn_lon_f64,
    input  logic signed [27:0] dw_delta
);

    // ---- pins into the clock domain -----------------------------------------------------------
    logic [2:0] sck_s, cs_s;
    logic [1:0] mosi_s;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_s <= 3'b000; cs_s <= 3'b111; mosi_s <= 2'b00;
        end else begin
            sck_s  <= {sck_s[1:0], spi_sck};
            cs_s   <= {cs_s[1:0], spi_cs_n};
            mosi_s <= {mosi_s[0], spi_mosi};
        end
    end
    wire selected = !cs_s[1];
    wire sck_rise = selected && !sck_s[2] &&  sck_s[1];
    wire sck_fall = selected &&  sck_s[2] && !sck_s[1];

    // ---- writable registers 00-3B ----------------------------------------------------------------
    logic [7:0] w [0:59];

    function automatic [7:0] reset_value;
        input [5:0] a;
        begin
            case (a)
                6'h02: reset_value = 8'h43;                             // timestamper on, lidar on, encoder check on
                6'h04: reset_value = 8'hC4;  6'h05: reset_value = 8'h09;   // 2500
                6'h06: reset_value = 8'hD0;  6'h07: reset_value = 8'h07;   // 2000
                6'h08: reset_value = 8'h40;  6'h09: reset_value = 8'h09;   // 2368
                6'h0C: reset_value = 8'h02;  6'h11: reset_value = 8'h01;   // 02:00:00:00:00:01
                6'h12, 6'h13, 6'h14, 6'h15, 6'h16, 6'h17: reset_value = 8'hFF;
                6'h18: reset_value = 8'd192; 6'h19: reset_value = 8'd168; 6'h1A: reset_value = 8'd1; 6'h1B: reset_value = 8'd50;
                6'h1C, 6'h1D, 6'h1E, 6'h1F: reset_value = 8'hFF;
                6'h20: reset_value = 8'h42;  6'h21: reset_value = 8'h09;   // 2370
                6'h22: reset_value = 8'h42;  6'h23: reset_value = 8'h09;
                default: reset_value = 8'h00;
            endcase
        end
    endfunction

    assign ts_enable      = w[2][0];
    assign lidar_enable   = w[2][1];
    assign lidar_bypass   = w[2][2];
    assign scale_4mm      = w[2][3];
    assign corr_rot_en    = w[2][4];
    assign corr_shift_en  = w[2][5];
    assign dw_check_turns = w[2][6];
    assign state_from_filter = w[2][7];
    assign lidar_model    = w[3][1:0];
    assign period_us      = {w[5], w[4]};
    assign deadline_us    = {w[7], w[6]};
    assign lidar_port     = {w[9], w[8]};
    assign yaw_offset     = {w[11], w[10]};
    assign src_mac        = {w[12], w[13], w[14], w[15], w[16], w[17]};
    assign dst_mac        = {w[18], w[19], w[20], w[21], w[22], w[23]};
    assign src_ip         = {w[24], w[25], w[26], w[27]};
    assign dst_ip         = {w[28], w[29], w[30], w[31]};
    assign src_port       = {w[33], w[32]};
    assign dst_port       = {w[35], w[34]};
    assign host_state_ts    = {w[39], w[38], w[37], w[36]};
    assign host_state_x     = {w[43], w[42], w[41], w[40]};
    assign host_state_y     = {w[47], w[46], w[45], w[44]};
    assign host_state_theta = {w[51], w[50], w[49], w[48]};
    assign host_state_omega = {w[55], w[54], w[53], w[52]};
    assign host_state_v     = {w[59], w[58], w[57], w[56]};

    // ---- event counters ------------------------------------------------------------------------
    logic [7:0] cnt [0:6];
    logic       cnt_clear;

    // ---- read data -----------------------------------------------------------------------------
    wire [31:0] dw_ext = {{4{dw_delta[27]}}, dw_delta};

    function automatic [7:0] pick;          // byte n of a value, low byte first
        input [63:0] v;
        input [2:0]  n;
        begin
            pick = v[8*n +: 8];
        end
    endfunction

    logic [6:0] rd_addr;
    logic [7:0] rd_data;
    always_comb begin
        rd_data = 8'h00;
        if (rd_addr == 7'h00)                           rd_data = 8'hA7;
        else if (rd_addr == 7'h01)                      rd_data = 8'h01;
        else if (rd_addr < 7'h3C)                       rd_data = w[rd_addr[5:0]];
        else if (rd_addr >= 7'h40 && rd_addr <= 7'h46)  rd_data = cnt[rd_addr[2:0]];
        else if (rd_addr >= 7'h48 && rd_addr <= 7'h4B)  rd_data = pick({32'd0, beat_ts}, {1'b0, rd_addr[1:0]});
        else if (rd_addr == 7'h4C)                      rd_data = {6'd0, dw_valid, vn_valid};
        else if (rd_addr >= 7'h50 && rd_addr <= 7'h53)  rd_data = pick({32'd0, vn_yaw_f32}, {1'b0, rd_addr[1:0]});
        else if (rd_addr >= 7'h54 && rd_addr <= 7'h57)  rd_data = pick({32'd0, vn_rate_f32}, {1'b0, rd_addr[1:0]});
        else if (rd_addr >= 7'h58 && rd_addr <= 7'h5F)  rd_data = pick(vn_lat_f64, rd_addr[2:0]);
        else if (rd_addr >= 7'h60 && rd_addr <= 7'h67)  rd_data = pick(vn_lon_f64, rd_addr[2:0]);
        else if (rd_addr >= 7'h68 && rd_addr <= 7'h6B)  rd_data = pick({32'd0, dw_ext}, {1'b0, rd_addr[1:0]});
    end

    // ---- the transfer ---------------------------------------------------------------------------
    logic [2:0] bit_cnt;
    logic [6:0] sh_in;              // the bits of the current byte received so far
    logic       got_addr;           // the first byte has been received
    logic       is_write;
    logic [6:0] addr;
    logic [7:0] sh_out;

    wire [7:0] byte_in   = {sh_in, mosi_s[1]};
    wire       byte_done = sck_rise && (bit_cnt == 3'd7);
    wire [6:0] next_addr = got_addr ? (addr + 7'd1) : byte_in[6:0];
    assign rd_addr = next_addr;

    integer k;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < 60; k = k + 1) w[k] <= reset_value(k[5:0]);
            for (k = 0; k < 7; k = k + 1)  cnt[k] <= 8'd0;
            bit_cnt <= 3'd0; sh_in <= 7'd0; got_addr <= 1'b0; is_write <= 1'b0; addr <= 7'd0;
            sh_out <= 8'd0; spi_miso <= 1'b0;
            host_state_valid <= 1'b0; cnt_clear <= 1'b0;
            bus_we <= 1'b0; bus_addr <= 7'd0; bus_wdata <= 8'd0;
        end else begin
            host_state_valid <= 1'b0;
            bus_we           <= 1'b0;
            cnt_clear        <= 1'b0;

            if (!selected) begin
                bit_cnt <= 3'd0; got_addr <= 1'b0; spi_miso <= 1'b0;
            end else begin
                if (sck_rise) begin
                    sh_in   <= byte_in[6:0];
                    bit_cnt <= bit_cnt + 3'd1;
                end
                if (byte_done) begin
                    if (!got_addr) begin
                        got_addr <= 1'b1;
                        is_write <= byte_in[7];
                        addr     <= byte_in[6:0];
                    end else begin
                        if (is_write) begin
                            bus_we <= 1'b1; bus_addr <= addr; bus_wdata <= byte_in;
                            if (addr >= 7'h02 && addr < 7'h3C) w[addr[5:0]] <= byte_in;
                            if (addr == 7'h3C) host_state_valid <= 1'b1;
                            if (addr == 7'h3D) cnt_clear <= 1'b1;
                        end
                        addr <= addr + 7'd1;
                    end
                    sh_out <= rd_data;              // the byte the host will clock out next
                end
                if (sck_fall) begin
                    spi_miso <= sh_out[7] && got_addr && !is_write;
                    sh_out   <= {sh_out[6:0], 1'b0};
                end
            end

            for (k = 0; k < 7; k = k + 1) begin
                if (cnt_clear)                               cnt[k] <= 8'd0;
                else if (stat_pulse[k] && cnt[k] != 8'hFF)   cnt[k] <= cnt[k] + 8'd1;
            end
        end
    end

endmodule
