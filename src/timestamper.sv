// Sensor front end: microsecond time base, beat, VN-300 sync and message receiver,
// AMT22 SPI reader. See docs/timestamper.md.
//
// Written in a plain subset of SystemVerilog (no structs, enums, casts or 'return') so that
// yosys, Verilator and Icarus all accept it.

module timestamper (
    input clk, rst_n, // 50 MHz clock

    // Settings
    input        enable,
    input [31:0] incr,          // 2^32 * 1e6 / f_clk
    input [15:0] period_us,     // beat frequency: 400Hz, so 2500us. 921,600 baud serial from VN300 gives you 0.6ms to send the full serial message
    input [7:0]  spi_half_div,  // sets the SPI clock rate
    input [15:0] deadline_us,   // stop waiting for sensor data this long after the beat, e.g. 2000. must be < period_us
    input        dw_check_turns, // 1: the turn-count word's check bits must be right too

    // VN GNSS IMU interface
    output logic vn_sync,
    input  vn_rx,
    
    // DW SPI interface
    output cs_n, sck, mosi,
    input miso,

    // UKF interface
    // Measurements for one beat. All of these are steady from ukf_pulse until the next beat,
    // so the UKF must read (and convert) what it needs inside that window.
    output logic               ukf_pulse,        // one clock: the measurements below are ready
    output logic        [31:0] ukf_beat_ts,      // time of the beat this snapshot belongs to, us
    output logic               ukf_vn_valid,     // new, CRC-good, status-good message this beat
    output logic        [31:0] ukf_vn_yaw_f32,   // raw IEEE float: yaw, degrees, clockwise from north
    output logic        [31:0] ukf_vn_rate_f32,  // raw IEEE float: body Z turn rate, rad/s
    output logic        [63:0] ukf_vn_lat_f64,   // raw IEEE double: latitude, degrees
    output logic        [63:0] ukf_vn_lon_f64,   // raw IEEE double: longitude, degrees
    output logic               ukf_dw_valid,     // good wheel reading this beat, with a previous one to compare against
    output logic signed [27:0] ukf_dw_delta,     // counts moved since the last good reading

    // LIDAR interface
    output logic [39:0] time_us // increments every us, never resets. 40 bits is 12.7 days
);  

    logic [5:0] us_ctr; // counts 50 clocks, updates per cycle , then resets (1us)
    logic [15:0] beat_ctr; // counts up to period_us, updates every us, then resets

    logic us_tick; // one clock per us
    assign us_tick = us_ctr == 6'd49;

    logic beat; // one clock per beat
    assign beat = enable && us_tick && (beat_ctr == period_us - 1'b1);

    // counter updates
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            us_ctr <= '0;
            beat_ctr <= '0;
            time_us <= '0;
        end else begin
            us_ctr <= (us_tick) ? '0 : us_ctr + 1'b1;
            if(us_tick) begin
                beat_ctr <= (beat || !enable) ? '0 : beat_ctr + 1'b1; // held at 0 while disabled
                time_us <= time_us + 1'b1;
            end
        end
    end

    logic vn_done, vn_good; // VN message finished / finished and trustworthy (assigned in the UART section)
    logic dw_done, dw_good; // wheel read finished / finished with good check bits (set in the SPI section)
    logic dw_delta_ok;      // there was an earlier good reading to subtract from (set in the SPI section)

    // high level state machine
    localparam S_IDLE = 1'b0, S_COLLECT = 1'b1;
    logic s;

    logic deadline_reached, collect_end;
    assign deadline_reached = (s == S_COLLECT) && us_tick && (beat_ctr == deadline_us);
    // collection ends when both sensors have answered, at the deadline, or (bad settings) at the next beat
    assign collect_end = (s == S_COLLECT) && ((dw_done && vn_done) || deadline_reached || beat);

    logic [31:0] beat_ts;           // time of the current beat
    logic vn_valid_r, dw_valid_r;   // verdicts frozen when collection ends

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            s <= S_IDLE;
            ukf_pulse <= 1'b0;
            beat_ts <= '0;
            vn_valid_r <= 1'b0;
            dw_valid_r <= 1'b0;
        end else begin
            case(s)
                S_IDLE: s <= (beat) ? S_COLLECT : S_IDLE;
                S_COLLECT: s <= (collect_end && !beat) ? S_IDLE : S_COLLECT;
            endcase

            ukf_pulse <= collect_end; // one clock: the UKF can start

            if(beat) begin
                beat_ts <= time_us[31:0] + 1'b1;  // the value time_us takes on this beat
                vn_valid_r <= 1'b0;               // nothing from the new beat is valid yet
                dw_valid_r <= 1'b0;
            end else if(collect_end) begin
                vn_valid_r <= vn_done && vn_good;
                dw_valid_r <= dw_done && dw_good && dw_delta_ok;
            end
        end
    end

    // sync pulse to the VN-300: 10us wide on every beat (SyncIn needs at least 100ns)
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) vn_sync <= 1'b0;
        else if(beat) vn_sync <= 1'b1;
        else if(us_tick && beat_ctr == 16'd9) vn_sync <= 1'b0;
    end

    // VN UART HANDLING
    logic [1:0] vn_rx_sync;

    // 2FF SYNC
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) vn_rx_sync <= '1;
        else vn_rx_sync <= {vn_rx_sync[0], vn_rx};
    end

    logic rx;
    assign rx = vn_rx_sync[1];

    // vn uart byte receiver: one pass through the states per byte
    localparam [1:0] RX_IDLE = 2'd0, RX_START = 2'd1, RX_DATA = 2'd2, RX_STOP = 2'd3;
    logic [1:0] rx_s, rx_ns;

    logic [5:0] baud_ctr;           // clocks within the current bit
    logic half_bit, full_bit;
    assign half_bit = baud_ctr == 6'd26;    // 27 clocks: middle of the start bit
    assign full_bit = baud_ctr == 6'd53;    // 54 clocks: one whole bit at 921600 baud, 50 MHz

    logic [2:0] bit_ctr;            // which of the 8 data bits
    logic sample, last_bit;
    assign sample   = (rx_s == RX_DATA) && full_bit;   // middle of a data bit // wouldnt this be half bit
    assign last_bit = sample && (bit_ctr == 3'd7);

    logic [7:0] rx_byte;
    logic       rx_byte_ready;      // one-clock pulse per received byte

    always_comb begin
        rx_ns = rx_s;
        case(rx_s)
            RX_IDLE:  if(!rx) rx_ns = RX_START;             // line dropped: maybe a start bit
            RX_START: begin
                if(rx)            rx_ns = RX_IDLE;          // went back high: glitch
                else if(half_bit) rx_ns = RX_DATA;          // still low at the middle: real start
            end
            RX_DATA:  if(last_bit) rx_ns = RX_STOP;
            RX_STOP:  if(full_bit) rx_ns = RX_IDLE;         // middle of the stop bit
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rx_s <= RX_IDLE;
            baud_ctr <= '0;
            bit_ctr <= '0;
            rx_byte <= '0;
            rx_byte_ready <= 1'b0;
        end else begin
            rx_s <= rx_ns;
            rx_byte_ready <= last_bit;
            // bit timer restarts whenever the state changes or a bit time ends
            if(rx_s == RX_IDLE || rx_ns != rx_s || full_bit) baud_ctr <= '0;
            else baud_ctr <= baud_ctr + 1'b1;
            if(rx_s == RX_START) bit_ctr <= '0;
            else if(sample) bit_ctr <= bit_ctr + 1'b1;
            if(sample) rx_byte <= {rx, rx_byte[7:1]};       // least significant bit arrives first
        end
    end

    function automatic [15:0] crc_next;
        input [15:0] c;
        input [7:0]  d;
        reg   [15:0] x;
        begin
            x = {c[7:0], c[15:8]};                 // swap the two bytes
            x = x ^ {8'h00, d};                    // mix in the new byte
            x = x ^ {12'h000, x[7:4]};
            x = x ^ {x[3:0], 12'h000};
            x = x ^ {3'b000, x[7:0], 5'b00000};
            crc_next = x;
        end
    endfunction

    logic[5:0] uart_byte_ctr;
    logic uart_packet_done;
    assign uart_packet_done = uart_byte_ctr == 6'd56; // all 56 bytes (0..55) are in

    logic [15:0] crc;
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) crc <= '0;
        else if(beat) crc <= '0;
        else if(rx_byte_ready && !uart_packet_done && uart_byte_ctr != 6'd0)
            crc <= crc_next(crc, rx_byte);
    end

    logic [31:0] header; // bytes 0..3, first byte ends up at the top
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) header <= '0;
        else if(beat) header <= '0;
        else if(rx_byte_ready && !uart_packet_done && uart_byte_ctr <= 6'd3)
            header <= {header[23:0], rx_byte};
    end

    logic valid_header;
    assign valid_header = header == 32'hFA016810;
    
    // byte ctr 
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            uart_byte_ctr <= '0;
        end else if(beat) begin
            uart_byte_ctr <= '0; // new message expected
        end else begin
            // stay at 0 until the FA sync byte shows up, then count every byte
            if(rx_byte_ready && !uart_packet_done && !(uart_byte_ctr == 6'd0 && rx_byte != 8'hFA)) begin
                uart_byte_ctr <= uart_byte_ctr + 1'b1; // number of the next byte to arrive
            end
        end
    end


    logic [31:0] yaw_raw, rate_raw; // 32-bit floats, as sent
    logic [63:0] lat_raw, lon_raw;  // 64-bit doubles, as sent
    logic [15:0] status_raw;
    // timestamper to ukf reg update
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            yaw_raw <= '0;
            rate_raw <= '0;
            lat_raw <= '0;
            lon_raw <= '0;
            status_raw <= '0;
        end else begin
            // each byte goes to the register that owns its position in the message.
            // values arrive least significant byte first, so shift in from the top
            if(rx_byte_ready && !uart_packet_done) begin
                if(uart_byte_ctr >= 6'd4  && uart_byte_ctr <= 6'd7)  yaw_raw    <= {rx_byte, yaw_raw[31:8]};    // yaw, deg
                if(uart_byte_ctr >= 6'd24 && uart_byte_ctr <= 6'd27) rate_raw   <= {rx_byte, rate_raw[31:8]};   // body Z turn rate, rad/s
                if(uart_byte_ctr >= 6'd28 && uart_byte_ctr <= 6'd35) lat_raw    <= {rx_byte, lat_raw[63:8]};    // latitude, deg
                if(uart_byte_ctr >= 6'd36 && uart_byte_ctr <= 6'd43) lon_raw    <= {rx_byte, lon_raw[63:8]};    // longitude, deg
                if(uart_byte_ctr >= 6'd52 && uart_byte_ctr <= 6'd53) status_raw <= {rx_byte, status_raw[15:8]}; // INS status
            end
        end
    end

    logic crc_ok;
    assign crc_ok = uart_packet_done && (crc == 16'h0000);

    // INS status: filter tracking (mode 2), GNSS fix, no sensor error
    logic status_ok;
    assign status_ok = (status_raw[1:0] == 2'd2) && status_raw[2] && (status_raw[6:3] == 4'd0);

    assign vn_done = uart_packet_done;                      // all 56 bytes arrived
    assign vn_good = crc_ok && valid_header && status_ok;   // and they can be trusted

    // SPI controller  

    logic [3:0] sck_ctr; // sencoder accepts at most 2MHz sck, each half of sck is 250ns = 12.5 clocks, so 13 clocks.
    logic [2:0] spi_byte_ctr; // bytes finished in this read, 0..4
    logic [3:0] spi_bit_ctr;  // bits finished in this byte, 0..8
    logic [7:0] spi_pause_ctr;
    logic pause_ended;
    assign pause_ended = (spi_pause_ctr == 8'd149); // 150 clocks = 3us

    localparam [1:0] SPI_IDLE = 2'd0, SPI_PAUSE = 2'd1, SPI_LOW = 2'd2, SPI_HIGH = 2'd3;
    logic [1:0] spi_s, spi_ns;

    always_comb begin
        spi_ns = spi_s;
        case (spi_s)
            SPI_IDLE: spi_ns = (beat) ? SPI_PAUSE : SPI_IDLE; // a read starts on every beat
            SPI_PAUSE: begin
                if(pause_ended) begin
                    spi_ns = (spi_byte_ctr == 3'd4) ? SPI_IDLE : SPI_HIGH; // go to idle if bytes are done
                end
            end
            SPI_HIGH: spi_ns = (sck_ctr == 4'd12) ? SPI_LOW : SPI_HIGH; 
            SPI_LOW: begin
                if(sck_ctr == 4'd12) begin
                    spi_ns = (spi_bit_ctr == 4'd8) ? SPI_PAUSE : SPI_HIGH; // go to pause if a byte is done
                end
            end
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin 
            spi_s <= SPI_IDLE;
            sck_ctr <= '0;
            spi_bit_ctr <= '0;
            spi_byte_ctr <= '0;
            spi_pause_ctr <= '0;
        end else begin
            spi_s <= spi_ns;

            // half-period timer: runs only while sck is being driven
            if(spi_s == SPI_HIGH || spi_s == SPI_LOW)
                sck_ctr <= (sck_ctr == 4'd12) ? '0 : sck_ctr + 1'b1;
            else
                sck_ctr <= '0;

            // pause timer: runs only in the pause state
            if(spi_s == SPI_PAUSE) spi_pause_ctr <= (spi_pause_ctr == 8'd149) ? '0 : spi_pause_ctr + 1'b1;
            else spi_pause_ctr <= '0;

            // bits finished in this byte: +1 on each falling edge of sck, cleared between bytes
            if(spi_s == SPI_PAUSE) spi_bit_ctr <= '0;
            else if(spi_s == SPI_HIGH && spi_ns == SPI_LOW) spi_bit_ctr <= spi_bit_ctr + 1'b1;

            // bytes finished in this read: +1 when a byte ends, cleared when idle
            if(spi_s == SPI_IDLE) spi_byte_ctr <= '0;
            else if(spi_s == SPI_LOW && spi_ns == SPI_PAUSE) spi_byte_ctr <= spi_byte_ctr + 1'b1;
        end
    end

    assign cs_n = (spi_s == SPI_IDLE); // low for the whole read
    assign sck  = (spi_s == SPI_HIGH); // idles low

    // mosi 8 bit sreg
    logic [7:0] tx_shift;
    assign mosi = tx_shift[7];

    // 2FF SYNC MISO
    logic [1:0] miso_sync;
    logic [31:0] rx_shift;

    logic sck_posedge, sck_negedge, spi_rd_end;
    assign sck_posedge = spi_ns == SPI_HIGH && spi_s != SPI_HIGH;
    assign sck_negedge = spi_s == SPI_HIGH && spi_ns == SPI_LOW;
    assign spi_rd_end = spi_s == SPI_PAUSE && spi_ns == SPI_IDLE;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift <= '0;
            rx_shift <= '0;
            miso_sync <= '0;
        end else begin
            miso_sync <= {miso_sync[0], miso}; // shifts every clock; [1] is the settled copy

            if(spi_s == SPI_PAUSE) tx_shift <= (spi_byte_ctr == 3'd1) ? 8'hA0 : 8'h00;

            if(sck_posedge) begin // POSEDGE = READ
                rx_shift <= {rx_shift[30:0], miso_sync[1]};
            end
            else if (sck_negedge) begin // NEGEDGE = WRITE
                tx_shift <= {tx_shift[6:0], 1'b0};
            end
        end
    end

    // AMT22 word = {K1, K0, 14 data bits}. K1 = !(xor of odd data bits), K0 = !(xor of even data bits)
    function automatic amt_check_ok;
        input [15:0] w;
        reg k1, k0;
        begin
            k1 = !(w[13] ^ w[11] ^ w[9] ^ w[7] ^ w[5] ^ w[3] ^ w[1]);
            k0 = !(w[12] ^ w[10] ^ w[8] ^ w[6] ^ w[4] ^ w[2] ^ w[0]);
            amt_check_ok = (w[15] == k1) && (w[14] == k0);
        end
    endfunction

    // rx_shift after 32 bits = {position word, turn-count word}
    logic rd_ok;
    assign rd_ok = amt_check_ok(rx_shift[31:16]) && (amt_check_ok(rx_shift[15:0]) || !dw_check_turns);

    logic [27:0] dw_now;            // {signed turn count, position within the turn}
    assign dw_now = {rx_shift[13:0], rx_shift[29:16]};

    logic [27:0] dw_prev;           // last good reading
    logic dw_have_prev;
    logic signed [27:0] dw_delta_r; // dw_now - dw_prev, wraps correctly in 28 bits

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            dw_done <= 1'b0;
            dw_good <= 1'b0;
            dw_delta_ok <= 1'b0;
            dw_prev <= '0;
            dw_have_prev <= 1'b0;
            dw_delta_r <= '0;
        end else if(beat) begin     // new read starting
            dw_done <= 1'b0;
            dw_good <= 1'b0;
            dw_delta_ok <= 1'b0;
        end else if(spi_rd_end) begin
            dw_done <= 1'b1;
            dw_good <= rd_ok;
            if(rd_ok) begin         // a bad reading is skipped: dw_prev keeps the last good one
                dw_delta_r <= dw_now - dw_prev;
                dw_delta_ok <= dw_have_prev;
                dw_prev <= dw_now;
                dw_have_prev <= 1'b1;
            end
        end
    end

    // UKF interface. The valid flags are frozen when collection ends; the data fields come
    // straight from the registers above, which do not change again until after the next beat.
    assign ukf_beat_ts      = beat_ts;
    assign ukf_vn_valid     = vn_valid_r;
    assign ukf_vn_yaw_f32   = yaw_raw;
    assign ukf_vn_rate_f32  = rate_raw;
    assign ukf_vn_lat_f64   = lat_raw;
    assign ukf_vn_lon_f64   = lon_raw;
    assign ukf_dw_valid     = dw_valid_r;
    assign ukf_dw_delta     = dw_delta_r;

endmodule
