module lat_to_xy #(
    // Six cosine coefficients, from -1/2! through +1/12!.
    parameter logic [6:0] COS_FIRST = 7'd109
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [31:0] latitude,
    input  logic [31:0] longitude,
    input  logic [31:0] mem_rdata,
    output logic [6:0]  mem_addr,
    output logic        mem_we,
    output logic [31:0] mem_wdata,
    output logic        busy,
    output logic        xy_valid,
    output logic [31:0] x_f32,
    output logic [31:0] y_f32
);
    localparam logic [6:0] SAMPLE_LAT = 7'd21;
    localparam logic [6:0] SAMPLE_LON = 7'd22;
    localparam logic [6:0] LAT_SQUARED = 7'd23;
    localparam logic [6:0] COSINE = 7'd24;
    localparam logic [6:0] NORTH_SCALE = 7'd25;
    localparam logic [6:0] EAST_SCALE = 7'd26;
    localparam logic [6:0] DELTA = 7'd27;
    localparam logic [6:0] EAST_METRES = 7'd28;
    localparam logic [6:0] NORTH_METRES = 7'd29;

    localparam logic [6:0] INIT_X = 7'd70; // Origin longitude, degrees
    localparam logic [6:0] INIT_Y = 7'd71; // Origin latitude, degrees
    localparam logic [6:0] PROM_ONE = 7'd96;
    localparam logic [6:0] PROM_DEG_TO_RAD = 7'd107;
    localparam logic [6:0] PROM_EARTH_RADIUS = 7'd108;
    localparam logic [6:0] COS_LAST = COS_FIRST + 7'd5;

    typedef enum logic [4:0] {
        IDLE, SAVE_LON, RAD, SQUARE, SEED_COS, LOAD_COEFF, HORNER,
        SCALE_NORTH, SCALE_EAST, LOAD_LON, SUB_LON, EAST,
        LOAD_LAT, SUB_LAT, NORTH, READ_EAST, PUBLISH
    } state_t;
    typedef enum logic [1:0] {READ_A, START_MAC, WAIT_MAC} phase_t;
    state_t state, next_state;
    phase_t phase;

    logic [6:0] src_a, src_b, dest, cos_addr;
    logic [31:0] operand_a, mac_a, mac_b, mac_result;
    logic mac_start, mac_clear, mac_valid;

    assign busy = state != IDLE;
    assign mac_b = state == SCALE_EAST && mem_rdata[31] ? 32'b0 : mem_rdata;

    fp32_mac projection_mac (
        .clock(clk), .reset(rst_n),
        .start(mac_start), .clear(mac_clear), .last(1'b1),
        .a(mac_a), .b(mac_b), .result(mac_result),
        .result_valid(mac_valid), .done(), .busy(), .exception_flags()
    );

    always_comb begin
        src_a = '0;
        src_b = PROM_ONE;
        dest = DELTA;
        next_state = IDLE;
        case (state)
            RAD: begin
                src_a = INIT_Y;
                src_b = PROM_DEG_TO_RAD;
                dest = LAT_SQUARED;
                next_state = SQUARE;
            end
            SQUARE: begin
                src_a = LAT_SQUARED;
                src_b = LAT_SQUARED;
                dest = LAT_SQUARED;
                next_state = SEED_COS;
            end
            SEED_COS: begin
                src_a = COS_LAST;
                dest = COSINE;
                next_state = LOAD_COEFF;
            end
            LOAD_COEFF: begin
                src_a = cos_addr;
                next_state = HORNER;
            end
            HORNER: begin
                src_a = COSINE;
                src_b = LAT_SQUARED;
                dest = COSINE;
                next_state = LOAD_COEFF;
                if (cos_addr == PROM_ONE)
                    next_state = SCALE_NORTH;
            end
            SCALE_NORTH: begin
                src_a = PROM_EARTH_RADIUS;
                src_b = PROM_DEG_TO_RAD;
                dest = NORTH_SCALE;
                next_state = SCALE_EAST;
            end
            SCALE_EAST: begin
                src_a = NORTH_SCALE;
                src_b = COSINE;
                dest = EAST_SCALE;
                next_state = LOAD_LON;
            end
            LOAD_LON: begin
                src_a = SAMPLE_LON;
                next_state = SUB_LON;
            end
            SUB_LON: begin
                src_a = INIT_X;
                next_state = EAST;
            end
            EAST: begin
                src_a = DELTA;
                src_b = EAST_SCALE;
                dest = EAST_METRES;
                next_state = LOAD_LAT;
            end
            LOAD_LAT: begin
                src_a = SAMPLE_LAT;
                next_state = SUB_LAT;
            end
            SUB_LAT: begin
                src_a = INIT_Y;
                next_state = NORTH;
            end
            NORTH: begin
                src_a = DELTA;
                src_b = NORTH_SCALE;
                dest = NORTH_METRES;
                next_state = READ_EAST;
            end
            default: begin end
        endcase
    end

    always_comb begin
        mem_addr = '0;
        mem_we = 1'b0;
        mem_wdata = mac_result;
        mac_start = 1'b0;
        mac_clear = 1'b0;
        mac_a = operand_a;
        case (state)
            IDLE: begin
                mem_addr = SAMPLE_LAT;
                mem_we = rst_n && start;
                mem_wdata = latitude;
            end
            SAVE_LON: begin
                mem_addr = SAMPLE_LON;
                mem_we = rst_n;
                mem_wdata = operand_a;
            end
            READ_EAST: mem_addr = EAST_METRES;
            PUBLISH: mem_addr = NORTH_METRES;
            default: case (phase)
                READ_A: mem_addr = src_a;
                START_MAC: begin
                    mem_addr = src_b;
                    mac_start = rst_n;
                    mac_clear = state != HORNER && state != SUB_LON && state != SUB_LAT;
                    if (state == SUB_LON || state == SUB_LAT)
                        mac_a[31] = ~operand_a[31];
                end
                WAIT_MAC: begin
                    mem_addr = dest;
                    // These steps only seed the MAC's accumulator.
                    mem_we = rst_n && mac_valid && state != LOAD_COEFF &&
                             state != LOAD_LON && state != LOAD_LAT;
                end
                default: begin end
            endcase
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            phase <= READ_A;
            xy_valid <= 1'b0;
            x_f32 <= '0;
            y_f32 <= '0;
        end else begin
            xy_valid <= 1'b0;
            case (state)
                IDLE: if (start) begin
                    operand_a <= longitude;
                    cos_addr <= COS_LAST - 7'd1;
                    state <= SAVE_LON;
                end
                SAVE_LON: state <= RAD;
                READ_EAST: begin
                    operand_a <= mem_rdata;
                    state <= PUBLISH;
                end
                PUBLISH: begin
                    x_f32 <= operand_a;
                    y_f32 <= mem_rdata;
                    xy_valid <= 1'b1;
                    state <= IDLE;
                end
                default: case (phase)
                    READ_A: begin
                        operand_a <= mem_rdata;
                        phase <= START_MAC;
                    end
                    START_MAC: phase <= WAIT_MAC;
                    WAIT_MAC: if (mac_valid) begin
                        if (state == HORNER)
                            cos_addr <= cos_addr == COS_FIRST ? PROM_ONE : cos_addr - 7'd1;
                        state <= next_state;
                        phase <= READ_A;
                    end
                    default: phase <= READ_A;
                endcase
            endcase
        end
    end
endmodule
