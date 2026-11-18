`ifndef SYS_DEFS_SVH
`define SYS_DEFS_SVH

`define OCC_WIDTH 100 
`define MEM_DEPTH 5000 // Banked so 10000 / 2

typedef logic signed [15:0] DATA;
typedef logic signed [7:0] OCC_ENTRY; 
typedef logic [6:0] MEM_X; 
typedef logic [6:0] MEM_Y; 
typedef logic [3:0] ELEVATION; // 4-bit Laser ID

typedef struct packed {
    logic [1:0] valid;
    DATA  [1:0] distance;
    DATA  [1:0] azimuth;
    ELEVATION [1:0] laser_id; 
} INGRESS_PACKET;

typedef struct packed {
    DATA  [1:0] x;
    DATA  [1:0] y;
    DATA  [1:0] z;
    logic [1:0] valid;
} TRANSFORM_PACKET;

typedef struct packed {
    MEM_X     [1:0] mem_x;
    MEM_Y     [1:0] mem_y;
    OCC_ENTRY [1:0] increment;
    logic     [1:0] valid;  
} RMW_PACKET;

typedef struct packed {
    logic         [1:0] valid; // [0]=Bank0 Valid, [1]=Bank1 Valid
    logic   [1:0][12:0] addr;  // Bank0/1 Addresses
    OCC_ENTRY     [1:0] increment;   // Bank0/1 Increments
} R2W_PACKET;

`endif // SYS_DEFS_SVH
