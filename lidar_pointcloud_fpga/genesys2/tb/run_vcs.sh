#!/usr/bin/env bash
# Simulate fpga_core at the RGMII pins with Synopsys VCS.
#   ./run_vcs.sh          (add +dump for a VCD)
set -euo pipefail
cd "$(dirname "$0")"
ROOT=$(cd ../.. && pwd)
TAXI=$ROOT/lib/taxi/src

# expand taxi's nested .f file lists into a flat, de-duplicated list
expand() {
    local f=$1 d; d=$(dirname "$f")
    while read -r line; do
        [ -z "$line" ] && continue
        case "$line" in
            *.f) expand "$d/$line" ;;
            *)   realpath "$d/$line" ;;
        esac
    done < "$f"
}
mkdir -p build && cd build
expand "$TAXI/eth/rtl/taxi_eth_mac_1g_rgmii_fifo.f" | awk '!seen[$0]++' | grep -v -E "taxi_stats_|taxi_eth_mac_stats" > taxi.f
# VCS never evaluates `wire mask = lfsr_mask();` (a function with no arguments in
# a continuous assignment), which leaves every CRC as X.  Simulate a copy with
# that one line turned into a localparam; synthesis uses the untouched file.
sed "s|^wire \(\[OUT_W-1:0\]\[IN_W-1:0\] mask = lfsr_mask();\)|localparam logic \1|" \
    "$TAXI/lfsr/rtl/taxi_lfsr.sv" > taxi_lfsr_vcs.sv
grep -q "^localparam logic .* mask = lfsr_mask" taxi_lfsr_vcs.sv
sed -i "s|.*/lfsr/rtl/taxi_lfsr.sv$|$PWD/taxi_lfsr_vcs.sv|" taxi.f
# interface definition has to be compiled first
{ grep taxi_axis_if.sv taxi.f; grep -v taxi_axis_if.sv taxi.f; } > taxi_sorted.f

cp "$ROOT/verilog/azimuth_sincos.mem" .

vcs -full64 -sverilog -ignore initializer_driver_checks +warn=noPCWM-W -timescale=1ns/1ps +incdir+"$ROOT/verilog" \
    -f taxi_sorted.f \
    "$ROOT/verilog/stage_t.sv" "$ROOT/verilog/trig_lut.sv" \
    "$ROOT"/genesys2/rtl/{vlp16_udp_rx,pc_fix2float,pc_point_fifo,pc_udp_tx,lidar_pc_core,fpga_core}.sv \
    ../tb_fpga_core.sv ${EXTRA_SRC:-} \
    -top tb_fpga_core ${EXTRA_TOP:-} -o simv -q
./simv -q "$@"
