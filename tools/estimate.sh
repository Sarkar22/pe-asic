#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Area and timing estimate. Synthesis with Yosys and slang in the LibreLane image, against the
# slow-corner liberty, with the SRAM as a black box and no placement: no wires, ideal clock.
#
# Usage: tools/estimate.sh <output folder>
# Needs PDK_ROOT. Writes flat.v, flat.txt, hier.txt and sta.txt there.
set -e
OUT=$(realpath "$1")
ROOT=$(realpath "$(dirname "$0")/..")
PDK=$(realpath "${PDK_ROOT:?set PDK_ROOT}")
IMAGE=ghcr.io/librelane/librelane:3.1.0.dev4 # the image of the flow, as in scripts/setup.sh
mkdir -p "$OUT"

LIB=/pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lib/sg13cmos5l_stdcell_slow_1p08V_125C.lib
SRAM=/pdk/ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram/lib/RM_IHPSG13_1P_512x16_c2_bm_bist_slow_1p08V_125C.lib
FILES="src/project.sv src/pe_pkg.sv src/pe_decode.sv src/pe_core.sv src/pe_thread.sv src/pe_exec.sv
       src/pe_regfile.sv src/pe_mailbox.sv src/pe_pins.sv src/pe_mem.sv src/pe_host.sv src/pe_spi.sv
       verif/sram_blackbox.v"
TOP=tt_um_sarkar22_pe

cat > "$OUT/abc.constr" <<CONSTR
set_driving_cell sg13cmos5l_buf_1
set_load 6.0
CONSTR

synth_script() {
  cat <<YS
plugin -i slang
read_slang --top $TOP $3 $(echo $FILES)
hierarchy -check -top $TOP
$1
opt -purge
techmap
opt -fast
dfflibmap -liberty $LIB
opt
abc -liberty $LIB -constr /out/abc.constr -script /work/tools/abc_area.script -D 20000
opt_clean -purge
hilomap -hicell sg13cmos5l_tiehi L_HI -locell sg13cmos5l_tielo L_LO -singleton
opt_clean -purge
tee -o /out/$2.txt stat -top $TOP -liberty $LIB -liberty $SRAM
write_verilog -noattr -noexpr -nohex -nodec /out/$2.v
YS
}

synth_script "synth -top $TOP -flatten" flat "" > "$OUT/flat.ys"
synth_script "synth -top $TOP" hier "--keep-hierarchy" > "$OUT/hier.ys"

cat > "$OUT/sta.tcl" <<STA
read_liberty $LIB
read_liberty $SRAM
read_verilog /out/flat.v
link_design $TOP
create_clock -name clk -period 20 [get_ports clk]
set_input_delay 8 -clock clk [all_inputs -no_clocks]
set_output_delay 8 -clock clk [all_outputs]
puts "=== worst paths, every group. The reset net has no buffers until placement"
report_checks -path_delay max -group_path_count 1 -format full
puts "=== worst paths between flops and the SRAM"
set from [all_registers -clock_pins]
set to [all_registers -data_pins]
report_checks -path_delay max -from \$from -to \$to -format full
puts "=== worst paths that start at a flop, not the SRAM"
set_false_path -from [get_pins core.mem.sram/A_CLK]
report_checks -path_delay max -from \$from -to \$to -format full
report_checks -path_delay max -from \$from -to \$to -group_path_count 12 -unique_paths_to_endpoint -format end
exit
STA

run() {
  docker run --rm --memory 3g -v "$ROOT":/work:ro -v "$PDK":/pdk:ro -v "$OUT":/out -w /work "$IMAGE" "$@"
}

run yosys -q -l /out/flat.log -s /out/flat.ys
run yosys -q -l /out/hier.log -s /out/hier.ys
run sta -no_init -no_splash -exit /out/sta.tcl > "$OUT/sta.txt" 2>&1
