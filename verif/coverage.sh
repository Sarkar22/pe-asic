#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
#
# Code coverage of the RTL and functional coverage against docs/ISA.md, from one set of runs.
#
#   verif/coverage.sh [OUT]            OUT defaults to verif/build/coverage
#
# 1. Builds the coverage models into OUT/models: the Verilator models of verif/Makefile (stub and pins kinds, depths
#    128 and 512) with --coverage-line and --coverage-toggle, and cov_shim.cpp linked in.
# 2. Runs the stimulus. Every run records the RTL's line, branch and toggle counts (vlcov.py) and, where sim/sim.py
#    runs beside the RTL, its functional coverage (fcov.py):
#    - verif/lockstep.py, the RTL against sim/sim.py every cycle, with each generator and host mode, long images
#      that pass TB's wrap, and the scripted corner cases;
#    - every bench of programs/check.py, with the RTL in place of sim/sim.py (every check must pass).
#    Each run must pass: the script stops at the first that fails.
# 3. Writes the reports: OUT/code.md (per module), OUT/code_uncovered.txt (every point never hit), OUT/func.md
#    (per group) and OUT/func_points.txt (every point with its count), and OUT/merged.dat for verilator_coverage.
#
# Needs Verilator, a C++ compiler and Python 3. VERILATOR may name a wrapper that runs Verilator somewhere else, such
# as in a container; VERILATOR_ROOT then names a copy of its runtime for this machine's compiler. JOBS (default 8)
# processes run at once. Every run's image count is multiplied by SCALE and divided by DIV (default 1 and 1; DIV=20
# is a quick trial).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
OUT=$(realpath -m "${1:-$HERE/build/coverage}")
VERILATOR=${VERILATOR:-verilator}
JOBS=${JOBS:-8}
SCALE=${SCALE:-1}
DIV=${DIV:-1}
PY=${PYTHON:-python3}
SRC=$ROOT/src
RTL="$SRC/pe_pkg.sv $SRC/pe_decode.sv $SRC/pe_core.sv $SRC/pe_thread.sv $SRC/pe_exec.sv $SRC/pe_regfile.sv
     $SRC/pe_mailbox.sv $SRC/pe_pins.sv $SRC/pe_shift.sv $SRC/pe_mem.sv $SRC/pe_host.sv"
FLAGS="--cc --exe -O3 --x-assign 0 --x-initial 0 -Wno-fatal -Wno-lint --top-module pe_core
       --coverage-line --coverage-toggle"
mkdir -p "$OUT/models" "$OUT/runs"

echo "== models"
for kind in stub pins; do
  for d in 128 512; do
    m=$OUT/models/${kind}_$d
    rm -rf "$m"
    mkdir -p "$m/obj"
    if [ $kind = stub ]; then extra=$HERE/pe_spi_stub.sv; cf="-O2 -fPIC"; else extra=$SRC/pe_spi.sv; cf="-O2 -fPIC -DPE_PINS"; fi
    $VERILATOR $FLAGS -CFLAGS "$cf" -LDFLAGS "-shared" +define+PE_DEPTH=$d --Mdir "$m/obj" -o ../libpe.so \
      $RTL $extra "$HERE/sram_model.v" "$HERE/cells_rtl.v" "$HERE/public.vlt" "$HERE/shim.cpp" "$HERE/cov_shim.cpp" \
      > "$m/verilator.log" 2>&1 || { cat "$m/verilator.log"; exit 1; }
    make -C "$m/obj" -f Vpe_core.mk -j "$JOBS" ${VERILATOR_ROOT:+VERILATOR_ROOT=$VERILATOR_ROOT} > "$m/make.log" 2>&1 \
      || { tail -30 "$m/make.log"; exit 1; }
    echo "built $m/libpe.so"
  done
done

n() { local x=$(( $1 * SCALE / DIV )); echo $(( x > 0 ? x : 1 )); }
lockstep() {                    # tag, then lockstep.py's arguments
  local tag=$1; shift
  local t0=$(date +%s)
  $PY -B "$HERE/lockstep.py" --build "$OUT/models" --jobs "$JOBS" --out "$OUT/runs/$tag.txt" \
    --cov "$OUT/runs/$tag.cov.json" --fcov "$OUT/runs/$tag.fcov.json" "$@" > "$OUT/runs/$tag.log" 2>&1 \
    || { tail -20 "$OUT/runs/$tag.log"; echo "lockstep run $tag failed"; exit 1; }
  echo "$tag: $(tail -1 "$OUT/runs/$tag.txt"), $(( $(date +%s) - t0 )) s"
}

echo "== lockstep: the RTL against sim/sim.py"
lockstep chips  --gen chips --images "$(n 2000)" --first 0
lockstep prog   --gen prog  --images "$(n 1000)" --first 100000
lockstep rows   --gen rows  --images "$(n 2000)" --first 200000
lockstep wide   --gen wide  --images "$(n 2000)" --first 500000
lockstep pins_chips --gen chips --images "$(n 300)" --first 300000 --host pins
lockstep pins_prog  --gen prog  --images "$(n 200)" --first 310000 --host pins
lockstep pins_rows  --gen rows  --images "$(n 300)" --first 320000 --host pins
lockstep pins_wide  --gen wide  --images "$(n 300)" --first 330000 --host pins
lockstep long_chips --gen chips --images "$(n 16)" --cycles 70000 --first 400000
lockstep long_rows  --gen rows  --images "$(n 16)" --cycles 70000 --first 410000
lockstep long_wide  --gen wide  --images "$(n 16)" --cycles 70000 --first 420000
lockstep directed   --gen directed --images 3 --cycles 70000 --first 430000
lockstep corners    --corners

echo "== every program bench on the RTL"
BENCHES="words basic standard trimmed extra manchester dmx ps2 mdio jtag_chain swd_host i2c_target usb_tx usb_rx usb_dev can"
t0=$(date +%s)
running=0
for b in $BENCHES; do
  ( $PY -B "$HERE/vlcov.py" benches "$OUT/runs/bench_$b.cov.json" --build "$OUT/models" "$b" \
      > "$OUT/runs/bench_$b.log" 2>&1 && echo ok > "$OUT/runs/bench_$b.ok" || echo fail > "$OUT/runs/bench_$b.ok" ) &
  running=$((running + 1))
  if [ $running -ge "$JOBS" ]; then wait -n; running=$((running - 1)); fi
done
wait
fails=$(grep -l fail "$OUT"/runs/bench_*.ok || true)
for f in "$OUT"/runs/bench_*.log; do grep -h '^=== ' "$f" | sed "s|^|$(basename "$f" .log): |"; done
[ -z "$fails" ] || { echo "benches failed: $fails"; exit 1; }
echo "benches: $(( $(date +%s) - t0 )) s"

echo "== reports"
$PY -B "$HERE/vlcov.py" report "$OUT"/runs/*.cov.json --md "$OUT/code.md" --list "$OUT/code_uncovered.txt" \
  --dat "$OUT/merged.dat"
$PY -B "$HERE/fcov.py" report "$OUT"/runs/*.fcov.json --md "$OUT/func.md" --holes "$OUT/func_points.txt"
echo "reports in $OUT"
