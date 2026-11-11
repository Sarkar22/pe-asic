#!/usr/bin/env bash
# Shows that the proofs have teeth: plants one one-line bug at a time in a scratch copy of the RTL, runs the
# check that should catch it, and expects it to fail with a counterexample trace from reset: the bmc task of
# that proof (the same assertions, from reset, with the host's commands free), the reset proof itself, or the
# equivalence check. The repository is not touched.
#
# Usage: formal/teeth.sh [bug ...]       with no bug, every bug in BUGS below
# Uses run.sh, with the same environment (FORMAL_IMAGE, FORMAL_DOCKER, FORMAL_WORK, PDK_ROOT, FORMAL_NETLIST).
# SPDX-License-Identifier: Apache-2.0
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$HERE")
WORK=${FORMAL_WORK:-$HERE/work}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)

# name # file # line # text on that line # its replacement # job that must fail # what the bug breaks
BUGS=$(cat <<'TABLE'
od_drives_high#src/pe_pins.sv#187#uio_out = v[15:8] & ~od_q;#uio_out = v[15:8];#pads:bmc#5.2: an open-drain pin drives its value high
bypass_flop#src/pe_core.sv#444#.ui_in   (ui_in),#.ui_in   (sync1_q[7:0]),#pads:bmc#5.8: the bypass path goes through a flop
put_not_owner#src/pe_pins.sv#65#mine[p]  = own_q[2*p+:2] == tid;#mine[p]  = 1'b1;#isolation:bmc#5.4: a thread writes the shadow of a pin it doesn't own
commit_any#src/pe_pins.sv#64#moves[p] = commit[own_q[2*p+:2]];#moves[p] = |commit;#isolation:bmc#5.4: any thread's commit moves every pin
abort_keeps_sel#src/pe_pins.sv#140#sel_d       = sel_d & 32'h0000_FFFF;#sel_d       = sel_d;#isolation:bmc#9.3: ABORT leaves uio pins on their shift outputs
unit_stop_shared#src/pe_core.sv#395#.sstop_tx(u_sstop_tx & ex_one[i]),#.sstop_tx(u_sstop_tx),#isolation:bmc#5.9.1: one thread's sstop stops every unit
symbol_too_long#src/pe_shift.sv#263#(tx_cd_q <= 17'd1)#(tx_cd_q == 17'd0)#shift_timing:bmc#5.9.5: every symbol lasts TPER + 1 cycles
start_without_commit#src/pe_shift.sv#380#if (tx_arm_q && cmt) begin#if (tx_arm_q) begin#shift_timing:bmc#5.9.5: an armed send starts without a commit
fraction_lost#src/pe_shift.sv#419#({1'b0, tper} + {16'd0, sym_carry})#({1'b0, tper})#shift_timing:bmc#5.9.5: the TFRAC carry never lengthens a symbol
sstop_no_idle#src/pe_shift.sv#491#la_d      = idlea;#la_d      = la_a;#shift_timing:bmc#5.9.4: sstop tx leaves output A where it was
own_reset#src/pe_pins.sv#162#own_q   <= 32'h0000_0000;#own_q   <= 32'h0000_0001;#reset:flops_prove#8: pin 0 is owned by thread 1 after reset
no_reset_s#src/pe_thread.sv#212#s_q        <= 16'h0000;##reset:flops_prove#8: S has no reset
pcfg_in_drives#src/pe_pins.sv#112#shoe_d[pin[2:0]] = 1'b0;#shoe_d[pin[2:0]] = 1'b1;#reset:pads_bmc#5.3: pcfg IN enables the driver at the next commit
netlist_differs#src/pe_pins.sv#187#uio_out = v[15:8] & ~od_q;#uio_out = v[15:8] & od_q;#chip.eqy#the netlist no longer matches the RTL
TABLE
)

names=${*:-$(echo "$BUGS" | cut -d'#' -f1)}
printf '%-22s %-20s %-8s %s\n' bug job result "what failed"
for name in $names; do
  line=$(echo "$BUGS" | grep "^$name#") || { echo "unknown bug $name"; continue; }
  IFS='#' read -r _ file lineno old new job what <<< "$line"
  scratch=$WORK/teeth/$name
  rm -rf "$scratch"; mkdir -p "$scratch/src" "$scratch/verif" "$scratch/formal"
  cp "$ROOT"/src/*.sv "$scratch/src/"
  cp "$ROOT/verif/cells_rtl.v" "$ROOT/verif/sram_blackbox.v" "$scratch/verif/"
  cp "$HERE"/*.sv "$HERE"/*.ys "$HERE"/*.sby "$HERE"/*.eqy "$HERE"/*.py "$HERE"/*.sh "$HERE"/*.v "$scratch/formal/"
  cp -r "$HERE/bin" "$scratch/formal/"
  if ! python3 -I - "$scratch/$file" "$lineno" "$old" "$new" <<'PY'
import sys
path, n, old, new = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
lines = open(path).read().split("\n")
if lines[n - 1].count(old) != 1:
    sys.exit(f"line {n} of {path} no longer holds the text to change")
lines[n - 1] = lines[n - 1].replace(old, new) if new else ""
open(path, "w").write("\n".join(lines))
PY
  then
    printf '%-22s %-20s %-8s %s\n' "$name" "$job" SKIPPED "the line moved"; continue
  fi
  out=$(FORMAL_WORK=$scratch/work "$scratch/formal/run.sh" "$job")
  status=$(echo "$out" | awk '{print $2}')
  case "$job" in
    *.eqy) log=$scratch/work/${job%.eqy}_eqy.log; detail=$(grep -m1 -oE "partition '[^']*'[^.]*not equivalent|Failed to prove equivalence of partition [^ ]*" "$log") ;;
    *) log=$scratch/work/${job%%:*}_${job##*:}.log; detail=$(grep -m1 -oE "failed assertion [^ ]+ at [^ ]+" "$log") ;;
  esac
  case "$job" in
    # The equivalence check of the real RTL already leaves pieces unproved, so here the bug must make a piece
    # fail that passes on the real RTL: the uio pads' piece.
    *.eqy) grep -q "uio_out.0' using strategy 'sat': partitions not equivalent" "$log" && result=CAUGHT || result="MISSED($status)" ;;
    *) [ "$status" = FAIL ] && result=CAUGHT || result="MISSED($status)" ;;
  esac
  printf '%-22s %-20s %-8s %s\n' "$name" "$job" "$result" "${detail:-see $log}"
done
