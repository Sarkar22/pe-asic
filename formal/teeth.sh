#!/usr/bin/env bash
# Shows that the proofs have teeth: plants one one-line bug at a time in a scratch copy of the RTL, runs the
# check that should catch it, and expects it to fail with a counterexample trace from reset: the bmc task of
# that proof (the same assertions, from reset, with the host's commands free), the reset proof itself, or the
# equivalence check. The repository is not touched.
#
# The bugs in NETLIST_BUGS are planted in a scratch copy of the gate netlist instead (FORMAL_NETLIST), by
# plant_netlist.py, and the equivalence check must fail in the piece the bug is in. Each takes one run of
# chip.eqy; a bug whose cells the netlist lacks (the layout's buffers, in the synthesis output) is skipped.
#
# Usage: formal/teeth.sh [bug ...]       with no bug, every bug in BUGS, and in NETLIST_BUGS if FORMAL_NETLIST is set
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

# Netlist bugs: name # edits for plant_netlist.py # the piece that must fail # what the bug breaks (and the rule of
# chip.eqy that closes that piece)
NETLIST_BUGS=$(cat <<'TABLE'
nl_unit1_reset#swap_reset:_49784_#unit1#unit 1's pwq_q[7] resets to the wrong value (inverted flops)
nl_unit0_reset#swap_reset:_49294_#unit0#unit 0's pwh_q[7] resets to the wrong value (inverted flops, merges kept apart)
nl_byp0_inverted#invert_d:_49109_#pins#byp_q[0] loads the inverse (constant flops)
nl_byp4_dropped#drop_flop:_49108_#pins#the real flop byp_q[4] removed and tied to 0 (a wrong constant)
nl_arg_stuck#hold_d:_51293_#core.hm_wdata.9#hm_wdata[9] never loads (the merged SCK flop)
nl_miso_inverted#buf_to_inv:_52053_#host#uo_out[3] through an inverter (core.host.miso unmatched)
nl_fsm_run#rewire_d:_51385_:_51391_#thread1#the FSM's one-hot RUN bit takes SETPC's next state (PDR)
nl_fsm_binary#invert_d:_51983_#host#bit 0 of the FSM's binary copy loads the inverse (PDR)
nl_fanout_buffer#buf_to_inv:fanout3296#unit2#a layout fanout buffer inverts (final netlist only)
nl_hold_cell#buf_to_inv:hold5195#host#a layout hold cell inverts (final netlist only)
TABLE
)

all=$(echo "$BUGS" | cut -d'#' -f1)
[ -n "${FORMAL_NETLIST:-}" ] && all="$all $(echo "$NETLIST_BUGS" | cut -d'#' -f1)"
names=${*:-$all}
printf '%-22s %-20s %-8s %s\n' bug job result "what failed"
for name in $names; do
  if nl=$(echo "$NETLIST_BUGS" | grep "^$name#"); then
    IFS='#' read -r _ edits piece what <<< "$nl"
    scratch=$WORK/teeth/$name
    # An old run is left alone rather than deleted: an EQY work folder holds thousands of files.
    if [ -e "$scratch" ]; then
      printf '%-22s %-20s %-8s %s\n' "$name" chip.eqy SKIPPED "$scratch exists; remove it or set another FORMAL_WORK"
      continue
    fi
    mkdir -p "$scratch/work"
    netlist=$scratch/$(basename "${FORMAL_NETLIST:?netlist bugs need FORMAL_NETLIST}")
    # shellcheck disable=SC2086  # the edits are separate arguments
    if ! python3 -I "$HERE/plant_netlist.py" "$FORMAL_NETLIST" "$netlist" $edits > "$scratch/edits.txt" 2>&1; then
      printf '%-22s %-20s %-8s %s\n' "$name" chip.eqy SKIPPED "the netlist has no such cell (see $scratch/edits.txt)"
      continue
    fi
    out=$(FORMAL_NETLIST=$netlist FORMAL_WORK=$scratch/work "$HERE/run.sh" chip.eqy)
    status=$(echo "$out" | awk '{print $2}')
    log=$scratch/work/chip_eqy.log
    if [ "$status" = FAIL ] && grep -q "partition 'tt_um_sarkar22_pe.$piece' using strategy '[a-z]*': partitions not equivalent" "$log"; then
      result=CAUGHT
    else
      result="MISSED($status)"
    fi
    printf '%-22s %-20s %-8s %s\n' "$name" chip.eqy "$result" "$piece: $what"
    continue
  fi
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
    # The equivalence check proves every piece of the real RTL, so the bug must make one fail, and it must be
    # the one the bug is in: the uio pads' piece.
    *.eqy) [ "$status" = FAIL ] && grep -q "uio_out.0' using strategy '[a-z]*': partitions not equivalent" "$log" && result=CAUGHT || result="MISSED($status)" ;;
    *) [ "$status" = FAIL ] && result=CAUGHT || result="MISSED($status)" ;;
  esac
  printf '%-22s %-20s %-8s %s\n' "$name" "$job" "$result" "${detail:-see $log}"
done
