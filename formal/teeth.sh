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
od_drives_high#src/pe_pins.sv#189#uio_out = v[15:8] & ~od_q;#uio_out = v[15:8];#pads:bmc#5.2: an open-drain pin drives its value high
bypass_flop#src/pe_core.sv#465#.ui_in   (ui_in),#.ui_in   (sync1_q[7:0]),#pads:bmc#5.8: the bypass path goes through a flop
put_not_owner#src/pe_pins.sv#67#mine[p]  = own_q[2*p+:2] == tid;#mine[p]  = 1'b1;#isolation:bmc#5.4: a thread writes the shadow of a pin it doesn't own
commit_any#src/pe_pins.sv#66#moves[p] = commit[own_q[2*p+:2]];#moves[p] = |commit;#isolation:bmc#5.4: any thread's commit moves every pin
abort_keeps_sel#src/pe_pins.sv#142#sel_d       = sel_d & 32'h0000_FFFF;#sel_d       = sel_d;#isolation:bmc#9.3: ABORT leaves uio pins on their shift outputs
unit_stop_shared#src/pe_core.sv#414#.sstop_tx(u_sstop_tx & ex_one[i]),#.sstop_tx(u_sstop_tx),#isolation:bmc#5.9.1: one thread's sstop stops every unit
symbol_too_long#src/pe_shift.sv#272#(tx_cd_q <= 17'd1)#(tx_cd_q == 17'd0)#shift_timing:bmc#5.9.5: every symbol lasts TPER + 1 cycles
start_without_commit#src/pe_shift.sv#391#if (tx_arm_q && cmt) begin#if (tx_arm_q) begin#shift_timing:bmc#5.9.5: an armed send starts without a commit
fraction_lost#src/pe_shift.sv#430#({1'b0, tper} + {16'd0, sym_carry})#({1'b0, tper})#shift_timing:bmc#5.9.5: the TFRAC carry never lengthens a symbol
sstop_no_idle#src/pe_shift.sv#510#la_d      = idlea;#la_d      = la_a;#shift_timing:bmc#5.9.4: sstop tx leaves output A where it was
half_on_bit6#src/pe_shift.sv#437#tx_acc_a[7];#tx_acc_a[6];#shift_timing:bmc#5.9.5: THALF puts a move at the falling edge from bit 6 of TXACC
half_not_stopped#src/pe_shift.sv#509#tx_h_d    = 1'b0;#tx_h_d    = tx_h_d;#shift_timing:bmc#5.9.5: after sstop tx the outputs move at a falling edge
n_never_moves#src/pe_shift.sv#1019#na_q <= na_q ^ dha;#na_q <= na_q;#shift_timing:bmc#5.2: the second half of a THALF move shows the first half's level
p_without_move#src/pe_shift.sv#1018#pa_q <= la_d ^ dha ^ na_q;#pa_q <= la_d ^ na_q;#shift_timing:bmc#5.2: the first half of a THALF move shows the new level
latch_no_reset#src/pe_shift.sv#1026#.RESET_B(rst_n),#.RESET_B(1'b1),#shift_timing:bmc#8: output A's latch is not cleared by reset
latch_gate_inverted#src/pe_shift.sv#1026#.GATE_N(clk),#.GATE_N(~clk),#reset:gen#5.2: output A's latch is open while the clock is high, so the halves swap
pair_no_reset#src/pe_core.sv#52#.RESET_B(rst_n_sync),#.RESET_B(1'b1),#reset:gen#8: the falling-edge sample's second latch is not cleared by reset
p3_reset_0#src/pe_shift.sv#900#p3_q      <= 12'd512;#p3_q      <= 12'd0;#reset:gen#8, 5.9.6 implementation note: RKIND 3's p resets to 0, not to 512, its value for the reset configuration
rxce_reset_1#src/pe_shift.sv#1069#rx_ce_q   <= 13'sd0;#rx_ce_q   <= -13'sd1;#reset:gen#8: RXCE resets to -1, not 0
rxc8_low_reset#src/pe_shift.sv#1067#rx_c8_q   <= 32'h0000_0000;#rx_c8_q   <= 32'h0000_8000;#reset:gen#8: bit 15 of RXC8, in the half the Manchester byte end added, resets to 1
sync1f_first_latch#src/pe_core.sv#70#sync1f_q <= fall_l;#sync1f_q <= fall_m;#reset:gen#3.1: SYNC1F reads the latch that opens at the rising edge it samples on
own_reset#src/pe_pins.sv#164#own_q   <= 32'h0000_0000;#own_q   <= 32'h0000_0001;#reset:flops_prove#8: pin 0 is owned by thread 1 after reset
no_reset_s#src/pe_thread.sv#212#s_q        <= 16'h0000;##reset:flops_prove#8: S has no reset
pcfg_in_drives#src/pe_pins.sv#114#shoe_d[pin[2:0]] = 1'b0;#shoe_d[pin[2:0]] = 1'b1;#reset:pads_bmc#5.3: pcfg IN enables the driver at the next commit
netlist_differs#src/pe_pins.sv#189#uio_out = v[15:8] & ~od_q;#uio_out = v[15:8] & od_q;#chip.eqy#the netlist no longer matches the RTL
TABLE
)

# Netlist bugs: name # edits for plant_netlist.py # the piece that must fail # what the bug breaks (and the rule of
# chip.eqy that closes that piece)
NETLIST_BUGS=$(cat <<'TABLE'
nl_unit1_reset#swap_reset:_61146_#unit1#unit 1's pwq_q[7] resets to the wrong value (inverted flops)
nl_unit0_reset#swap_reset:_60562_#unit0#unit 0's pwh_q[7] resets to the wrong value (inverted flops, merges kept apart)
nl_unit2_p3_reset#swap_reset:_61656_#unit2#unit 2's RKIND 3 p (p3_q[9], 512 at reset) resets to the wrong value (inverted flops)
nl_unit3_wq3_reset#swap_reset:_62699_#unit3#unit 3's RKIND 3 window point wq3_q[8] (384 at reset) resets to the wrong value (inverted flops)
nl_byp0_inverted#invert_d:_60425_#pins#byp_q[0] loads the inverse (constant flops)
nl_byp4_dropped#drop_flop:_60424_#pins#the real flop byp_q[4] removed and tied to 0 (a wrong constant)
nl_arg_stuck#hold_d:_62925_#core.hm_wdata.9#hm_wdata[9] never loads (the merged SCK flop)
nl_miso_inverted#buf_to_inv:_63839_#host#uo_out[3] through an inverter (core.host.miso unmatched)
nl_fsm_run#rewire_d:_63017_:_63023_#thread1#the FSM's one-hot RUN bit takes SETPC's next state (PDR)
nl_fsm_binary#invert_d:_63769_#host#a bit of the FSM's binary copy loads the inverse (PDR)
nl_fanout_buffer#buf_to_inv:fanout3959#unit2#a layout fanout buffer on unit 2's tx_stf_q inverts (final netlist only)
nl_hold_cell#buf_to_inv:hold6185#host#a layout hold cell on the host's addr_q[7] inverts (final netlist only)
nl_latch_read_inverted#invert_pin:_41138_:B#uo_out.0#unit 0's output A reads its latch inverted, so after a THALF move the pad's second half is wrong (latch boxes: a box output)
nl_latch_input_buffer#buf_to_inv:wire6096#core.ui_in.3#the layout's repeater in front of ui_in[3]'s falling-edge latch inverts (latch boxes, final netlist only)
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
