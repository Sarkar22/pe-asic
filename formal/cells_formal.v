/*
 * The PDK standard cells that src/ instantiates by name, for the proofs. A proof takes one step per clock cycle:
 * every flop moves once a step, at the one global clock, and the clock net carries no level. Each output of a
 * latch that is open on a low clock still has a level in each half of the cycle (sections 3.1 and 5.2), so the
 * model shows one of them, chosen by the free bit `first` in every step: 1 the first half, from the rising edge,
 * while the latch holds; 0 the second half, after the falling edge, while it is open. A proof that looks at both
 * halves of a pad ties every latch's `first` to one free input (pads.sv, shift_timing.sv); the other proofs
 * leave each latch's choice free, which only adds behaviours. RTL simulation uses verif/cells_rtl.v instead, and
 * the gate-level test the PDK's own models.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module sg13cmos5l_buf_2 (
    input  wire A,
    output wire X
);
  assign X = A;
endmodule

// Level latch, open while GATE is high, with GATE the clock: it holds, from the falling edge to the next rising
// edge, the level its input had at the falling edge. Its input is a pad, whose level at the falling edge is not
// tied to its levels at the rising edges, so here that level is free in every cycle (section 10, item 2: the half
// cycle in which an asynchronous change is first seen).
module sg13cmos5l_dlhq_1 (
    input  wire D,
    input  wire GATE,
    output wire Q
);
  assign Q = $anyseq;
endmodule

// Level latch, open while GATE_N is low, cleared while RESET_B is low, with GATE_N the clock. It closes at the
// rising edge, so in the first half of a cycle it holds what D was at the end of the cycle before (D's value in
// the step before), and in the second half it is open and shows D. While RESET_B is low it shows 0 and holds 0.
module sg13cmos5l_dllrq_1 (
    input  wire D,
    input  wire GATE_N,
    input  wire RESET_B,
    output wire Q
);
  (* anyseq *) wire first;    // the half this step shows: 1 the first, 0 the second
  reg held;
  always @(posedge GATE_N or negedge RESET_B) begin
    if (!RESET_B) held <= 1'b0;
    else held <= D;
  end
  assign Q = !RESET_B ? 1'b0 : first ? held : D;
endmodule

`default_nettype wire
