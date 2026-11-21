/*
 * A techmap rule for the equivalence check, on both sides: each library latch becomes the box of latch_box.v of
 * the same instance name (_TECHMAP_REPLACE_), with the same D, RESET_B and Q connections and no gate pin.
 * sg13cmos5l_dlhq_1 is open while GATE is high, sg13cmos5l_dllrq_1 while GATE_N is low; chip.eqy explains why the
 * gate pin is left out.
 * SPDX-License-Identifier: Apache-2.0
 */
module sg13cmos5l_dlhq_1 (
    input  wire D,
    input  wire GATE,
    output wire Q
);
  pe_latch_hi _TECHMAP_REPLACE_ (
      .D(D),
      .Q(Q)
  );
endmodule

module sg13cmos5l_dllrq_1 (
    input  wire D,
    input  wire GATE_N,
    input  wire RESET_B,
    output wire Q
);
  pe_latch_lo _TECHMAP_REPLACE_ (
      .D      (D),
      .RESET_B(RESET_B),
      .Q      (Q)
  );
endmodule
