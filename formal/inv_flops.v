/*
 * A techmap rule for the equivalence check, applied only to the netlist's flops that hold a reset-to-1 bit
 * of the RTL inverted (chip.eqy lists them by instance name). The library has only reset-to-0 flops, so the
 * synthesis stores such a bit inverted, in a flop with no name. The rule keeps the cell and puts two
 * inverters in series after it: the net between them is the RTL's value of the bit, and chip.eqy gives it
 * the RTL's name, so both designs are cut at the same point. The cell's loads see ~~Q, which is Q, so the
 * netlist's function is unchanged; the net's value is not assumed but proved, as an output of its piece.
 * SPDX-License-Identifier: Apache-2.0
 */
(* techmap_celltype = "sg13cmos5l_dfrbpq_1" *)
module _pe_inverted_flop_ (
    input  wire CLK,
    input  wire D,
    input  wire RESET_B,
    output wire Q
);
  wire q;
  wire \_TECHMAP_REPLACE_.rtl ;
  // The cell itself, under a stand-in type name so the rule doesn't apply to it again; chip.eqy sets the
  // type back to sg13cmos5l_dfrbpq_1 right after, so its function is the liberty's.
  \$__pe_dfrbpq_1 _TECHMAP_REPLACE_ (
      .CLK    (CLK),
      .D      (D),
      .RESET_B(RESET_B),
      .Q      (q)
  );
  assign \_TECHMAP_REPLACE_.rtl = ~q;
  assign Q = ~\_TECHMAP_REPLACE_.rtl ;
endmodule
