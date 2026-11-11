/*
 * A techmap rule for the equivalence check: each $buf cell becomes a plain connection. slang joins every
 * port of a module to its connection through one; the gate netlist has none.
 * SPDX-License-Identifier: Apache-2.0
 */
(* techmap_celltype = "$buf" *)
module _unbuf_ (A, Y);
  parameter WIDTH = 1;
  input  [WIDTH-1:0] A;
  output [WIDTH-1:0] Y;
  assign Y = A;
endmodule
