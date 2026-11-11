/*
 * The SRAM pin buffer as an empty box, for the equivalence check: the RTL instantiates this cell by name
 * (pe_mem.sv), and the netlist keeps the same 44 instances. As a box on both sides, each buffer splits its
 * input net from its output net on both sides alike.
 * SPDX-License-Identifier: Apache-2.0
 */
(* blackbox *)
module sg13cmos5l_buf_2 (
    input  wire A,
    output wire X
);
endmodule
