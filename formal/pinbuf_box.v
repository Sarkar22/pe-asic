/*
 * The SRAM's 44 pin buffers as an empty box, for the equivalence check. The RTL instantiates the library's
 * sg13cmos5l_buf_2 by name for them (pe_mem.sv) and the netlist keeps the same 44 instances; chip.eqy gives
 * these 44, and only these, this type on both sides, so each splits its input net from its output net on
 * both sides alike. Every other buffer in the netlist keeps the liberty's function.
 * SPDX-License-Identifier: Apache-2.0
 */
(* blackbox *)
module pe_sram_pinbuf (
    input  wire A,
    output wire X
);
endmodule
