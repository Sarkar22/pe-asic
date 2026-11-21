/*
 * The library cells that src/ instantiates by name, as empty modules, for the RTL side of the equivalence check
 * only; slang needs a module for each. chip.eqy then turns them into the same boxes as their instances in the
 * netlist: the SRAM's 44 pin buffers (sg13cmos5l_buf_2, pe_mem.sv) into the box of pinbuf_box.v, and the
 * half-cycle latches (pe_core.sv's falling-edge pairs, pe_shift.sv's output latches) into the boxes of
 * latch_box.v, through latch_map.v.
 * SPDX-License-Identifier: Apache-2.0
 */
(* blackbox *)
module sg13cmos5l_buf_2 (
    input  wire A,
    output wire X
);
endmodule

(* blackbox *)
module sg13cmos5l_dlhq_1 (
    input  wire D,
    input  wire GATE,
    output wire Q
);
endmodule

(* blackbox *)
module sg13cmos5l_dllrq_1 (
    input  wire D,
    input  wire GATE_N,
    input  wire RESET_B,
    output wire Q
);
endmodule
