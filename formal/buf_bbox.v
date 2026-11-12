/*
 * The library's sg13cmos5l_buf_2 as an empty module, for the RTL side of the equivalence check only: the RTL
 * instantiates this cell by name for the SRAM's 44 pin buffers (pe_mem.sv), and slang needs a module for it.
 * chip.eqy then gives those 44 cells the box type of pinbuf_box.v, as it does to the same 44 in the netlist.
 * SPDX-License-Identifier: Apache-2.0
 */
(* blackbox *)
module sg13cmos5l_buf_2 (
    input  wire A,
    output wire X
);
endmodule
