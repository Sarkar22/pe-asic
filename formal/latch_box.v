/*
 * The half-cycle latches as empty boxes, for the equivalence check, on both sides: the 16 sg13cmos5l_dlhq_1 and
 * 16 sg13cmos5l_dllrq_1 of the falling-edge input samples (pe_core.sv, g_fall) and the 8 sg13cmos5l_dllrq_1
 * behind the shift units' N flops (pe_shift.sv, lat_na and lat_nb). The RTL instantiates these library cells by
 * name, and the netlist keeps the same 40 instances under the same names. latch_map.v puts a box of the same
 * instance name in place of each, on both sides, so each box's D (and RESET_B) is compared, and its Q is an input
 * both sides share.
 *
 * The box has no gate pin. EQY puts every flop on one global clock and has no clock net to give a box, and a box
 * input driven by the chip's clk, as the gate pins are on both sides, makes a piece with no logic that EQY
 * rejects (the SRAM's A_CLK is tied off for the same reason). That each gate pin is the clk pin, through clock
 * buffers only and no inverter, is checked from the netlist text, as for the flops' clock pins.
 * SPDX-License-Identifier: Apache-2.0
 */
(* blackbox *)
module pe_latch_hi (
    input  wire D,
    output wire Q
);
endmodule

(* blackbox *)
module pe_latch_lo (
    input  wire D,
    input  wire RESET_B,
    output wire Q
);
endmodule
