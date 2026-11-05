`default_nettype none
`timescale 1ns / 1ps

/*
 * Testbench for test.py: the top module and wires cocotb drives and reads. At gate level the RTL
 * core also runs beside the netlist, and every cycle where an output pin differs is counted.
 * SPDX-License-Identifier: Apache-2.0
 */
module tb ();

`ifndef GL_TEST
  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  // A gate-level run is too long to dump.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end
`endif

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Top module:
  tt_um_sarkar22_pe user_project (
      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

`ifdef GL_TEST
  // A netlist can pass the protocol tests and still be wrong on pins they never
  // look at, so the RTL core runs beside it and every output pin is compared.
  wire [7:0] rtl_uo_out;
  wire [7:0] rtl_uio_out;
  wire [7:0] rtl_uio_oe;

  pe_core rtl_core (
      .ui_in  (ui_in),
      .uo_out (rtl_uo_out),
      .uio_in (uio_in),
      .uio_out(rtl_uio_out),
      .uio_oe (rtl_uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  reg     compare = 1'b0;
  integer mismatches = 0;

  // On the falling edge outputs have settled, and cocotb has not yet changed the inputs.
  always @(negedge clk) begin
    if (compare && {uo_out, uio_out, uio_oe} !== {rtl_uo_out, rtl_uio_out, rtl_uio_oe})
      mismatches = mismatches + 1;
  end
`endif

endmodule
