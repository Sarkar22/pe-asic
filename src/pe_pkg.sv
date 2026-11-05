/*
 * Constants shared by the design.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Words of memory. 512 is the SRAM. Simulation also builds 128, which timing does not notice.
`ifndef PE_DEPTH
`define PE_DEPTH 512
`endif

package pe_pkg;

  localparam int unsigned DEPTH      = `PE_DEPTH;
  localparam logic [8:0]  ADDR_MASK  = 9'(DEPTH - 1);
  localparam logic [3:0]  LOG2_DEPTH = 4'($clog2(DEPTH));

  localparam logic [7:0] ISA_VERSION = 8'h01;
  localparam logic [3:0] THREADS     = 4'd4;

  // What a pending event is.
  localparam logic KIND_TICK = 1'b0;
  localparam logic KIND_WAIT = 1'b1;

  // What a memory instruction does in its memory cycle.
  localparam logic MEM_LOAD  = 1'b0;
  localparam logic MEM_STORE = 1'b1;

  // pcfg modes, as encoded in the instruction.
  localparam logic [1:0] CFG_IN  = 2'd0;
  localparam logic [1:0] CFG_PP  = 2'd1;
  localparam logic [1:0] CFG_OD  = 2'd2;
  localparam logic [1:0] CFG_BYP = 2'd3;

endpackage
