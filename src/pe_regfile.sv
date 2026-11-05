/*
 * R0..R7 of all four threads: 32 words of 16 bits, two reads in stage 3, one write in stage 4.
 * Reads and writes in one cycle belong to different threads, so nothing is bypassed.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_regfile (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [4:0]  raddr_a,
    input  logic [4:0]  raddr_b,
    output logic [15:0] rdata_a,
    output logic [15:0] rdata_b,
    input  logic        we,
    input  logic [4:0]  waddr,
    input  logic [15:0] wdata
);

  logic [15:0] regs_q [0:31];

  assign rdata_a = regs_q[raddr_a];
  assign rdata_b = regs_q[raddr_b];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 32; i = i + 1) regs_q[i] <= 16'h0000;
    end else if (we) begin
      regs_q[waddr] <= wdata;
    end
  end

endmodule
