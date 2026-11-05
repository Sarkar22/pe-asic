/*
 * Behavioural stand-in for the PDK's 512x16 SRAM, for RTL simulation. Same name and ports.
 * A read shows up after the clock edge and stays until the next read.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module RM_IHPSG13_1P_512x16_c2_bm_bist (
    input  wire         A_CLK,
    input  wire         A_MEN,
    input  wire         A_WEN,
    input  wire         A_REN,
    input  wire  [8:0]  A_ADDR,
    input  wire  [15:0] A_DIN,
    input  wire         A_DLY,
    output wire  [15:0] A_DOUT,
    input  wire  [15:0] A_BM,
    input  wire         A_BIST_CLK,
    input  wire         A_BIST_EN,
    input  wire         A_BIST_MEN,
    input  wire         A_BIST_WEN,
    input  wire         A_BIST_REN,
    input  wire  [8:0]  A_BIST_ADDR,
    input  wire  [15:0] A_BIST_DIN,
    input  wire  [15:0] A_BIST_BM
);

  reg [15:0] memory [0:511];
  reg [15:0] dr_r;

  assign A_DOUT = dr_r;

  always @(posedge A_CLK) begin
    if (A_MEN && A_WEN) begin
      memory[A_ADDR] <= (memory[A_ADDR] & ~A_BM) | (A_DIN & A_BM);
    end else if (A_MEN && A_REN) begin
      dr_r <= memory[A_ADDR];
    end
  end

  wire _unused = &{A_DLY, A_BIST_CLK, A_BIST_EN, A_BIST_MEN, A_BIST_WEN, A_BIST_REN,
                   A_BIST_ADDR, A_BIST_DIN, A_BIST_BM, 1'b0};

endmodule
