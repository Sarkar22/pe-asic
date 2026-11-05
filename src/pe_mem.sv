/*
 * The one memory, for code and data: a 512x16 SRAM with a synchronous read. The address goes in
 * during the memory cycle and the word comes out in the next cycle. BIST pins are tied off.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_mem (
    input  logic        clk,
    input  logic        en,
    input  logic        wen,
    input  logic        ren,
    input  logic [8:0]  addr,
    input  logic [15:0] din,
    output logic [15:0] dout
);

  RM_IHPSG13_1P_512x16_c2_bm_bist sram (
      .A_CLK      (clk),
      .A_MEN      (en),
      .A_WEN      (wen),
      .A_REN      (ren),
      .A_ADDR     (addr),
      .A_DIN      (din),
      .A_DLY      (1'b1),
      .A_DOUT     (dout),
      .A_BM       (16'hFFFF),
      .A_BIST_CLK (1'b0),
      .A_BIST_EN  (1'b0),
      .A_BIST_MEN (1'b0),
      .A_BIST_WEN (1'b0),
      .A_BIST_REN (1'b0),
      .A_BIST_ADDR(9'd0),
      .A_BIST_DIN (16'h0000),
      .A_BIST_BM  (16'h0000)
  );

endmodule
