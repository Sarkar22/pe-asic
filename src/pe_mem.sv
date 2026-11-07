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

  // A buffer at every signal pin of the SRAM, placed next to the pin by MANUAL_GLOBAL_PLACEMENTS in
  // src/config.json. The macro allows 0.064 pF on each output and 0.5952 ns of slew on each input at the
  // slow corner, the tightest limits on the chip. Without the buffers, the flow's repair let A_MEN reach
  // 0.44 to 0.64 ns and A_DOUT 0.060 to 0.079 pF across three builds, because it under-estimates these
  // wires and never sizes a driver up for a load pin's limit. The buffers carry no logic.
  logic        men_p, wen_p, ren_p;
  logic [8:0]  addr_p;
  logic [15:0] din_p, dout_p;

  sg13cmos5l_buf_2 pinbuf_men (.A(en),  .X(men_p));
  sg13cmos5l_buf_2 pinbuf_wen (.A(wen), .X(wen_p));
  sg13cmos5l_buf_2 pinbuf_ren (.A(ren), .X(ren_p));
  for (genvar i = 0; i < 9; i++) begin : g_addr
    sg13cmos5l_buf_2 pinbuf (.A(addr[i]), .X(addr_p[i]));
  end
  for (genvar i = 0; i < 16; i++) begin : g_din
    sg13cmos5l_buf_2 pinbuf (.A(din[i]), .X(din_p[i]));
  end
  for (genvar i = 0; i < 16; i++) begin : g_dout
    sg13cmos5l_buf_2 pinbuf (.A(dout_p[i]), .X(dout[i]));
  end

  RM_IHPSG13_1P_512x16_c2_bm_bist sram (
      .A_CLK      (clk),
      .A_MEN      (men_p),
      .A_WEN      (wen_p),
      .A_REN      (ren_p),
      .A_ADDR     (addr_p),
      .A_DIN      (din_p),
      .A_DLY      (1'b1),
      .A_DOUT     (dout_p),
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
