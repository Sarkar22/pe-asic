/*
 * The covers that need a long trace from reset, on a reduced model of the chip (cut_cover.ys): the host's
 * commands, the memory's read data, the register file's read data and the mailbox are free in every cycle,
 * and units 1 to 3 are cut, so every cover here is about thread 0 and its unit. Each trace found here is one
 * the chip itself can run with a longer program: a free register read is a value ldi and lui can load, a free
 * memory read is a program word or a loaded value, and the host can send each of these commands, only slower.
 * The covers for every pin and unit that need shorter traces are in pads.sv, isolation.sv and the reset proofs,
 * on the whole chip.
 *
 *   cov_od_low         an open-drain uio pin of thread 0 on its shift output A, pulled low because LA is 0
 *   cov_od_released    the same, released because LA is 1 (pads.sv, od_*; section 5.2: "A shift output on a
 *                      pin in OD releases the pin on 1 and pulls it low on 0")
 *   cov_la_start       LA of unit 0 changes at a start, in the cycle of thread 0's commit (isolation.sv)
 *   cov_la_running     LA of unit 0 changes while its send half runs (isolation.sv, the unit rule)
 *   cov_abort_release  a driven uio pad, released by ABORT in the next cycle (isolation.sv, abort_*)
 *   cov_asked_drives   a uio pad drives after a running thread asked (reset_pads.sv, uio)
 *   cov_asked_uo       a uo pin goes high after a running thread asked (reset_pads.sv, uo)
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module covers_deep (
    input wire       clk,
    input wire       rst_n,
    input wire       ena,
    input wire [7:0] ui_in,
    input wire [7:0] uio_in
);

  wire [7:0] uo_out, uio_out, uio_oe;

  tt_um_sarkar22_pe dut (
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  // Probes, connected by probes_pins.ys, probes_iso.ys and probes_reset_pads.ys.
  wire [15:0] out_q, shd_q;
  wire [7:0]  oe_q, shoe_q, od_q, byp_q, shbyp_q;
  wire [31:0] own_q, sel_q, shsel_q;
  wire [3:0]  la, lb, run;
  wire        miso;
  wire [1:0]  th4;
  wire [15:0] ir4;
  wire [3:0]  cmt;
  wire        abort, rst_ok;
  wire [3:0]  txrun, txtl;
  wire [3:0]  u_cfg, u_push, u_pop, u_sstat, u_sgo_tx, u_sgo_rx, u_sstop_tx, u_sstop_rx, u_cmt, u_abort;
  wire        pin_put, pin_value;
  wire [3:0]  pin_f;
  wire        valid4;

  always @* if ($initstate) assume (!rst_n);

  reg past_ok = 1'b0;
  always @(posedge clk) past_ok <= 1'b1;

  reg [3:0] p_la, p_txrun, p_txtl, p_cmt;
  reg [7:0] p_oe;
  reg       p_abort, p_rst_ok;
  always @(posedge clk) begin
    p_la <= la; p_txrun <= txrun; p_txtl <= txtl; p_cmt <= cmt; p_oe <= oe_q; p_abort <= abort; p_rst_ok <= rst_ok;
  end
  wire check = past_ok && p_rst_ok && rst_ok;

  // reset_pads.sv's monitor of a running thread asking.
  wire       w9     = ir4[15:12] == 4'b1001;
  wire       w_put  = w9 && ir4[7:5] <= 3'd5;
  wire       w_oe   = w9 && ir4[7:5] == 3'd6;
  wire       w_cfg  = w9 && ir4[7:5] == 3'd7;
  wire [1:0] w_mode = ir4[4:3];
  wire       w_edge = ir4[15:12] == 4'b1110;
  wire       w_shift = ir4[15:12] == 4'b1111;
  wire       w_smap = ir4[15:9] == 7'b1101110;
  wire       w_put8 = ir4[15:7] == 9'b000001110;
  wire       w_oe8  = ir4[15:7] == 9'b000001111;
  wire [3:0] w_pin  = ir4[11:8];
  wire [3:0] w_spin = ir4[3:0];
  wire [3:0] w_mpin = ir4[8:5];
  wire live    = valid4 && run[th4];
  wire ask_uio = live && ((w_oe || w_cfg && (w_mode == 2'd1 || w_mode == 2'd2)) && w_pin[3] || w_oe8);
  wire ask_uo  = live && ((w_put || w_edge || w_cfg) && !w_pin[3] || w_shift && !w_spin[3] ||
                          w_put8 && !ir4[0] || w_smap && !w_mpin[3]);
  reg asked_uio, asked_uo;
  always @(posedge clk) begin
    if (!rst_ok) begin
      asked_uio <= 1'b0;
      asked_uo  <= 1'b0;
    end else begin
      if (ask_uio) asked_uio <= 1'b1;
      if (ask_uo) asked_uo <= 1'b1;
    end
  end

  // Pins of thread 0 on shift output A, open drain, released and pulled low.
  wire [7:0] od_rel, od_low;
  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : g_uio
      assign od_rel[i] = od_q[i] && sel_q[2*(8+i)+1-:2] == 2'd1 && own_q[2*(8+i)+1-:2] == 2'd0 && !uio_oe[i];
      assign od_low[i] = od_q[i] && sel_q[2*(8+i)+1-:2] == 2'd1 && own_q[2*(8+i)+1-:2] == 2'd0 && uio_oe[i];
    end
  endgenerate

  always @* begin
    cov_od_released: cover (|od_rel);
    cov_od_low: cover (|od_low);
    if (check) begin
      cov_la_start: cover (!p_txrun[0] && !p_txtl[0] && p_cmt[0] && la[0] != p_la[0]);
      cov_la_running: cover (p_txrun[0] && la[0] != p_la[0]);
      cov_abort_release: cover (p_abort && p_oe != 8'h00 && uio_oe == 8'h00);
    end
    cov_asked_drives: cover (asked_uio && uio_oe != 8'h00);
    cov_asked_uo: cover (asked_uo && (uo_out & 8'hF7) != 8'h00);
  end

endmodule

`default_nettype wire
