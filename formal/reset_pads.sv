/*
 * No pad drives until a program asks (section 8: "after reset every uio pin is undriven, and every uo pin
 * is low ... Nothing runs until the host starts it").
 *
 *   uio  uio_oe stays 0 from reset until a running thread has, in stage 4, a word that can enable a uio
 *        driver: oe or pcfg PP or OD naming a uio pin, or oe8.
 *   uo   uo_out is 0 on every pin but uo3 from reset until a running thread has, in stage 4, a word that
 *        writes or configures a uo pin: put, edge or a shift naming it, put8 on the uo bank, pcfg or smap.
 *        uo3 is the host link's data out (section 5.2) and follows the host's commands.
 *
 * "Running" is RUN of the thread in stage 4 with a fetched word there. A reset at any later time starts
 * the count again. The host's commands, ABORT included, are free in every cycle.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module reset_pads (
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

  // Probes, connected by probes_pins.ys and probes_iso.ys (some are used only by isolation.sv).
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
  wire        valid4;  // probes_reset_pads.ys

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

  always @* if ($initstate) assume (!rst_n);

  // Set from the cycle after the first asking word; cleared by reset.
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

  wire [7:0] uo_mask = 8'hF7;  // every uo pin but uo3

  always @* begin
    if (!$initstate && !asked_uio) begin
      uio_undriven: assert (uio_oe == 8'h00);
      // Invariants that make the induction close, proved like the rest.
      inv_uio: assert (shoe_q == 8'h00 && oe_q == 8'h00 && od_q == 8'h00);
    end
    if (!$initstate && !asked_uo) begin
      uo_low: assert ((uo_out & uo_mask) == 8'h00);
      inv_uo: assert ((out_q[7:0] & uo_mask) == 8'h00 && (shd_q[7:0] & uo_mask) == 8'h00 &&
                      byp_q == 8'h00 && shbyp_q == 8'h00 && sel_q[15:0] == 16'h0000 && shsel_q[15:0] == 16'h0000);
    end
    // Their covers, a pad driving and a uo pin high once asked, are in covers_deep.sv.
  end

endmodule

`default_nettype wire
