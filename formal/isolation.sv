/*
 * Thread pin isolation (sections 5.3, 5.4, 5.9 and 9.3). The rule, derived from those sections:
 *
 * A pin p's state changes only in a cycle in which one of the causes listed for it is present. OWN[p] is
 * read at the start of the cycle, X is the thread in stage 4 (its execute cycle, TB mod 4), W is the word
 * in stage 4, CMT[t] is thread t's commit in this cycle, and ABORT is the host's.
 *
 *   OWN[p]    a pcfg in W names p, and then OWN[p] becomes X                      claim()
 *   SHD[p]    X = OWN[p] and W is put, edge or a shift naming p, or put8 on p's   shadow(), put8()
 *             bank; or W is pcfg p, OD; or ABORT, for a uio pin                  pcfg_od(), abort_pins()
 *   SHSEL[p]  X = OWN[p] and W is smap p; or W is a pcfg of p; or ABORT, uio      shadow_sel(), claim()
 *   SHOE      X = OWN[p] and W is oe p or oe8; or W is pcfg p, IN, PP or OD;      shadow_oe(), oe8()
 *             or ABORT                                                           pcfg_*(), abort_pins()
 *   OD        W is pcfg p, IN, PP or OD                                           pcfg_*()
 *   SHBYP     W is pcfg p, PP or BYP, for a lane                                  pcfg_pp(), pcfg_byp()
 *   OUT[p]    CMT[OWN[p]]; or W is pcfg p, OD; or ABORT, uio                      commit(), pcfg_od()
 *   SEL[p]    CMT[OWN[p]]; or W is a pcfg of p; or ABORT, uio                     commit(), claim()
 *   OE        CMT[OWN[p]]; or W is pcfg p, OD; or ABORT                           commit(), pcfg_od()
 *   BYP       CMT[OWN[p]], for a lane                                             commit()
 *
 * What a pad shows is the equation of section 5.2 on these flops (pads.sv proves it), so beyond them it
 * moves only with the shift outputs LA and LB of OWN[p]'s unit, a bypass lane's input, or uo3's host
 * data. For each thread t's unit:
 *
 *   LA[t], LB[t]   change only while its send half runs or is in its tail, at CMT[t], when X = t and W
 *                  is sstop tx or sstop txrx, or at ABORT                            tx_step(), tx_stop()
 *   inputs         every step-3 strobe of unit t (scfg, txw, rxw, sstat, sgo, sstop) comes from X = t
 *                  and from a W of that instruction, and its CMT is CMT[t]           section 5.9.1
 *
 *   ABORT          from the next cycle every uio pad is undriven: OE, SHOE and the uio pins' SEL and SHSEL
 *                  are 0, and OUT and SHD are 1 on every open-drain pin (section 9.3, abort_pins())
 *
 * So no instruction or commit of a thread changes a pin it doesn't own, except by claiming it with pcfg,
 * and a unit's outputs move only by its own thread or by its own running (section 5.9.1: "Units share
 * nothing with each other").
 *
 * Each rule is checked on every cycle, reset cycles excluded (a reset may change anything to its reset
 * value, section 8). W counts as naming a pin when its bits have that row's shape, whether or not it
 * runs, so the causes are a superset of what the rows do: an instruction that stalls changes nothing.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module isolation (
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

  // Probes, connected by the proof script (probes_pins.ys, probes_iso.ys).
  wire [15:0] out_q, shd_q;
  wire [7:0]  oe_q, shoe_q, od_q, byp_q, shbyp_q;
  wire [31:0] own_q, sel_q, shsel_q;
  wire [3:0]  la, lb, run;
  wire        miso;
  wire [1:0]  th4;         // X: the thread in stage 4
  wire [15:0] ir4;         // W: the word in stage 4
  wire [3:0]  cmt;         // CMT[t]
  wire        abort;       // host ABORT
  wire        rst_ok;      // the core is out of reset
  wire [3:0]  txrun, txtl; // TXRUN, TXTL of each unit
  // The step-3 strobes and the CMT input of each unit.
  wire [3:0]  u_cfg, u_push, u_pop, u_sstat, u_sgo_tx, u_sgo_rx, u_sstop_tx, u_sstop_rx, u_cmt, u_abort;
  // For the covers: pe_exec's put strobe, its pin and value.
  wire        pin_put, pin_value;
  wire [3:0]  pin_f;

  // The shapes of W (section 6.3).
  wire       w9       = ir4[15:12] == 4'b1001;
  wire       w_put    = w9 && ir4[7:5] <= 3'd5;
  wire       w_oe     = w9 && ir4[7:5] == 3'd6;
  wire       w_cfg    = w9 && ir4[7:5] == 3'd7;
  wire [1:0] w_mode   = ir4[4:3];               // 0 IN, 1 PP, 2 OD, 3 BYP
  wire       w_edge   = ir4[15:12] == 4'b1110;
  wire       w_shift  = ir4[15:12] == 4'b1111;
  wire       w_smap   = ir4[15:9] == 7'b1101110;
  wire       w_put8   = ir4[15:7] == 9'b000001110;
  wire       w_oe8    = ir4[15:7] == 9'b000001111;
  wire       w_txw    = ir4[15:11] == 5'b11010;
  wire       w_scfg   = ir4[15:10] == 6'b110110;
  wire       w_rxw    = ir4[15:6] == 10'b1101111000;
  wire       w_sstat  = ir4[15:6] == 10'b1101111001;
  wire       w_sgo    = ir4[15:6] == 10'b1101111100;
  wire       w_sstop  = ir4[15:6] == 10'b1101111101;
  wire [3:0] w_pin    = ir4[11:8];              // put, oe, pcfg, edge
  wire [3:0] w_spin   = ir4[3:0];               // a shift's output pin
  wire [3:0] w_mpin   = ir4[8:5];               // smap

  // Every trace starts in reset: the core's flops hold their reset values in step 0 (section 8).
  always @* if ($initstate) assume (!rst_n);

  reg past_ok = 1'b0;
  always @(posedge clk) past_ok <= 1'b1;

  // The cycle before, as the causes saw it.
  reg [15:0] p_out, p_shd;
  reg [7:0]  p_oe, p_shoe, p_od, p_byp, p_shbyp;
  reg [31:0] p_own, p_sel, p_shsel;
  reg [3:0]  p_la, p_lb, p_txrun, p_txtl, p_cmt;
  reg [1:0]  p_th4;
  reg        p_abort, p_rst_ok, p_sstop_tx;

  always @(posedge clk) begin
    p_out <= out_q; p_shd <= shd_q; p_oe <= oe_q; p_shoe <= shoe_q; p_od <= od_q;
    p_byp <= byp_q; p_shbyp <= shbyp_q; p_own <= own_q; p_sel <= sel_q; p_shsel <= shsel_q;
    p_la <= la; p_lb <= lb; p_txrun <= txrun; p_txtl <= txtl; p_cmt <= cmt;
    p_th4 <= th4; p_abort <= abort; p_rst_ok <= rst_ok; p_sstop_tx <= w_sstop && ir4[4];
  end

  wire check = past_ok && p_rst_ok && rst_ok;

  genvar p;
  generate
    for (p = 0; p < 16; p = p + 1) begin : g_pin
      localparam UIO  = (p >= 8) ? 1'b1 : 1'b0;
      localparam LANE = (p == 0 || p >= 4 && p <= 7) ? 1'b1 : 1'b0;
      localparam BANK = UIO;

      wire [1:0] o     = own_q[2*p+1:2*p];
      wire       owner = th4 == o;
      wire       cfg_p = w_cfg && w_pin == p;
      wire       ab    = UIO && abort;

      wire c_own   = cfg_p;
      wire c_shd   = owner && ((w_put || w_edge) && w_pin == p || w_shift && w_spin == p ||
                               w_put8 && ir4[0] == BANK) || cfg_p && w_mode == 2'd2 || ab;
      wire c_shsel = owner && w_smap && w_mpin == p || cfg_p || ab;
      wire c_shoe  = UIO && (owner && (w_oe && w_pin == p || w_oe8) || cfg_p && w_mode != 2'd3 || abort);
      wire c_od    = UIO && cfg_p && w_mode != 2'd3;
      wire c_shbyp = LANE && cfg_p && w_mode[0];
      wire c_out   = cmt[o] || cfg_p && w_mode == 2'd2 || ab;
      wire c_sel   = cmt[o] || cfg_p || ab;
      wire c_oe    = UIO && (cmt[o] || cfg_p && w_mode == 2'd2 || abort);
      wire c_byp   = LANE && cmt[o];

      reg pc_own, pc_shd, pc_shsel, pc_shoe, pc_od, pc_shbyp, pc_out, pc_sel, pc_oe, pc_byp;
      always @(posedge clk) begin
        pc_own <= c_own; pc_shd <= c_shd; pc_shsel <= c_shsel; pc_shoe <= c_shoe; pc_od <= c_od;
        pc_shbyp <= c_shbyp; pc_out <= c_out; pc_sel <= c_sel; pc_oe <= c_oe; pc_byp <= c_byp;
      end

      always @* begin
        if (check) begin
          assert (own_q[2*p+1:2*p] == p_own[2*p+1:2*p] || pc_own && own_q[2*p+1:2*p] == p_th4);
          assert (shd_q[p] == p_shd[p] || pc_shd);
          assert (shsel_q[2*p+1:2*p] == p_shsel[2*p+1:2*p] || pc_shsel);
          assert (out_q[p] == p_out[p] || pc_out);
          assert (sel_q[2*p+1:2*p] == p_sel[2*p+1:2*p] || pc_sel);
        end
      end
      if (p >= 8) begin : g_uio
        always @* begin
          if (check) begin
            assert (shoe_q[p-8] == p_shoe[p-8] || pc_shoe);
            assert (od_q[p-8] == p_od[p-8] || pc_od);
            assert (oe_q[p-8] == p_oe[p-8] || pc_oe);
          end
        end
      end
      if (p == 0 || p >= 4 && p <= 7) begin : g_lane
        always @* begin
          if (check) begin
            assert (shbyp_q[p] == p_shbyp[p] || pc_shbyp);
            assert (byp_q[p] == p_byp[p] || pc_byp);
          end
        end
      end

      // Covers: the triggers of the rules happen. uo3 is the host's (section 5.1): no legal word names it (it
      // is "no pin" for put and edge, and no pcfg mode takes it) and its OUT, SHD, SEL and SHSEL are held at 0,
      // so none of these triggers can happen there, and it has no covers.
      if (p != 3) begin : g_cov
      always @* begin
        if (check) begin
          // A claim of another thread's pin.
          cover (own_q[2*p+1:2*p] != p_own[2*p+1:2*p] && p_own[2*p+1:2*p] != p_th4);
          // A commit of a thread that doesn't own the pin, with SHD and OUT apart, and OUT holds.
          cover (|(p_cmt & ~(4'b0001 << p_own[2*p+1:2*p])) && p_shd[p] != p_out[p] && out_q[p] == p_out[p]);
          // The owner's commit moves the pin.
          cover (out_q[p] != p_out[p] && !pc_shd);
        end
        // A put from a thread that doesn't own the pin, with a value that would change SHD.
        if (rst_ok) cover (pin_put && pin_f == p && th4 != o && pin_value != shd_q[p]);
      end
      end
    end
  endgenerate

  // ABORT (section 9.3, abort_pins()): from the next cycle every uio pad is undriven, with OE, SHOE and the
  // uio pins' output switches cleared and every open-drain pin's OUT and SHD at 1, whatever else that cycle
  // did. "So every uio pad is undriven".
  always @* begin
    if (check && p_abort) begin
      abort_undriven: assert (uio_oe == 8'h00);
      abort_state: assert (oe_q == 8'h00 && shoe_q == 8'h00 && sel_q[31:16] == 16'h0000 && shsel_q[31:16] == 16'h0000 &&
                           (out_q[15:8] & od_q) == od_q && (shd_q[15:8] & od_q) == od_q);
    end
    // Its cover, a driven pad released by ABORT, is in covers_deep.sv.
  end

  // The units.
  genvar t;
  generate
    for (t = 0; t < 4; t = t + 1) begin : g_unit
      always @* begin
        if (check) begin
          assert ((la[t] == p_la[t] && lb[t] == p_lb[t]) || p_txrun[t] || p_txtl[t] || p_cmt[t] ||
                  p_th4 == t && p_sstop_tx || p_abort);
          // The covers of an output moving at a start and while the send half runs need longer traces:
          // covers_deep.sv, for unit 0.
        end
        if (rst_ok) begin
          assert (u_cmt[t] == cmt[t]);
          assert (u_abort[t] == abort);
          assert (!u_cfg[t]      || th4 == t && w_scfg);
          assert (!u_push[t]     || th4 == t && w_txw);
          assert (!u_pop[t]      || th4 == t && w_rxw);
          assert (!u_sstat[t]    || th4 == t && w_sstat);
          assert (!u_sgo_tx[t]   || th4 == t && w_sgo && ir4[4]);
          assert (!u_sgo_rx[t]   || th4 == t && w_sgo && ir4[5]);
          assert (!u_sstop_tx[t] || th4 == t && w_sstop && ir4[4]);
          assert (!u_sstop_rx[t] || th4 == t && w_sstop && ir4[5]);
          cover (u_cfg[t]);
          cover (u_push[t]);
          cover (u_sgo_tx[t]);
          cover (u_sstop_rx[t]);
        end
      end
    end
  endgenerate

endmodule

`default_nettype wire
