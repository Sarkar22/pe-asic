// J' (rk3_equiv.sv; 5.9.6, the Period bullet) on one shift unit (src/pe_shift.sv as the flow reads it, every input free
// in every cycle, any scfg in any cycle, mid-reception writes of RXPER and RXOPT included), by k-induction, with
// RKIND 3's p RXP3 and its RGAIN RXRG as the reception holds them, and the window points WQ3 and WH3 against RXP3 in
// every cycle. No assumption: a reception of RKIND 3 keeps the p, RGAIN and RACQ it started with (5.9.6, Start).
// And a reception takes RXP3, RXRG and RXRA from the slots as the cycle it starts in reads them, before that
// cycle's own scfg (3.2), so a write of RXPER or RXOPT in that very cycle acts from the next reception on.
module rk3_unit (
    input wire        clk,
    input wire        rst_n,
    input wire [15:0] sync2,
    input wire [15:0] sync3,
    input wire [15:0] sync2f,
    input wire        cmt,
    input wire        cfg_we,
    input wire [3:0]  cfg_slot,
    input wire [15:0] cfg_data,
    input wire        push,
    input wire [15:0] push_v,
    input wire [3:0]  push_n,
    input wire        push_l,
    input wire [1:0]  push_k,
    input wire        sgo_tx,
    input wire        sstop_tx,
    input wire        sgo_rx,
    input wire        sstop_rx,
    input wire        pop,
    input wire        sstat_rd,
    input wire        h_abort,
    input wire [1:0]  srd_sel,
    input wire [4:0]  rd_reg
);
  wire        la, lb, pad_a, pad_b, txfull, rxfull, rxbl;
  wire [15:0] rxbuf, status, srd_data, rd_data;
  pe_shift u (
      .clk(clk), .rst_n(rst_n), .sync2(sync2), .sync3(sync3), .sync2f(sync2f), .cmt(cmt),
      .cfg_we(cfg_we), .cfg_slot(cfg_slot), .cfg_data(cfg_data),
      .push(push), .push_v(push_v), .push_n(push_n), .push_l(push_l), .push_k(push_k),
      .sgo_tx(sgo_tx), .sstop_tx(sstop_tx), .sgo_rx(sgo_rx), .sstop_rx(sstop_rx),
      .pop(pop), .sstat_rd(sstat_rd), .h_abort(h_abort), .srd_sel(srd_sel), .rd_reg(rd_reg),
      .la(la), .lb(lb), .pad_a(pad_a), .pad_b(pad_b), .txfull(txfull), .rxfull(rxfull), .rxbuf(rxbuf), .rxbl(rxbl),
      .status(status), .srd_data(srd_data), .rd_data(rd_data)
  );
  // probes (probes_rk3.ys)
  wire               rxrun, rxep, op, cd;
  wire [1:0]         kind;
  wire signed [24:0] ph;
  wire signed [12:0] ce;
  wire [4:0]         g;
  wire [11:0]        p3, wq3, wh3;
  wire [15:0]        rxper, rxopt;

  always @* if ($initstate) assume (!rst_n);
  reg past_ok = 1'b0;
  always @(posedge clk) past_ok <= 1'b1;
  // no assumption: a reception of RKIND 3 keeps the p, RGAIN and RACQ it started with (P3, RG3, RA3)

  wire [11:0] pw   = (rxper[15:1] == 15'd0) ? 12'd512 : {((|rxper[15:4]) ? 4'd15 : rxper[3:0]), rxopt[7:0]};
  wire [1:0]  rg;
  wire signed [26:0] p  = $signed({15'd0, p3});
  wire signed [26:0] x  = {{2{ph[24]}}, ph};
  wire signed [26:0] lo = op ? (27'sd256 - p) : (27'sd128 - p);   // J': from 128 - p with the window closed
  wire signed [26:0] hi = (rg != 2'd0) ? (p + 27'sd128) : (((p * 7) >>> 2) - 27'sd128);
  always @* begin
    if (past_ok && rst_n) begin
      if (rxrun) assert (p3 >= 12'd512);
      assert (wq3 == p3 - {2'b00, p3[11:2]});
      assert (wh3 == p3 - {1'b0, p3[11:1]});
      assert (!(rxrun && rxep));          // a helper: a reception that ends with its pattern found holds its word
      if (rxrun && !rxep && kind == 2'd3) begin
        assert (g >= 5'd1 && g <= 5'd16);
        assert (!cd || op);
        assert (x >= lo && x < hi);
        if (cd) assert ($signed({{14{ce[12]}}, ce}) >= 27'sd256 - p && ce < 13'sd0);
      end
    end
  end
  // the bounds are reached: an open window below -p/2, a closed one at p + 128 or more, RXCE below -2048
  always @* if (past_ok && rst_n && rxrun && kind == 2'd3) begin
    cover (op && x < -(p >>> 1));
    cover (!op && x >= p + 27'sd128);
  end
  // the start: p, RGAIN and RACQ from the slots as the start cycle reads them (3.2: step 2b reads the flops as they
  // were at the start of the cycle, before step 3's scfg); the cover reaches a start in the cycle of such a write
  wire        beg, ra;                      // probes: the cycle a reception starts (rx_begin); RXRA
  reg         beg_p = 1'b0, rst_p = 1'b0, we_p = 1'b0;
  reg  [15:0] rxper_p = 16'd0, rxopt_p = 16'd0;
  always @(posedge clk) begin
    beg_p <= beg; rst_p <= rst_n; rxper_p <= rxper; rxopt_p <= rxopt;
    we_p  <= cfg_we && (cfg_slot == 4'd5 || cfg_slot == 4'd10);
  end
  wire [11:0] pw_p = (rxper_p[15:1] == 15'd0) ? 12'd512 : {((|rxper_p[15:4]) ? 4'd15 : rxper_p[3:0]), rxopt_p[7:0]};
  always @* if (past_ok && rst_p && rst_n && beg_p) begin
    assert (p3 == pw_p);
    assert (rg == rxopt_p[10:9] && ra == rxopt_p[11]);
  end
  always @* if (past_ok && rst_p && rst_n) cover (beg_p && we_p);
endmodule
