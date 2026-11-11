/*
 * The shift unit's exact timing (section 5.9), on one unit with every input free in every cycle: any
 * configuration, any instruction strobe, any CMT, any ABORT, any input pins. All four units are this one
 * module, and isolation.sv proves that unit t's CMT is thread t's commit and that its strobes come only from
 * thread t's instructions.
 *
 *   start_cmt    The send half starts only at a CMT: TXRUN rises only in the cycle after one in which TXARM,
 *                CMT, no tail and no stop (section 5.9.5, tx_step(); section 5.9.1, "A unit starts only at
 *                a commit of its own thread").
 *   start_sym    "The commit at E writes the first symbol" (section 5.9.5, the grid): in the commit's cycle
 *                tx_symbol() runs, and from the next cycle LA and LB show symbol 0 of the first bit, as
 *                MAPA, MAPB, TGLA and TGLB give it, and TXRUN is 1.
 *   start_empty  "A start with no word does the same" as an underrun: TXUR sets and the outputs go to the
 *                end levels, or to the idle levels with TAIL 0 (section 5.9.5, Underrun).
 *   sym_tper     With TFRAC 0, a symbol written in cycle w lasts exactly TPER cycles, TPER 0 counted as 1:
 *                the next symbol of the send is written in cycle w + TPER (section 5.9.5, the grid: "for
 *                exactly TPER cycles plus any fraction carry").
 *   sym_frac     With any TFRAC, the length is TPER plus the carry of TXACC + TFRAC, with TXACC starting at
 *                0 at each start, as tx_symbol() defines it (TPER 0: one cycle, TXACC left alone).
 *   hold         Between those cycles LA and LB hold: they change only in a cycle after a symbol write,
 *                the end of a send, the end of a tail, sstop tx or ABORT.
 *   tail_len     After the last symbol the outputs show ENDA and ENDB for exactly TAIL cycles, then IDLEA and
 *                IDLEB (section 5.9.5, The end).
 *   stop_tx      After sstop tx or ABORT in cycle c, from c + 1 the send half is idle with both buffers empty
 *                and shows its idle levels (section 5.9.4, sstop; section 9.3, ABORT runs tx_stop()).
 *   stop_rx      After sstop rx or ABORT in cycle c, from c + 1 the receive half is not armed, hunting,
 *                receiving or holding a word, and its buffer is empty (rx_stop()).
 *   rx_start     A reception starts only at a commit (without HUNT) or at the hunt's edge, and a commit that
 *                starts one does it in its own cycle (section 5.9.6, Start).
 *
 * A symbol is "written" in a cycle where tx_symbol() runs; the probe do_sym is the unit's own name for it.
 * The checks of a symbol's length leave out symbols during which TEXT was 1, since those end at an
 * outside clock edge (section 5.9.5, Outside clock). The configuration may change in any cycle: each check
 * uses the configuration the unit used.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module shift_timing (
    input wire        clk,
    input wire        rst_n,
    input wire [15:0] sync2,
    input wire [15:0] sync3,
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

  wire        la, lb, txfull, rxfull, rxbl;
  wire [15:0] rxbuf, status, srd_data, rd_data;

  pe_shift u (
      .clk(clk), .rst_n(rst_n), .sync2(sync2), .sync3(sync3), .cmt(cmt),
      .cfg_we(cfg_we), .cfg_slot(cfg_slot), .cfg_data(cfg_data),
      .push(push), .push_v(push_v), .push_n(push_n), .push_l(push_l), .push_k(push_k),
      .sgo_tx(sgo_tx), .sstop_tx(sstop_tx), .sgo_rx(sgo_rx), .sstop_rx(sstop_rx),
      .pop(pop), .sstat_rd(sstat_rd), .h_abort(h_abort), .srd_sel(srd_sel), .rd_reg(rd_reg),
      .la(la), .lb(lb), .txfull(txfull), .rxfull(rxfull), .rxbuf(rxbuf), .rxbl(rxbl),
      .status(status), .srd_data(srd_data), .rd_data(rd_data)
  );

  // Probes, connected by probes_shift.ys: the unit's flops and the cycles of tx_symbol() and tx_end().
  wire        run, tl, arm, ur, full, dosym, doend;
  wire [16:0] cd;
  wire [7:0]  acc;
  wire [4:0]  n, bn;
  wire [15:0] sr, bufw;
  wire [1:0]  k, bk, sym;
  wire [15:0] txmode, txmap, txper, txtail, coding, rxmode;
  wire        rxarm, rxhunt, rxrun, rxep, rxfullq, rxvio, rxseen;

  // Configuration fields (section 5.9.3).
  wire       tmsb  = txmode[2];
  wire       tgla  = txmode[3];
  wire       tglb  = txmode[4];
  wire       idlea = txmode[5];
  wire       idleb = txmode[6];
  wire       enda  = txmode[7];
  wire       endb  = txmode[8];
  wire       text  = txmode[9];
  wire [7:0] mapa  = txmap[7:0];
  wire [7:0] mapb  = txmap[15:8];
  wire [15:0] tper = txper;
  wire [7:0] tail  = txtail[7:0];
  wire [7:0] tfrac = txtail[15:8];
  wire       cinit = coding[5];
  wire [1:0] cmode = coding[8:7];
  wire       ctx   = coding[9];
  wire       hunt  = rxmode[5];
  wire       hlvl  = rxmode[6];
  wire [3:0] rpin  = rxmode[11:8];

  always @* if ($initstate) assume (!rst_n);

  reg past_ok = 1'b0;
  always @(posedge clk) past_ok <= 1'b1;

  wire stop_tx = sstop_tx || h_abort;
  wire stop_rx = sstop_rx || h_abort;

  // --- The start (tx_step(), idle branch). tx_load() runs first if the shift register is empty.
  wire        ld     = n == 5'd0 && full;
  wire [4:0]  n_ld   = ld ? bn : n;
  wire [15:0] sr_ld  = ld ? bufw : sr;
  wire [1:0]  k_ld   = ld ? bk : k;
  wire        go     = arm && !run && !tl && cmt && !stop_tx;
  // The first bit: bit n - 1 or bit 0, scrambled by bit 31 of crc_init() in LFSR mode. That bit is CINIT.
  wire        b0     = (tmsb ? sr_ld[n_ld[3:0] - 4'd1] : sr_ld[0]) ^ (cmode == 2'd2 && ctx && k_ld >= 2'd2 && cinit);
  wire [1:0]  ca     = mapa[1:0];
  wire [1:0]  cb     = mapb[1:0];
  wire        va     = ca == 2'd0 ? b0 : ca == 2'd1 ? !b0 : ca[0];
  wire        vb     = cb == 2'd0 ? b0 : cb == 2'd1 ? !b0 : cb[0];
  wire        la0    = tgla ? la ^ va : va;
  wire        lb0    = tglb ? lb ^ vb : vb;

  // --- The previous cycle.
  reg        p_rst_n, p_run, p_tl, p_arm, p_cmt, p_stop_tx, p_stop_rx, p_dosym, p_doend, p_la, p_lb;
  reg        p_go, p_go_word, p_la0, p_lb0, p_idlea, p_idleb, p_enda, p_endb, p_tail0, p_ur, p_sstat;
  reg        p_sstop_tx_only, p_tail_end, p_rx_go, p_rx_hunt_go, p_rxseen, p_rxrun;
  reg [16:0] p_cd1;
  reg        p_abort;

  wire tail_end = tl && !run && cd <= 17'd1;
  wire rx_idle  = !rxep && !rxrun && !rxhunt;
  wire rx_go    = rx_idle && rxarm && cmt && !hunt && !stop_rx;
  wire rx_hunt_go = !rxep && !rxrun && rxhunt && sync2[rpin] == hlvl && sync3[rpin] != hlvl && !stop_rx;

  always @(posedge clk) begin
    p_rst_n <= rst_n; p_run <= run; p_tl <= tl; p_arm <= arm; p_cmt <= cmt; p_stop_tx <= stop_tx;
    p_stop_rx <= stop_rx; p_dosym <= dosym; p_doend <= doend; p_la <= la; p_lb <= lb;
    p_go <= go; p_go_word <= n_ld != 5'd0; p_la0 <= la0; p_lb0 <= lb0;
    p_idlea <= idlea; p_idleb <= idleb; p_enda <= enda; p_endb <= endb; p_tail0 <= tail == 8'd0;
    p_ur <= ur; p_sstat <= sstat_rd; p_tail_end <= tail_end;
    // sstop tx alone: no txw, sgo tx or TXMODE write in its cycle. The core never pairs them: isolation.sv
    // proves each strobe comes from a word of its own row, and one cycle has one word.
    p_sstop_tx_only <= sstop_tx && !h_abort && !push && !sgo_tx && !(cfg_we && cfg_slot == 4'd0);
    p_rx_go <= rx_go; p_rx_hunt_go <= rx_hunt_go; p_rxseen <= rxseen; p_rxrun <= rxrun;
    p_cd1 <= (tper == 16'd0) ? 17'd1 : {1'b0, tper};
    p_abort <= h_abort;
  end

  wire check = past_ok && rst_n && p_rst_n;

  always @* begin
    if (check) begin
      start_cmt: assert (!(run && !p_run) || p_cmt && p_arm && !p_tl && !p_stop_tx);
      if (p_go && p_go_word) begin
        start_sym: assert (p_dosym && run && sym == 2'd0 && la == p_la0 && lb == p_lb0);
        start_len: assert (cd == p_cd1);
      end
      if (p_go && !p_go_word) begin
        start_empty: assert (!run && (ur || p_sstat && p_ur) &&
                             (p_tail0 ? !tl && la == p_idlea && lb == p_idleb : tl && la == p_enda && lb == p_endb));
      end
      hold: assert (la == p_la && lb == p_lb || p_dosym || p_doend || p_tail_end || p_stop_tx);
      if (p_sstop_tx_only || p_abort) begin
        stop_tx_idle: assert (!arm && !run && !tl && n == 5'd0 && !full && la == idlea && lb == idleb && !status[5]);
      end
      if (p_stop_rx) begin
        stop_rx_idle: assert (!rxarm && !rxhunt && !rxrun && !rxep && !rxfullq && !rxvio && !status[15] &&
                              !status[8] && !status[9]);
      end
      rx_start: assert (!(rxrun && !p_rxrun) || p_rx_go || p_rx_hunt_go);
      if (p_rx_go) rx_start_seen: assert (rxseen || p_sstat && p_rxseen);
    end
  end

  // --- The tail monitor's state, declared here for the invariants below.
  reg        t_on;
  reg [16:0] t_cnt;
  reg [7:0]  t_len;
  reg        enda_at_end, endb_at_end;   // ENDA and ENDB as tx_end() used them

  // --- Symbol lengths. The monitor follows the symbol last written in this send.
  reg        m_on;      // a symbol of this send is followed
  reg        m_clean;   // TEXT has been 0 since it was written
  reg [16:0] m_cnt;     // cycles since it was written
  reg [16:0] m_len;     // TPER + carry, or 1 with TPER 0
  reg [16:0] m_tper;    // TPER as it was then, 0 counted as 1
  reg        m_frac0;   // TFRAC was 0 then
  reg [7:0]  m_acc;     // TXACC after it, by section 5.9.5

  wire        first   = dosym && !run;
  wire [7:0]  acc_in  = first ? 8'd0 : m_acc;
  wire [8:0]  acc_sum = {1'b0, acc_in} + {1'b0, tfrac};
  wire [16:0] len_new = (tper == 16'd0) ? 17'd1 : {1'b0, tper} + {16'd0, acc_sum[8]};

  always @(posedge clk) begin
    if (!rst_n) begin
      m_on <= 1'b0;
    end else if (dosym) begin
      m_on    <= 1'b1;
      m_clean <= 1'b1;
      m_cnt   <= 17'd1;
      m_len   <= len_new;
      m_tper  <= (tper == 16'd0) ? 17'd1 : {1'b0, tper};
      m_frac0 <= tfrac == 8'd0;
      m_acc   <= (tper == 16'd0) ? acc_in : acc_sum[7:0];
    end else begin
      if (m_cnt != 17'h1FFFF) m_cnt <= m_cnt + 17'd1;
      if (text) m_clean <= 1'b0;
    end
  end

  wire follow = m_on && run && m_clean;

  always @* begin
    if (rst_n && dosym && follow && !text) begin
      sym_frac: assert (m_cnt == m_len);
      if (m_frac0) sym_tper: assert (m_cnt == m_tper);
    end
    // Invariants that make the induction close. They are proved like the rest.
    if (rst_n && follow) inv_cnt: assert ({1'b0, m_cnt} + {1'b0, cd} == {1'b0, m_len} + 18'd1 && m_cnt <= m_len);
    if (rst_n && m_on && run) inv_acc: assert (m_acc == acc);
    if (rst_n) inv_on: assert (!run || m_on);
    if (rst_n) inv_ton: assert (!(tl && !run) || t_on);
    if (rst_n) inv_excl: assert (!(run && tl));
    if (rst_n && m_on) inv_len: assert ((m_len == m_tper || m_len == m_tper + 17'd1) && (!m_frac0 || m_len == m_tper));
  end

  // --- The tail.

  always @(posedge clk) begin
    if (!rst_n) begin
      t_on <= 1'b0;
    end else if (doend) begin
      t_on  <= tail != 8'd0;
      t_cnt <= 17'd1;
      t_len <= tail;
    end else begin
      if (t_cnt != 17'h1FFFF) t_cnt <= t_cnt + 17'd1;
    end
  end

  always @* begin
    if (rst_n && t_on && tl && !run) begin
      inv_tail: assert ({1'b0, t_cnt} + {1'b0, cd} == {10'd0, t_len} + 18'd1 && t_cnt <= {9'd0, t_len});
      tail_ends: assert (la == enda_at_end && lb == endb_at_end);
      if (cd <= 17'd1) tail_len: assert (t_cnt == {9'd0, t_len});
    end
  end

  always @(posedge clk) if (doend) begin enda_at_end <= enda; endb_at_end <= endb; end

  always @* begin
    if (check && p_tail_end && !p_stop_tx && !p_dosym) tail_idle: assert (la == p_idlea && lb == p_idleb && !tl);
  end

  // --- Covers: each check's trigger happens.
  always @* begin
    if (check) begin
      cover (p_go && p_go_word && la != p_la);
      cover (p_go && !p_go_word && !p_tail0);
      cover (p_sstop_tx_only && p_run && la != p_la);
      cover (p_abort && p_run);
      cover (p_stop_rx && p_rxrun);
      cover (p_rx_go && rxrun);
      cover (p_rx_hunt_go && rxrun);
    end
    if (rst_n && dosym && follow && !text) begin
      cover (m_frac0 && m_tper == 17'd3);
      cover (!m_frac0 && m_len == m_tper + 17'd1);
      cover (!m_frac0 && m_len == m_tper && m_tper == 17'd2);
    end
    if (rst_n && t_on && tl && !run && cd <= 17'd1) cover (t_len == 8'd3);
  end

endmodule

`default_nettype wire
