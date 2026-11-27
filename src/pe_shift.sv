/*
 * One thread's shift unit (section 5.9): the send half (section 5.9.5), the receive half
 * (section 5.9.6), the CRC register, the stuffer and the destuffer (section 5.9.7), the 16
 * configuration slots (section 5.9.3), and the unit's side of the instructions (section 5.9.4),
 * of host ABORT (section 9.3) and of RDREG (section 9.2).
 *
 * Cycle order (section 3.2). Step 2b runs tx_step() and rx_step() on the flops as they were at
 * the start of the cycle, and on CMT. The two halves share no flop: the CRC register belongs to
 * the half that CTX names, so its next value is the send half's or the receive half's, chosen by
 * CTX. Step 3, this thread's instruction, reads the flops as they were at the start of the cycle,
 * and where it writes a flop that step 2b also writes (sstop, sstat) the instruction wins. Host
 * ABORT takes effect after step 5: it runs tx_stop() and rx_stop() on top of everything else,
 * with the configuration as this cycle's scfg leaves it. Every output comes from flops, so step 3
 * and RDREG see the unit as it was at the start of the cycle.
 *
 * The receive half holds p - p/4 and p - p/2, the points where a Manchester window opens, in
 * flops of their own, and forms the decoder's arithmetic in 17 bits; RKIND 3 keeps its own p and
 * window points from a reception's start, in 15 bits (the implementation note of section 5.9.6).
 *
 * Each CUT_ parameter, set to 1, leaves one piece out: its configuration fields become reserved
 * (not stored, read 0) and its state never sets. With all of them 0 the unit is section 5.9 as
 * written.
 *   CUT_OCK     outside clocks: TEXT, TFALL, TCLK and RFALL, and RCLK too unless the end test
 *               stays. RKIND 2 acts as 0
 *   CUT_TFRAC   the send fraction TFRAC
 *   CUT_RESYNC  RESYNC
 *   CUT_RFRAC   the receive fraction RFRAC
 *   CUT_CRC     the CRC register: CSKIP, POLYHI to CHKLO and CODING bits 9:0, the state CRC, TXAN
 *               and RXCOV. srd lo and hi read 0, and the LFSR mode goes with it
 *   CUT_LFSR    the LFSR mode: CMODE 2 acts as 0
 *   CUT_STUFF   the stuffer and the destuffer: SMODE and SN, the state TXSTF, TXRL, TXRV, RXRL,
 *               RXRV and RXVIO
 *   CUT_EOP     REOP: the end test and both byte ends, the Manchester one too (section 5.9.7)
 *
 * Ports:
 *   clk        clock
 *   rst_n      reset, asynchronous, active low
 *   sync2      SYNC2: {uio_in, ui_in} after the synchroniser
 *   sync3      SYNC3: SYNC2 one cycle older
 *   sync2f     SYNC2F: the falling-edge sample, half a cycle after SYNC3's and before SYNC2's (section 3.1)
 *   cmt        CMT: this thread's prepared pins commit in this cycle (section 5.4)
 *   cfg_we     scfg runs in this cycle: set_cfg(cfg_slot, cfg_data)
 *   cfg_slot   scfg's slot k
 *   cfg_data   scfg's value, Rs
 *   push       txw runs in this cycle, TXFULL 0 at its start: tx_push(push_v, n, push_l, push_k)
 *   push_v     txw's word, Rs
 *   push_n     txw's bit count n, mod 16
 *   push_l     txw's last flag
 *   push_k     txw's kind
 *   sgo_tx     sgo with bit 0 of m set
 *   sstop_tx   sstop with bit 0 of m set
 *   sgo_rx     sgo with bit 1 of m set
 *   sstop_rx   sstop with bit 1 of m set
 *   pop        rxw runs in this cycle, RXFULL 1 at its start: rx_pop()
 *   sstat_rd   sstat runs in this cycle: it clears the sticky bits it reads as 1
 *   h_abort    host ABORT in this cycle
 *   srd_sel    srd's register s: 0 lo, 1 hi, 2 end
 *   rd_reg     RDREG's register number, bits 4:0
 *   la         LA, shift output A
 *   lb         LB, shift output B
 *   pad_a      shift output A as the pads show it: V0's level in the first half of the cycle, LA's in the
 *              second (section 5.2, two halves)
 *   pad_b      the same for shift output B
 *   txfull     TXFULL, which txw's need() tests
 *   rxfull     RXFULL, which rxw's need() tests
 *   rxbuf      RXBUF, the word rxw takes
 *   rxbl       RXBL, the T that rxw sets
 *   status     the status word that sstat reads (section 5.9.4); its T is bit 7 | bit 10
 *   srd_data   what srd reads for srd_sel
 *   rd_data    what RDREG reads for rd_reg 12 to 31, and 0 for rd_reg 0 to 11
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_shift #(
    parameter logic CUT_OCK    = 1'b0,
    parameter logic CUT_TFRAC  = 1'b0,
    parameter logic CUT_RESYNC = 1'b0,
    parameter logic CUT_RFRAC  = 1'b0,
    parameter logic CUT_CRC    = 1'b0,
    parameter logic CUT_LFSR   = 1'b0,
    parameter logic CUT_STUFF  = 1'b0,
    parameter logic CUT_EOP    = 1'b0
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [15:0] sync2,
    input  logic [15:0] sync3,
    input  logic [15:0] sync2f,
    input  logic        cmt,
    // Step 3: this thread's instruction. A strobe is set only in the cycle its row runs.
    input  logic        cfg_we,
    input  logic [3:0]  cfg_slot,
    input  logic [15:0] cfg_data,
    input  logic        push,
    input  logic [15:0] push_v,
    input  logic [3:0]  push_n,
    input  logic        push_l,
    input  logic [1:0]  push_k,
    input  logic        sgo_tx,
    input  logic        sstop_tx,
    input  logic        sgo_rx,
    input  logic        sstop_rx,
    input  logic        pop,
    input  logic        sstat_rd,
    // The host.
    input  logic        h_abort,
    // Reads, from the flops at the start of the cycle.
    input  logic [1:0]  srd_sel,
    input  logic [4:0]  rd_reg,
    output logic        la,
    output logic        lb,
    output logic        pad_a,
    output logic        pad_b,
    output logic        txfull,
    output logic        rxfull,
    output logic [15:0] rxbuf,
    output logic        rxbl,
    output logic [15:0] status,
    output logic [15:0] srd_data,
    output logic [15:0] rd_data
);

  // ============================================================================================
  // Configuration (section 5.9.3). KEEP[k] of section 5.9.4 masks each write, so reserved bits
  // aren't stored and read 0. A cut piece's fields are reserved too.

  localparam logic [15:0] KEEP0  = CUT_OCK ? 16'h81FF : 16'hFFFF;
  localparam logic [15:0] KEEP3  = CUT_TFRAC ? 16'h00FF : 16'hFFFF;
  localparam logic [15:0] KEEP4  = 16'hFFFF & ~(CUT_OCK ? 16'h0080 : 16'h0000)
                                           & ~((CUT_OCK && CUT_EOP) ? 16'hF000 : 16'h0000);
  localparam logic [15:0] KEEP10 = 16'hFFFF & ~(CUT_CRC ? 16'hF000 : 16'h0000)
                                           & ~(CUT_RESYNC ? 16'h0100 : 16'h0000)
                                           & ~(CUT_RFRAC ? 16'h00FF : 16'h0000);
  localparam logic [15:0] KEEP11 = CUT_CRC ? 16'h0000 : 16'hFFFF;    // slots 11 to 14
  localparam logic [15:0] KEEP15 = 16'hFFFF & ~(CUT_CRC ? 16'h03FF : 16'h0000)
                                           & ~(CUT_STUFF ? 16'h7C00 : 16'h0000)
                                           & ~(CUT_EOP ? 16'h8000 : 16'h0000);

  logic [15:0] txmode_q, txmap_q, txper_q, txtail_q;
  logic [15:0] rxmode_q, rxper_q, rxfirst_q, rxend_q, rxcnt_q, rxsync_q, rxopt_q;
  logic [15:0] polyhi_q, polylo_q, chkhi_q, chklo_q, coding_q;

  // Send fields.
  logic [1:0]  syms;
  logic        tmsb, tgla, tglb, idlea, idleb, enda, endb, text, tfall, thalf;
  logic [3:0]  tclk;
  logic [7:0]  mapa, mapb;
  logic [15:0] tper;
  logic [7:0]  tail, tfrac;

  assign syms  = txmode_q[1:0];
  assign tmsb  = txmode_q[2];
  assign tgla  = txmode_q[3];
  assign tglb  = txmode_q[4];
  assign idlea = txmode_q[5];
  assign idleb = txmode_q[6];
  assign enda  = txmode_q[7];
  assign endb  = txmode_q[8];
  assign text  = CUT_OCK ? 1'b0 : txmode_q[9];
  assign tfall = CUT_OCK ? 1'b0 : txmode_q[10];
  assign tclk  = CUT_OCK ? 4'd0 : txmode_q[14:11];
  assign thalf = txmode_q[15];
  assign mapa  = txmap_q[7:0];
  assign mapb  = txmap_q[15:8];
  assign tper  = txper_q;
  assign tail  = txtail_q[7:0];
  assign tfrac = CUT_TFRAC ? 8'd0 : txtail_q[15:8];

  // Receive fields.
  logic [1:0]  rkind;
  logic        rmsb, rinv, rdiff, hunt, hlvl, rfall;
  logic [3:0]  rpin, rclk;
  logic [15:0] rper, first, rend, sync;
  logic [10:0] rcnt;
  logic [4:0]  synclen;
  logic [7:0]  rfrac;
  logic        resync, racq;
  logic [1:0]  rgain;
  logic [3:0]  cskip;

  assign rkind   = rxmode_q[1:0];
  assign rmsb    = rxmode_q[2];
  assign rinv    = rxmode_q[3];
  assign rdiff   = rxmode_q[4];
  assign hunt    = rxmode_q[5];
  assign hlvl    = rxmode_q[6];
  assign rfall   = CUT_OCK ? 1'b0 : rxmode_q[7];
  assign rpin    = rxmode_q[11:8];
  assign rclk    = (CUT_OCK && CUT_EOP) ? 4'd0 : rxmode_q[15:12];
  assign rper    = rxper_q;
  assign first   = rxfirst_q;
  assign rend    = rxend_q;
  assign rcnt    = rxcnt_q[10:0];
  assign synclen = rxcnt_q[15:11];
  assign sync    = rxsync_q;
  assign rfrac   = CUT_RFRAC ? 8'd0 : rxopt_q[7:0];
  assign resync  = CUT_RESYNC ? 1'b0 : rxopt_q[8];
  assign rgain   = rxopt_q[10:9];
  assign racq    = rxopt_q[11];
  assign cskip   = CUT_CRC ? 4'd0 : rxopt_q[15:12];

  // CRC and stuffer fields. SNE is SN as the stuff tests read it: with SMODE 2, SN 0 and 1 act
  // as 2 (section 5.9.7).
  logic [31:0] poly, chk;
  logic [4:0]  cw;
  logic        cinit, cxor, ctx, reop;
  logic [1:0]  cmode_raw, cmode, smode;
  logic [2:0]  sn, sne;
  logic        cm1, cm2, cmode_on, smode_on;

  assign poly      = CUT_CRC ? 32'd0 : {polyhi_q, polylo_q};
  assign chk       = CUT_CRC ? 32'd0 : {chkhi_q, chklo_q};
  assign cw        = CUT_CRC ? 5'd0 : coding_q[4:0];
  assign cinit     = CUT_CRC ? 1'b0 : coding_q[5];
  assign cxor      = CUT_CRC ? 1'b0 : coding_q[6];
  assign cmode_raw = CUT_CRC ? 2'd0 : coding_q[8:7];
  assign cmode     = (CUT_LFSR && cmode_raw == 2'd2) ? 2'd0 : cmode_raw;
  assign ctx       = CUT_CRC ? 1'b0 : coding_q[9];
  assign smode     = CUT_STUFF ? 2'd0 : coding_q[11:10];
  assign sn        = CUT_STUFF ? 3'd0 : coding_q[14:12];
  assign reop      = CUT_EOP ? 1'b0 : coding_q[15];
  assign cm1       = cmode == 2'd1;
  assign cm2       = cmode == 2'd2;
  assign cmode_on  = cm1 | cm2;
  assign smode_on  = (smode == 2'd1) | (smode == 2'd2);
  // With the stuffer cut SMODE reads 0, so no stuff test reads SNE, and 7 keeps its compares
  // from being constant.
  assign sne       = CUT_STUFF ? 3'd7 : (smode == 2'd2 && sn < 3'd2) ? 3'd2 : sn;

  // IDLEA and IDLEB as this cycle's scfg leaves them, for ABORT.
  logic idlea_n, idleb_n;

  assign idlea_n = (cfg_we && cfg_slot == 4'd0) ? cfg_data[5] : idlea;
  assign idleb_n = (cfg_we && cfg_slot == 4'd0) ? cfg_data[6] : idleb;

  // crc_init()'s value: the top CW + 1 bits all ones with CINIT, otherwise 0.
  logic [31:0] crc_initv;

  always_comb begin
    for (int i = 0; i < 32; i = i + 1) crc_initv[i] = cinit & (5'(31 - i) <= cw);
  end

  logic [31:0] crc_q;
  logic [31:0] crc_shl;

  assign crc_shl = {crc_q[30:0], 1'b0};

  // ============================================================================================
  // Send half (sections 5.9.5 and 5.9.7).

  logic        tx_arm_q, tx_run_q, tx_tl_q, tx_lw_q, tx_bl_q, tx_full_q, la_q, lb_q, tx_ur_q;
  logic        tx_h_q, ola_q, olb_q;
  logic [15:0] tx_sr_q, tx_buf_q;
  logic [4:0]  tx_n_q, tx_bn_q;
  logic [1:0]  tx_sym_q, tx_k_q, tx_bk_q;
  logic [16:0] tx_cd_q;
  logic [7:0]  tx_acc_q;
  logic        tx_b_q, tx_stf_q, tx_rv_q;
  logic [5:0]  tx_an_q;
  logic [2:0]  tx_rl_q;

  // The parts of tx_done() that read only flops: tx_run_len(TXB), the stuff test of tx_stuff()
  // on its result, and crc_bit(TXB).
  logic        tx_due, tclk_edge, tx_stuff_due, tx_fb;
  logic [2:0]  tx_rl_new;
  logic [31:0] tx_crc_next;

  assign tclk_edge    = (sync2[tclk] != sync3[tclk]) && (sync2[tclk] != tfall);
  assign tx_due       = text ? tclk_edge : (tx_cd_q <= 17'd1);
  assign tx_rl_new    = (smode == 2'd1 && !tx_b_q) ? 3'd0 :
                        (tx_rl_q == 3'd0 || tx_b_q != tx_rv_q) ? 3'd1 :
                        (tx_rl_q < 3'd7) ? tx_rl_q + 3'd1 : tx_rl_q;
  assign tx_stuff_due = (tx_rl_new != 3'd0) && (tx_rl_new >= sne);
  assign tx_fb        = cm1 ? (crc_q[31] ^ tx_b_q) : crc_q[31];
  assign tx_crc_next  = crc_shl ^ (tx_fb ? poly : 32'd0);

  // tx_step() up to its call of tx_symbol() or tx_end(), in code order. A line reads what an
  // earlier line of the step wrote. crc_t is the CRC register as the send half leaves it.
  logic        tx_arm_a, tx_run_a, tx_tl_a, tx_lw_a, tx_full_a, tx_ur_a, tx_stf_a, tx_rv_a;
  logic        la_a, lb_a;
  logic [15:0] tx_sr_a;
  logic [4:0]  tx_n_a;
  logic [1:0]  tx_sym_a, tx_k_a;
  logic [16:0] tx_cd_a;
  logic [7:0]  tx_acc_a;
  logic [5:0]  tx_an_a;
  logic [2:0]  tx_rl_a;
  logic [31:0] crc_t;
  logic        do_sym, do_end, tail_end;

  always_comb begin
    tx_arm_a  = tx_arm_q;
    tx_run_a  = tx_run_q;
    tx_tl_a   = tx_tl_q;
    tx_lw_a   = tx_lw_q;
    tx_full_a = tx_full_q;
    tx_ur_a   = tx_ur_q;
    tx_stf_a  = tx_stf_q;
    tx_rv_a   = tx_rv_q;
    la_a      = la_q;
    lb_a      = lb_q;
    tx_sr_a   = tx_sr_q;
    tx_n_a    = tx_n_q;
    tx_sym_a  = tx_sym_q;
    tx_k_a    = tx_k_q;
    tx_cd_a   = tx_cd_q;
    tx_acc_a  = tx_acc_q;
    tx_an_a   = tx_an_q;
    tx_rl_a   = tx_rl_q;
    crc_t     = crc_q;
    do_sym    = 1'b0;
    do_end    = 1'b0;
    tail_end  = 1'b0;
    if (tx_run_q) begin
      // tx_run()
      if (!tx_due) begin
        if (!text) tx_cd_a = tx_cd_q - 17'd1;
      end else if (tx_sym_q < syms) begin
        tx_sym_a = tx_sym_q + 2'd1;
        do_sym   = 1'b1;
      end else begin
        tx_sym_a = 2'd0;
        // tx_done()
        if (tx_stf_q) begin
          tx_stf_a = 1'b0;
          tx_rl_a  = tx_rl_new;
          tx_rv_a  = tx_b_q;
        end else if (tx_an_q != 6'd0) begin
          if (ctx) crc_t = crc_shl;
          tx_an_a = tx_an_q - 6'd1;
          if (tx_an_a == 6'd0 && ctx && cmode_on) crc_t = crc_initv;
          if (smode_on) begin  // tx_stuff(TXB)
            tx_rl_a = tx_rl_new;
            tx_rv_a = tx_b_q;
            if (tx_stuff_due) tx_stf_a = 1'b1;
          end
        end else begin
          if (tx_k_q >= 2'd2 && ctx && cmode_on) crc_t = tx_crc_next;
          if (tx_k_q >= 2'd1) begin
            if (smode_on) begin  // tx_stuff(TXB)
              tx_rl_a = tx_rl_new;
              tx_rv_a = tx_b_q;
              if (tx_stuff_due) tx_stf_a = 1'b1;
            end
          end else begin
            tx_rl_a = 3'd0;
          end
          tx_n_a = tx_n_q - 5'd1;
          if (!tmsb) tx_sr_a = tx_sr_q >> 1;
          if (tx_n_a == 5'd0 && tx_k_q == 2'd3 && ctx && cm1) tx_an_a = {1'b0, cw} + 6'd1;
        end
        if (tx_stf_a || tx_n_a != 5'd0 || tx_an_a != 6'd0) begin
          do_sym = 1'b1;
        end else if (tx_lw_q) begin
          do_end = 1'b1;
        end else if (tx_full_q) begin
          // tx_load(), at a word boundary
          tx_sr_a   = tx_buf_q;
          tx_n_a    = tx_bn_q;
          tx_lw_a   = tx_bl_q;
          tx_k_a    = tx_bk_q;
          tx_full_a = 1'b0;
          do_sym    = 1'b1;
        end else begin
          tx_ur_a = 1'b1;
          do_end  = 1'b1;
        end
      end
    end else if (tx_tl_q) begin
      // tx_tail()
      if (tx_cd_q <= 17'd1) begin
        tx_tl_a  = 1'b0;
        tail_end = 1'b1;
        la_a     = idlea;
        lb_a     = idleb;
      end else begin
        tx_cd_a = tx_cd_q - 17'd1;
      end
    end else begin
      if (tx_n_q == 5'd0 && tx_full_q) begin
        // tx_load(), while idle
        tx_sr_a   = tx_buf_q;
        tx_n_a    = tx_bn_q;
        tx_lw_a   = tx_bl_q;
        tx_k_a    = tx_bk_q;
        tx_full_a = 1'b0;
      end
      if (tx_arm_q && cmt) begin
        tx_arm_a = 1'b0;
        tx_acc_a = 8'd0;
        if (tx_n_a == 5'd0) begin
          tx_ur_a = 1'b1;
          do_end  = 1'b1;
        end else begin
          tx_run_a = 1'b1;
          tx_sym_a = 2'd0;
          // tx_begin()
          tx_stf_a = 1'b0;
          tx_an_a  = 6'd0;
          tx_rl_a  = 3'd0;
          if (ctx && cmode_on) crc_t = crc_initv;
          do_sym   = 1'b1;
        end
      end
    end
  end

  // tx_symbol() on that state: the bit's source (a stuff bit, an appended CRC bit, or a data bit
  // scrambled or not), the output codes, and the symbol's length with its fraction.
  logic        sym_data, sym_bit, sym_va, sym_vb, sym_la, sym_lb, sym_carry;
  logic [1:0]  sym_ca, sym_cb;
  logic [7:0]  sym_frac, sym_acc;
  logic [16:0] sym_cd;

  assign sym_data = tmsb ? tx_sr_a[tx_n_a[3:0] - 4'd1] : tx_sr_a[0];
  assign sym_bit  = tx_stf_a ? ~tx_rv_a :
                    (tx_an_a != 6'd0) ? ((ctx & crc_t[31]) ^ cxor) :
                    (sym_data ^ (cm2 & ctx & (tx_k_a >= 2'd2) & crc_t[31]));
  assign sym_ca   = mapa[{tx_sym_a, 1'b0}+:2];
  assign sym_cb   = mapb[{tx_sym_a, 1'b0}+:2];
  assign sym_va   = (sym_ca == 2'd0) ? sym_bit : (sym_ca == 2'd1) ? ~sym_bit : sym_ca[0];
  assign sym_vb   = (sym_cb == 2'd0) ? sym_bit : (sym_cb == 2'd1) ? ~sym_bit : sym_cb[0];
  assign sym_la   = tgla ? (la_a ^ sym_va) : sym_va;
  assign sym_lb   = tglb ? (lb_a ^ sym_vb) : sym_vb;
  assign {sym_carry, sym_frac} = {1'b0, tx_acc_a} + {1'b0, tfrac};
  // TPER 0 acts as 1 and ignores TFRAC.
  assign sym_cd   = (tper == 16'd0) ? 17'd1 : ({1'b0, tper} + {16'd0, sym_carry});
  assign sym_acc  = (tper == 16'd0) ? tx_acc_a : sym_frac;

  // tx_half(), which tx_symbol(), tx_end() and the end of tx_tail() call: with THALF, a move whose TXACC,
  // the fraction carried into it, is 128 or more shows on the pads from the next falling edge (section 5.9.5).
  logic        tx_half;

  assign tx_half = (do_sym | do_end | tail_end) & thalf & ~text & (tper != 16'd0) & tx_acc_a[7];

  // The rest of tx_step(): tx_symbol() or tx_end(), whichever the branch calls last. Then step 3
  // and ABORT, which win. tx_step() clears TXH first, and tx_stop() clears it.
  logic        tx_arm_d, tx_run_d, tx_tl_d, tx_lw_d, tx_bl_d, tx_full_d, la_d, lb_d, tx_ur_d, tx_h_d;
  logic [15:0] tx_sr_d, tx_buf_d;
  logic [4:0]  tx_n_d, tx_bn_d;
  logic [1:0]  tx_sym_d, tx_k_d, tx_bk_d;
  logic [16:0] tx_cd_d;
  logic [7:0]  tx_acc_d;
  logic        tx_b_d, tx_stf_d, tx_rv_d;
  logic [5:0]  tx_an_d;
  logic [2:0]  tx_rl_d;

  always_comb begin
    tx_arm_d  = tx_arm_a;
    tx_run_d  = tx_run_a;
    tx_tl_d   = tx_tl_a;
    tx_lw_d   = tx_lw_a;
    tx_bl_d   = tx_bl_q;
    tx_full_d = tx_full_a;
    la_d      = la_a;
    lb_d      = lb_a;
    tx_ur_d   = tx_ur_a;
    tx_sr_d   = tx_sr_a;
    tx_buf_d  = tx_buf_q;
    tx_n_d    = tx_n_a;
    tx_bn_d   = tx_bn_q;
    tx_sym_d  = tx_sym_a;
    tx_k_d    = tx_k_a;
    tx_bk_d   = tx_bk_q;
    tx_cd_d   = tx_cd_a;
    tx_acc_d  = tx_acc_a;
    tx_b_d    = tx_b_q;
    tx_stf_d  = tx_stf_a;
    tx_rv_d   = tx_rv_a;
    tx_an_d   = tx_an_a;
    tx_rl_d   = tx_rl_a;
    tx_h_d    = tx_half;
    if (do_sym) begin
      // tx_symbol()
      tx_b_d   = sym_bit;
      la_d     = sym_la;
      lb_d     = sym_lb;
      tx_cd_d  = sym_cd;
      tx_acc_d = sym_acc;
    end
    if (do_end) begin
      // tx_end()
      tx_run_d = 1'b0;
      if (tail == 8'd0) begin
        la_d = idlea;
        lb_d = idleb;
      end else begin
        tx_tl_d = 1'b1;
        tx_cd_d = {9'd0, tail};
        la_d    = enda;
        lb_d    = endb;
      end
    end

    // Step 3: the instruction, on the flops as they were at the start of the cycle.
    if (sstop_tx) begin
      // tx_stop()
      tx_arm_d  = 1'b0;
      tx_run_d  = 1'b0;
      tx_tl_d   = 1'b0;
      tx_n_d    = 5'd0;
      tx_full_d = 1'b0;
      tx_sym_d  = 2'd0;
      tx_stf_d  = 1'b0;
      tx_an_d   = 6'd0;
      tx_h_d    = 1'b0;
      la_d      = idlea;
      lb_d      = idleb;
    end
    if (sgo_tx) tx_arm_d = 1'b1;
    if (push) begin
      // tx_push()
      tx_buf_d  = push_v;
      tx_bn_d   = (push_n == 4'd0) ? 5'd16 : {1'b0, push_n};
      tx_bl_d   = push_l;
      tx_bk_d   = push_k;
      tx_full_d = 1'b1;
    end
    if (sstat_rd && tx_ur_q) tx_ur_d = 1'b0;

    // After step 5: host ABORT runs tx_stop().
    if (h_abort) begin
      tx_arm_d  = 1'b0;
      tx_run_d  = 1'b0;
      tx_tl_d   = 1'b0;
      tx_n_d    = 5'd0;
      tx_full_d = 1'b0;
      tx_sym_d  = 2'd0;
      tx_stf_d  = 1'b0;
      tx_an_d   = 6'd0;
      tx_h_d    = 1'b0;
      la_d      = idlea_n;
      lb_d      = idleb_n;
    end

    // A cut piece's state never sets.
    if (CUT_CRC) tx_an_d = 6'd0;
    if (CUT_STUFF) begin
      tx_stf_d = 1'b0;
      tx_rl_d  = 3'd0;
      tx_rv_d  = 1'b0;
    end
  end

  // ============================================================================================
  // Receive half (sections 5.9.6 and 5.9.7).

  logic        rx_arm_q, rx_hunt_q, rx_run_q, rx_ep_q;
  logic [15:0] rx_sr_q;
  logic [4:0]  rx_n_q;
  logic [10:0] rx_tot_q;
  logic        rx_syn_q;
  logic [1:0]  rx_kind_q;
  logic signed [24:0] rx_ph_q;
  logic [4:0]  rx_g_q;
  logic        rx_open_q, rx_cand_q;
  logic [15:0] rx_ic_q;
  logic        rx_last_q, rx_win_q;
  logic [15:0] rx_buf_q;
  logic [4:0]  rx_bn_q;
  logic        rx_bl_q, rx_full_q;
  logic [4:0]  rx_pn_q;
  logic        rx_ovr_q, rx_seen_q;
  logic [2:0]  rx_rl_q;
  logic        rx_rv_q, rx_vio_q, rx_cov_q;
  logic [2:0]  rx_ef_q, rx_be_q, rx_pe_q;
  logic [15:0] rx_sr8_q;             // RXSR8 (section 5.9.7, the byte ends)
  logic [31:0] rx_c8_q;              // RXC8, the whole CRC register at the last byte boundary
  logic [4:0]  rx_n8_q;               // RXN8
  logic        rx_ones_q;             // RXONES
  logic signed [12:0] rx_ce_q;        // RXCE (RKIND 3: a candidate's phase, 256 - p to -1)

  // The window points p - p/4 and p - p/2, loaded in the cycle of an scfg that writes RXPER or
  // RXOPT, from the value written and the other slot as stored.
  logic [15:0] pwq_q, pwh_q;
  logic        pw_load;
  logic [15:0] pw_per, pw_pm, pwq_next, pwh_next;
  logic [7:0]  pw_rs, pw_frac;

  assign pw_load  = cfg_we & ((cfg_slot == 4'd5) | (cfg_slot == 4'd10));
  assign pw_per   = (cfg_slot == 4'd5) ? cfg_data : rxper_q;
  assign pw_frac  = CUT_RFRAC ? 8'd0 : (cfg_slot == 4'd10) ? cfg_data[7:0] : rxopt_q[7:0];
  assign pw_rs    = (|pw_per[15:8]) ? 8'hFF : pw_per[7:0];
  assign pw_pm    = (pw_per == 16'd0) ? 16'd256 : {pw_rs, pw_frac};
  assign pwq_next = pw_pm - {2'b00, pw_pm[15:2]};
  assign pwh_next = pw_pm - {1'b0, pw_pm[15:1]};
  // RKIND 3's p, RXP3 (512 for RPER 0 and 1, RPER above 15 as 15), its window points p - p/4 and p - p/2, and RXRG
  // and RXRA: flops loaded in the cycle a reception starts, from the slots as that cycle reads them (section 5.9.6,
  // rx_begin and the implementation note)
  logic [11:0] p3_q, p3_next, wq3_q, wq3_next, wh3_q, wh3_next;
  assign p3_next  = (rper[15:1] == 15'd0) ? 12'd512 : {((|rper[15:4]) ? 4'd15 : rper[3:0]), rfrac};
  logic [1:0]  rg3_q;
  logic        ra3_q;
  assign wq3_next = p3_next - {2'b00, p3_next[11:2]};
  assign wh3_next = p3_next - {1'b0, p3_next[11:1]};

  // The pins: the data, its edge, the outside clock, and the hunt's edge to HLVL.
  logic lvl2, lvl3, e, ck2, ck3, cedge, hunt_hit;

  assign lvl2     = sync2[rpin];
  assign lvl3     = sync3[rpin];
  assign e        = lvl2 ^ lvl3;
  assign ck2      = sync2[rclk];
  assign ck3      = sync3[rclk];
  assign cedge    = (ck2 ^ ck3) & (ck2 ^ rfall);
  assign hunt_hit = (lvl2 == hlvl) & (lvl3 != hlvl);

  // Which branch of rx_step() runs. A start is rx_begin(), from a hunt or from an arm.
  logic b_ep, b_run, b_hunt, b_arm, do_begin, man, man3, tim, ock, acq, rz;

  assign b_ep     = rx_ep_q;
  assign b_run    = ~rx_ep_q & rx_run_q;
  assign b_hunt   = ~rx_ep_q & ~rx_run_q & rx_hunt_q;
  assign b_arm    = ~rx_ep_q & ~rx_run_q & ~rx_hunt_q & rx_arm_q & cmt;
  assign do_begin = (b_hunt & hunt_hit) | (b_arm & ~hunt);
  assign man      = rx_kind_q == 2'd1;
  assign tim      = (rx_kind_q == 2'd0) | (CUT_OCK & (rx_kind_q == 2'd2));
  assign ock      = ~CUT_OCK & (rx_kind_q == 2'd2);
  assign man3     = rx_kind_q == 2'd3;
  assign acq      = rx_g_q[4:2] == 3'd0;    // RXG <= 3: RXG is 1 to 16 once started
  assign rz       = rper == 16'd0;

  // rx_manch(), in 17 bits: RXPH - p and RXPH - p/2 are formed beside the window compare, then
  // the error is selected, shifted by the gain's fixed wiring and folded into one add with the
  // + 256.
  logic [7:0]  rs;
  logic [15:0] pm;
  logic signed [17:0] ph, pp, p2, pw, a1, b0, ph1, merr, msh, mfull, mph3;
  logic        wide, open_now, mrun, endw, run1, open1, cand1, neg;
  logic        cand_hit, clk_hit, bnd_hit, mpull;

  assign rs       = (|rper[15:8]) ? 8'hFF : rper[7:0];
  assign pm       = rz ? 16'd256 : {rs, rfrac};
  assign ph       = {rx_ph_q[16], rx_ph_q[16:0]};
  assign pp       = {2'b00, pm};
  assign p2       = {3'b000, pm[15:1]};
  assign wide     = racq & (rgain != 2'd0) & ~rx_g_q[4];
  assign pw       = wide ? $signed({2'b00, pwh_q}) : $signed({2'b00, pwq_q});
  assign open_now = ph >= pw;
  assign a1       = ph - pp;
  assign b0       = ph - p2;
  assign ph1      = open_now ? a1 : ph;
  assign mrun     = b_run & man;
  assign endw     = mrun & open_now & rx_open_q;  // a window closes empty
  assign run1     = mrun & ~endw;
  assign open1    = open_now | rx_open_q;
  assign cand1    = open_now ? 1'b0 : rx_cand_q;
  assign neg      = ph1[17];
  assign cand_hit = run1 & open1 & e & neg;
  assign clk_hit  = run1 & open1 & ~cand_hit & (e | (cand1 & ~neg));
  assign bnd_hit  = run1 & ~open1 & e;
  assign mpull    = cand_hit | (clk_hit & e) | bnd_hit;
  assign merr     = bnd_hit ? b0 : ph1;
  assign msh      = acq ? (merr >>> 1) : (rgain == 2'd1) ? (merr >>> 2) :
                    (rgain == 2'd2) ? (merr >>> 4) : (merr >>> 5);
  assign mfull    = bnd_hit ? p2 : 18'sd0;
  assign mph3     = (mpull & (rgain == 2'd0)) ? mfull + 18'sd256 :
                    ph1 + 18'sd256 - (mpull ? msh : 18'sd0);

  // rx_timed(), in 25 bits: the error RXPH - 256 + p (when sampling) minus FIRST's target is
  // formed beside the countdown, by a carry-save add, then shifted by the gain and subtracted.
  logic [23:0] pt;
  logic [15:0] f1;
  logic signed [25:0] rt, th, tph1, terr, tsh, tnxt;
  logic [25:0] nrt, sp, cs_s, cs_c;
  logic        smp, trun, tpull;

  assign pt    = rz ? 24'd256 : {rper, rfrac};
  assign f1    = (first == 16'd0) ? 16'd1 : first;
  assign rt    = {2'b00, f1 - 16'd1, 8'd0};
  assign nrt   = ~{2'b00, f1, 8'd0};
  assign th    = {rx_ph_q[24], rx_ph_q};
  assign smp   = th[25] | (th[24:8] == 17'd0);   // RXPH < 256
  assign sp    = smp ? {2'b00, pt} : 26'd0;
  assign tph1  = th + sp - 26'sd256;
  assign cs_s  = th ^ sp ^ nrt;
  assign cs_c  = ((th & sp) | (th & nrt) | (sp & nrt)) << 1;
  assign terr  = cs_s + cs_c + 26'd1;
  assign tsh   = acq ? (terr >>> 1) : (rgain == 2'd1) ? (terr >>> 2) :
                 (rgain == 2'd2) ? (terr >>> 4) : (terr >>> 5);
  assign trun  = b_run & tim;
  assign tpull = trun & e & resync & (~racq | (lvl2 == hlvl));   // with RACQ only an edge to HLVL pulls
  assign tnxt  = tpull ? ((rgain == 2'd0) ? rt : tph1 - tsh) : tph1;

  // The bit of this cycle, at most one. REOP's end test takes the place of a timed sample.
  logic eop_hit, tsamp, osamp, samp, begin_bit, bit_v, bit_r, bv;
  logic bit3, blv3, endw3;   // RKIND 3: a bit in this cycle, its value, a window closed empty (below)

  assign eop_hit   = trun & smp & reop & ~lvl2 & ~ck2;
  assign tsamp     = trun & smp & ~eop_hit;
  assign osamp     = b_run & ock & cedge;
  assign samp      = tsamp | osamp;
  assign begin_bit = do_begin & rkind[0] & ~rdiff;    // RKIND 1 or 3
  assign bit_v     = clk_hit | bit3 | samp | begin_bit;
  assign bit_r     = clk_hit ? (rdiff ? rx_win_q : lvl2) : bit3 ? blv3 :
                     samp ? (rdiff ? (lvl2 == rx_last_q) : lvl2) : lvl2;
  assign bv        = bit_r ^ rinv;    // rx_bit()'s b after RINV

  // rx_bit() on the values that a start leaves (rx_begin() and rx_cbegin()), or on the flops.
  logic        syn0, vio0, cov0;
  logic [15:0] sr0;
  logic [4:0]  n0;
  logic [10:0] tot0;
  logic [2:0]  rl0, ef0;

  assign syn0 = do_begin ? (synclen == 5'd0) : rx_syn_q;
  assign sr0  = do_begin ? 16'd0 : rx_sr_q;
  assign n0   = do_begin ? 5'd0 : rx_n_q;
  assign tot0 = do_begin ? 11'd0 : rx_tot_q;
  assign vio0 = do_begin ? 1'b0 : rx_vio_q;
  assign rl0  = do_begin ? 3'd0 : rx_rl_q;
  assign cov0 = do_begin ? 1'b0 : rx_cov_q;
  assign ef0  = do_begin ? 3'd0 : rx_ef_q;

  // The destuffer. Whether this bit is at the stuff position depends on flops only.
  logic       stuffpos, vbit, pbit, pvio, pstuff, sbit, dbit, rl_upd;
  logic [2:0] rx_rl_new;

  assign stuffpos  = syn0 & smode_on & (rl0 != 3'd0) & (rl0 >= sne);
  assign vbit      = bit_v & vio0;                       // the bit after a violation: the end
  assign pbit      = bit_v & ~vio0 & stuffpos;           // the stuff position
  assign pvio      = pbit & (bv == rx_rv_q);             // a violation: dropped
  // With SMODE 1 the next bit ends the reception (RXVIO); with SMODE 2 the violating bit itself does (section 5.9.7)
  logic       pvio1, pvio2;
  assign pvio1     = pvio & (smode == 2'd1);
  assign pvio2     = pvio & (smode == 2'd2);
  assign pstuff    = pbit & (bv != rx_rv_q);             // a stuff bit: dropped
  assign sbit      = bit_v & ~vio0 & ~stuffpos & ~syn0;  // a bit of the sync search
  assign dbit      = bit_v & ~vio0 & ~stuffpos & syn0;   // a data bit
  assign rl_upd    = pstuff | ((sbit | dbit) & smode_on);
  assign rx_rl_new = (smode == 2'd1 && !bv) ? 3'd0 :
                     (rl0 == 3'd0 || bv != rx_rv_q) ? 3'd1 :
                     (rl0 < 3'd7) ? rl0 + 3'd1 : rl0;     // rx_run_len(b)

  // The CRC, beside the word path: crc_init() at a start, then this bit. With CTX 0 the send half
  // leaves the register alone, so the receive half starts from the register itself.
  logic        covered, rx_crc, rx_fb, bw, crc_pass;
  logic [31:0] crc_base, crc_r;

  assign covered  = dbit & (cov0 | (tot0 == {7'd0, cskip}));
  assign rx_crc   = covered & ~ctx & cmode_on;
  assign crc_base = (do_begin & ~ctx & cmode_on) ? crc_initv : crc_q;
  assign rx_fb    = cm1 ? (crc_base[31] ^ bv) : crc_base[31];
  assign crc_r    = rx_crc ? ({crc_base[30:0], 1'b0} ^ (rx_fb ? poly : 32'd0)) : crc_base;
  assign bw       = (rx_crc & cm2) ? (bv ^ crc_base[31]) : bv;    // descrambled
  // The byte ends (section 5.9.7). At the end test, with 1 to 6 ones after the last byte boundary and no stuff
  // violation, rx_eop() puts RXSR, RXN and the register's bits 31 to 16 back to their copies before the check. With
  // RKIND 3 and REOP, at a window that closes empty with 1 to 7 bits after the last byte boundary and no stuff
  // violation, rx_mend() puts back RXSR, RXN and the whole register (an IEEE 802.3 MAC's truncation to whole
  // octets, 4.2.9). No bit enters the register in either cycle (RKIND 3 gives no bit in a cycle whose window
  // closes empty), so the check of the copy is formed from flops beside the other, and chosen by its 1-bit result.
  logic        eop_rest, mend_rest, c8_on, c8_hi_eq, c8_lo_eq, crc_lo_eq;
  logic [31:0] crc_e;

  assign c8_on     = ~ctx & cmode_on;
  assign eop_rest  = eop_hit & rx_ones_q & ~rx_vio_q & (rx_tot_q[2:0] != 3'd0) & (rx_tot_q[2:0] != 3'd7);
  assign mend_rest = endw3 & reop & ~rx_vio_q & (rx_tot_q[2:0] != 3'd0);
  assign crc_e     = {((eop_rest | mend_rest) & c8_on) ? rx_c8_q[31:16] : crc_r[31:16],
                      (mend_rest & c8_on) ? rx_c8_q[15:0] : crc_r[15:0]};
  assign c8_hi_eq  = rx_c8_q[31:16] == chk[31:16];
  assign c8_lo_eq  = rx_c8_q[15:0] == chk[15:0];
  assign crc_lo_eq = crc_q[15:0] == chk[15:0];
  assign crc_pass  = cm1 & ~ctx & (((eop_rest | mend_rest) & c8_on) ? (c8_hi_eq & (mend_rest ? c8_lo_eq : crc_lo_eq)) :
                                   (crc_r == chk));

  // The sync search and the words.
  logic [15:0] lowmask, highmask;

  always_comb begin
    for (int i = 0; i < 16; i = i + 1) begin
      lowmask[i]  = synclen > 5'(i);
      highmask[i] = synclen > 5'(15 - i);
    end
  end

  logic [15:0] sh_m, sh_l, shd, ins, sr_b;
  logic        match, rcnt_end, full16, syn_b;
  logic [4:0]  n1, n_b;
  logic [10:0] tot1, tot_b;

  assign sh_m     = {sr0[14:0], bv};
  assign sh_l     = {bv, sr0[15:1]};
  assign shd      = rmsb ? sh_m : sh_l;
  assign match    = ((shd ^ sync) & (rmsb ? lowmask : highmask)) == 16'd0;
  assign ins      = rmsb ? {sr0[14:0], bw} : (sr0 | ({15'd0, bw} << n0[3:0]));
  assign n1       = n0 + 5'd1;
  assign tot1     = tot0 + 11'd1;
  assign rcnt_end = dbit & (rcnt != 11'd0) & (tot1 == rcnt);
  assign full16   = dbit & ~rcnt_end & (n1 == 5'd16);
  assign syn_b    = sbit ? match : syn0;
  assign sr_b     = sbit ? (match ? 16'd0 : shd) : dbit ? (full16 ? 16'd0 : ins) : sr0;
  assign n_b      = dbit ? (full16 ? 5'd0 : n1) : n0;
  assign tot_b    = dbit ? tot1 : tot0;

  // RKIND 3 (section 5.9.6, rx_manch3): an edge in each half of the cycle (SYNC3 against SYNC2F, SYNC2F against
  // SYNC2), decisions on RXPH as it was at the start of the cycle, a candidate kept in RXCE until it gives the bit,
  // and the cycle's one pull at its end: module rk3n_f below, two carry chains between flops (5.9.6, implementation
  // note).
  logic               lvlf, bit_ends, ran3;
  logic [11:0]        p3, pq3, ph3w;
  logic signed [17:0] r3;
  logic signed [12:0] ce3;
  logic               op3, cd3, wn3;
  logic [4:0]         g3;

  assign lvlf     = sync2f[rpin];
  assign p3       = p3_q;
  assign pq3      = wq3_q;
  assign ph3w     = wh3_q;
  // A bit now ends the reception (the bit after a violation, or a data bit that reaches RCNT): from the flops
  // alone, since the block runs only in rx_run, never with rx_begin. With SMODE 2 a bit at the stuff position ends
  // it too if it is a violation, equal to RXRV after RINV: vio2 with the value vb the bit must have before RINV.
  logic vio2, vb3;
  assign bit_ends = rx_vio_q | (~(rx_syn_q & smode_on & (rx_rl_q != 3'd0) & (rx_rl_q >= sne)) & rx_syn_q &
                    (rcnt != 11'd0) & ((rx_tot_q + 11'd1) == rcnt));
  assign vio2     = ~rx_vio_q & rx_syn_q & (smode == 2'd2) & (rx_rl_q != 3'd0) & (rx_rl_q >= sne);
  assign vb3      = rx_rv_q ^ rinv;

  rk3n_f #(.PW(12), .CW(13)) u_rk3n (.run(b_run & man3), .ph(ph), .op(rx_open_q), .cd(rx_cand_q), .wn(rx_win_q),
               .g(rx_g_q), .ce(rx_ce_q), .p3(p3), .pq3(pq3), .ph3w(ph3w), .rgain(rg3_q), .racq(ra3_q), .rdiff(rdiff),
               .l0(lvl3), .l1(lvlf), .l2(lvl2), .bit_ends(bit_ends), .vio2(vio2), .vb(vb3), .bit3(bit3),
               .blv3(blv3), .endw3(endw3),
               .r3(r3), .op3(op3), .cd3(cd3), .wn3(wn3), .g3(g3), .ce3(ce3), .ran(ran3));

  // RXIC and the ends of a reception, at most one per cycle.
  logic [15:0] ic_inc, ic_run;
  logic        vio_end, rend_end, end_now;
  logic [2:0]  ef_b;

  assign ic_inc   = (rx_ic_q != 16'hFFFF) ? rx_ic_q + 16'd1 : rx_ic_q;
  assign ic_run   = (clk_hit | bit3 | (trun & e) | osamp) ? 16'd0 : ic_inc;
  assign vio_end  = vbit;
  assign rend_end = b_run & ~endw & ~endw3 & ~rcnt_end & ~vio_end & ~pvio2 & ~eop_hit & (rend != 16'd0) &
                    (ic_run >= rend);
  assign end_now  = endw | endw3 | rcnt_end | rend_end | vio_end | pvio2 | eop_hit;
  assign ef_b     = ef0 | {vbit & (bv == rx_rv_q), pvio, end_now & crc_pass};

  // rx_deliver(): a full word, or the last word of a reception once the buffer is empty.
  logic        ep_deliver, deliver, wr_buf, ovr_set, pop_eff, rx_stop;
  logic [15:0] dv;
  logic [4:0]  dn;

  assign ep_deliver = b_ep & ~rx_full_q;
  assign deliver    = ep_deliver | full16;
  assign dv         = ep_deliver ? rx_sr_q : ins;
  assign dn         = ep_deliver ? rx_n_q : 5'd16;
  assign wr_buf     = deliver & ~rx_full_q;
  assign ovr_set    = deliver & rx_full_q;
  assign pop_eff    = pop & rx_full_q;
  assign rx_stop    = sstop_rx | h_abort;     // rx_stop(), from sstop rx or ABORT

  // Next values of the receive flops that the branches write.
  logic               rx_upd, gear, rv_we, rv_b, cov_b, open_b, cand_b, win_b;
  logic [2:0]         rl_b;
  logic [4:0]         g_inc;
  logic signed [24:0] ph_beg;

  assign rx_upd = do_begin | b_run;
  assign gear   = (clk_hit | tpull) & ~rx_g_q[4];     // rx_gear()
  assign g_inc  = rx_g_q + 5'd1;
  assign rl_b   = CUT_STUFF ? 3'd0 : rl_upd ? rx_rl_new : rl0;
  assign rv_we  = CUT_STUFF | rl_upd;
  assign rv_b   = ~CUT_STUFF & bv;
  assign cov_b  = CUT_CRC ? 1'b0 : (cov0 | (dbit & cmode_on & (tot0 == {7'd0, cskip})));
  assign open_b = open1 & ~clk_hit;
  assign cand_b = cand_hit | (cand1 & ~clk_hit);
  assign win_b  = clk_hit ? 1'b0 : (bnd_hit | (cand_hit & cand1)) ? 1'b1 : rx_win_q;
  // RXPH's next value: one AND-OR of every source, its selects from flops (exclusive), so each source is two gates
  // from the flop.
  logic signed [24:0] ph_d;
  logic               sp3, sp1, sp0, spb, sph;
  assign sp3    = b_run & man3;
  assign sp1    = b_run & man;
  assign sp0    = b_run & tim;
  assign spb    = do_begin;
  assign sph    = ~(sp3 | sp1 | sp0 | spb);
  assign ph_d   = ({25{sp3}} & {{8{r3[16]}}, r3[16:0]}) | ({25{sp1}} & {{8{mph3[16]}}, mph3[16:0]}) |
                  ({25{sp0}} & tnxt[24:0]) | ({25{spb}} & ph_beg) | ({25{sph}} & rx_ph_q);
  // rx_begin(): the Manchester decoders start at 256; RKIND 3 at 128 when a hunt's start edge falls in the second
  // half of its cycle (SYNC2F still shows the old level).
  assign ph_beg = (rkind == 2'd1) ? 25'sd256 : (rkind == 2'd3) ? ((b_hunt & (lvlf != lvl2)) ? 25'sd128 : 25'sd256) :
                  rt[24:0];

  // ============================================================================================
  // Flops.

  // The slots, written by scfg (step 3) and seen by the unit from the next cycle, and the window
  // points beside them.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      txmode_q  <= 16'h0000;
      txmap_q   <= 16'h0000;
      txper_q   <= 16'h0000;
      txtail_q  <= 16'h0000;
      rxmode_q  <= 16'h0000;
      rxper_q   <= 16'h0000;
      rxfirst_q <= 16'h0000;
      rxend_q   <= 16'h0000;
      rxcnt_q   <= 16'h0000;
      rxsync_q  <= 16'h0000;
      rxopt_q   <= 16'h0000;
      polyhi_q  <= 16'h0000;
      polylo_q  <= 16'h0000;
      chkhi_q   <= 16'h0000;
      chklo_q   <= 16'h0000;
      coding_q  <= 16'h0000;
      pwq_q     <= 16'd192;     // p = 256 in the reset configuration
      pwh_q     <= 16'd128;
    end else begin
      if (cfg_we) begin
        case (cfg_slot)
          4'd0:    txmode_q  <= cfg_data & KEEP0;
          4'd1:    txmap_q   <= cfg_data;
          4'd2:    txper_q   <= cfg_data;
          4'd3:    txtail_q  <= cfg_data & KEEP3;
          4'd4:    rxmode_q  <= cfg_data & KEEP4;
          4'd5:    rxper_q   <= cfg_data;
          4'd6:    rxfirst_q <= cfg_data;
          4'd7:    rxend_q   <= cfg_data;
          4'd8:    rxcnt_q   <= cfg_data;
          4'd9:    rxsync_q  <= cfg_data;
          4'd10:   rxopt_q   <= cfg_data & KEEP10;
          4'd11:   polyhi_q  <= cfg_data & KEEP11;
          4'd12:   polylo_q  <= cfg_data & KEEP11;
          4'd13:   chkhi_q   <= cfg_data & KEEP11;
          4'd14:   chklo_q   <= cfg_data & KEEP11;
          default: coding_q  <= cfg_data & KEEP15;
        endcase
      end
      if (pw_load) begin
        pwq_q <= pwq_next;
        pwh_q <= pwh_next;
      end
    end
  end

  // The CRC register: the half that CTX names writes it, and the other leaves it alone.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) crc_q <= 32'd0;
    else        crc_q <= CUT_CRC ? 32'd0 : ctx ? crc_t : crc_e;
  end

  // The send half. Its next values already hold step 3 and ABORT.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_arm_q  <= 1'b0;
      tx_run_q  <= 1'b0;
      tx_tl_q   <= 1'b0;
      tx_lw_q   <= 1'b0;
      tx_bl_q   <= 1'b0;
      tx_full_q <= 1'b0;
      la_q      <= 1'b0;
      lb_q      <= 1'b0;
      tx_ur_q   <= 1'b0;
      tx_sr_q   <= 16'h0000;
      tx_buf_q  <= 16'h0000;
      tx_n_q    <= 5'd0;
      tx_bn_q   <= 5'd0;
      tx_sym_q  <= 2'd0;
      tx_k_q    <= 2'd0;
      tx_bk_q   <= 2'd0;
      tx_cd_q   <= 17'd0;
      tx_acc_q  <= 8'd0;
      tx_b_q    <= 1'b0;
      tx_stf_q  <= 1'b0;
      tx_rv_q   <= 1'b0;
      tx_an_q   <= 6'd0;
      tx_rl_q   <= 3'd0;
      tx_h_q    <= 1'b0;
      ola_q     <= 1'b0;
      olb_q     <= 1'b0;
    end else begin
      tx_arm_q  <= tx_arm_d;
      tx_run_q  <= tx_run_d;
      tx_tl_q   <= tx_tl_d;
      tx_lw_q   <= tx_lw_d;
      tx_bl_q   <= tx_bl_d;
      tx_full_q <= tx_full_d;
      la_q      <= la_d;
      lb_q      <= lb_d;
      tx_ur_q   <= tx_ur_d;
      tx_sr_q   <= tx_sr_d;
      tx_buf_q  <= tx_buf_d;
      tx_n_q    <= tx_n_d;
      tx_bn_q   <= tx_bn_d;
      tx_sym_q  <= tx_sym_d;
      tx_k_q    <= tx_k_d;
      tx_bk_q   <= tx_bk_d;
      tx_cd_q   <= tx_cd_d;
      tx_acc_q  <= tx_acc_d;
      tx_b_q    <= tx_b_d;
      tx_stf_q  <= tx_stf_d;
      tx_rv_q   <= tx_rv_d;
      tx_an_q   <= tx_an_d;
      tx_rl_q   <= tx_rl_d;
      tx_h_q    <= tx_h_d;
      ola_q     <= la_q;      // tx_step(): OLA := LA, every cycle
      olb_q     <= lb_q;
    end
  end

  // The pads' view of shift outputs A and B, in the two halves of a cycle (section 5.2): V0's A0 = OLA if TXH else
  // LA in the first half, LA in the second. Flop P moves the pad at the rising edge, flop N moves it at the falling
  // edge through a latch that is open while the clock is low, and the pad is P XOR the latch. For the next cycle's
  // levels v1 (first half) and v2 (second), P takes v1 XOR N and N takes P's new value XOR v2, so N changes only
  // when the halves differ, and the XOR's two inputs never change in the same half cycle. TXH next and OLA next = LA
  // give v1 = LA next XOR dh, with dh = TXH next AND (LA XOR LA next), and v2 = LA next. At reset P, N and the latch
  // are 0, so every shift output shows 0 (section 5.2).
  logic dha, dhb, pa_q, na_q, pb_q, nb_q, na_lat, nb_lat;

  assign dha = tx_h_d & (la_q ^ la_d);
  assign dhb = tx_h_d & (lb_q ^ lb_d);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pa_q <= 1'b0;
      na_q <= 1'b0;
      pb_q <= 1'b0;
      nb_q <= 1'b0;
    end else begin
      pa_q <= la_d ^ dha ^ na_q;
      na_q <= na_q ^ dha;
      pb_q <= lb_d ^ dhb ^ nb_q;
      nb_q <= nb_q ^ dhb;
    end
  end

  // Library latches, open while GATE_N (the clock) is low, cleared by RESET_B.
  sg13cmos5l_dllrq_1 lat_na (.D(na_q), .GATE_N(clk), .RESET_B(rst_n), .Q(na_lat));
  sg13cmos5l_dllrq_1 lat_nb (.D(nb_q), .GATE_N(clk), .RESET_B(rst_n), .Q(nb_lat));

  assign pad_a = pa_q ^ na_lat;
  assign pad_b = pb_q ^ nb_lat;

  // The receive half: rx_step() (step 2b), then the flops that step 3 or ABORT also write.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_arm_q  <= 1'b0;
      rx_hunt_q <= 1'b0;
      rx_run_q  <= 1'b0;
      rx_ep_q   <= 1'b0;
      rx_sr_q   <= 16'h0000;
      rx_n_q    <= 5'd0;
      rx_tot_q  <= 11'd0;
      rx_syn_q  <= 1'b0;
      rx_kind_q <= 2'd0;
      rx_ph_q   <= 25'sd0;
      rx_g_q    <= 5'd0;
      rx_open_q <= 1'b0;
      rx_cand_q <= 1'b0;
      rx_ic_q   <= 16'h0000;
      rx_last_q <= 1'b0;
      rx_win_q  <= 1'b0;
      rx_buf_q  <= 16'h0000;
      rx_bn_q   <= 5'd0;
      rx_bl_q   <= 1'b0;
      rx_full_q <= 1'b0;
      rx_pn_q   <= 5'd0;
      rx_ovr_q  <= 1'b0;
      rx_seen_q <= 1'b0;
      rx_rl_q   <= 3'd0;
      rx_rv_q   <= 1'b0;
      rx_vio_q  <= 1'b0;
      rx_cov_q  <= 1'b0;
      rx_ef_q   <= 3'd0;
      rx_be_q   <= 3'd0;
      rx_pe_q   <= 3'd0;
      rx_sr8_q  <= 16'h0000;
      rx_n8_q   <= 5'd0;
      rx_c8_q   <= 32'h0000_0000;
      rx_ones_q <= 1'b0;
      rx_ce_q   <= 13'sd0;
      p3_q      <= 12'd0;
      wq3_q     <= 12'd0;
      wh3_q     <= 12'd0;
      rg3_q     <= 2'd0;
      ra3_q     <= 1'b0;
    end else begin
      if (do_begin) begin     // RKIND 3 keeps the p, RGAIN and RACQ it started with
        p3_q  <= p3_next;
        wq3_q <= wq3_next;
        wh3_q <= wh3_next;
        rg3_q <= rgain;
        ra3_q <= racq;
      end
      // rx_byte() (section 5.9.7): the copies at the start, which rx_cbegin() takes, and after every 8th data bit;
      // RXONES clears at a 0 between. The start's own bit, for the Manchester decoders, is bit 1.
      if (do_begin) begin
        rx_sr8_q  <= 16'h0000;
        rx_n8_q   <= 5'd0;
        if (c8_on) rx_c8_q <= crc_base;
        rx_ones_q <= ~dbit | bw;
      end else if (dbit) begin
        if (tot1[2:0] == 3'd0) begin
          rx_sr8_q  <= sr_b;
          rx_n8_q   <= n_b;
          if (c8_on) rx_c8_q <= crc_r;
          rx_ones_q <= 1'b1;
        end else if (!bw) begin
          rx_ones_q <= 1'b0;
        end
      end
      if (rx_upd) begin
        rx_sr_q  <= (eop_rest | mend_rest) ? rx_sr8_q : sr_b;
        rx_n_q   <= (eop_rest | mend_rest) ? rx_n8_q : n_b;
        rx_tot_q <= tot_b;
        rx_syn_q <= syn_b;
        rx_rl_q  <= rl_b;
        rx_cov_q <= cov_b;
        rx_ef_q  <= ef_b;
      end
      if (rv_we) rx_rv_q <= rv_b;
      rx_ph_q <= ph_d;
      if (do_begin) begin
        rx_kind_q <= rkind;
        rx_g_q    <= 5'd1;
        rx_open_q <= 1'b0;
        rx_cand_q <= 1'b0;
        rx_ic_q   <= 16'h0000;
        rx_last_q <= lvl3;
        rx_win_q  <= 1'b0;
      end else if (b_run) begin
        if (gear) rx_g_q <= g_inc;
        if (man) begin
          rx_open_q <= open_b;
          rx_cand_q <= cand_b;
          rx_win_q  <= win_b;
        end
        if (man3) begin
          rx_g_q    <= g3;
          rx_open_q <= op3;
          rx_cand_q <= cd3;
          rx_win_q  <= wn3;
          rx_ce_q   <= ce3;
        end
        rx_ic_q <= ic_run;
        if (samp) rx_last_q <= lvl2;
      end
      if (wr_buf) begin
        rx_buf_q <= dv;
        rx_bn_q  <= dn;
        rx_bl_q  <= ep_deliver;
        rx_be_q  <= ep_deliver ? rx_ef_q : 3'd0;
      end
      // Step 3 and ABORT win here.
      rx_vio_q  <= (rx_stop | CUT_STUFF) ? 1'b0 : rx_upd ? (vio0 | pvio1) : rx_vio_q;
      rx_arm_q  <= rx_stop ? 1'b0 : sgo_rx ? 1'b1 : b_arm ? 1'b0 : rx_arm_q;
      rx_hunt_q <= rx_stop ? 1'b0 : do_begin ? 1'b0 :
                   ((end_now & ~syn_b) | ep_deliver | b_arm) ? (hunt | rx_hunt_q) : rx_hunt_q;
      rx_run_q  <= rx_stop ? 1'b0 : do_begin ? ~(rcnt_end | vio_end) : b_run ? ~end_now : rx_run_q;
      rx_ep_q   <= rx_stop ? 1'b0 : b_ep ? rx_full_q : (end_now & syn_b) ? 1'b1 : rx_ep_q;
      rx_full_q <= rx_stop ? 1'b0 : pop_eff ? 1'b0 : wr_buf ? 1'b1 : rx_full_q;
      rx_pn_q   <= pop_eff ? rx_bn_q : rx_pn_q;
      rx_pe_q   <= pop_eff ? rx_be_q : rx_pe_q;
      rx_ovr_q  <= (sstat_rd & rx_ovr_q) ? 1'b0 : ovr_set ? 1'b1 : rx_ovr_q;
      rx_seen_q <= (sstat_rd & rx_seen_q) ? 1'b0 : do_begin ? 1'b1 : rx_seen_q;
    end
  end

  // ============================================================================================
  // Outputs.

  logic [15:0] crc_lo, crc_hi, pe_word;

  assign crc_lo  = crc_q[15:0];
  assign crc_hi  = crc_q[31:16];
  assign pe_word = {13'd0, rx_pe_q};

  assign la       = la_q;
  assign lb       = lb_q;
  assign txfull   = tx_full_q;
  assign rxfull   = rx_full_q;
  assign rxbuf    = rx_buf_q;
  assign rxbl     = rx_bl_q;
  assign status   = {(rx_arm_q | rx_hunt_q | rx_ep_q), lb_q, la_q, rx_seen_q, rx_syn_q, rx_ovr_q,
                     rx_full_q, rx_run_q, tx_ur_q, tx_full_q, (tx_arm_q | tx_run_q | tx_tl_q),
                     rx_pn_q};
  assign srd_data = srd_sel[1] ? pe_word : srd_sel[0] ? crc_hi : crc_lo;

  // RDREG (section 9.2): 12 the status word with nothing cleared, 13 and 14 the CRC register's
  // halves, 15 RXPE, and 16 + k slot k.
  always_comb begin
    case (rd_reg)
      5'd12:   rd_data = status;
      5'd13:   rd_data = crc_lo;
      5'd14:   rd_data = crc_hi;
      5'd15:   rd_data = pe_word;
      5'd16:   rd_data = txmode_q;
      5'd17:   rd_data = txmap_q;
      5'd18:   rd_data = txper_q;
      5'd19:   rd_data = txtail_q;
      5'd20:   rd_data = rxmode_q;
      5'd21:   rd_data = rxper_q;
      5'd22:   rd_data = rxfirst_q;
      5'd23:   rd_data = rxend_q;
      5'd24:   rd_data = rxcnt_q;
      5'd25:   rd_data = rxsync_q;
      5'd26:   rd_data = rxopt_q;
      5'd27:   rd_data = polyhi_q;
      5'd28:   rd_data = polylo_q;
      5'd29:   rd_data = chkhi_q;
      5'd30:   rd_data = chklo_q;
      5'd31:   rd_data = coding_q;
      default: rd_data = 16'h0000;
    endcase
  end

  // Bits the arithmetic forms and nothing reads. TXH, OLA and OLB are section 5.9.2's names for what P, N and the
  // latch hold in another form: the pads take them from there, so nothing reads them here.
  logic _unused;
  assign _unused = &{mph3[17], tnxt[25], r3[17], tx_h_q, ola_q, olb_q, ran3, 1'b0};

endmodule

// RKIND 3's cycle step, rx_manch3 of section 5.9.6, as its implementation note shapes it: two carry chains between
// flops. RKIND 3 takes RPER above 15 as 15, so p is below 4096 (PW = 12), every sum fits IW = PW + 3 bits, and RXCE,
// from 256 - p to -1, fits CW = PW + 1.
// Chain 1, side by side from the flops: the window compares and the phase compares (x is RXPH at the start of the
// cycle), every error a pull can take (x - p, x + 128 - p, x - p/2, x + 128, x + 128 - p/2; RXCE is a flop), and the
// bases. Then the decisions, written flat as sums of products of the compares and the flops, the error's one-hot
// choice, the gain's fixed wiring, and chain 2: the base less the correction. The base is x + 256, or x + 256 - p
// after a window opening, and x + 128 (- p) when the first half step ends the reception, since the code then runs no
// second one. The outputs are the code's in every state that a reception of RKIND 3 keeps (5.9.6, the Period bullet;
// its p, RGAIN and RACQ are RXP3, RXRG and RXRA, held from its start): p from 512 to 4095 with its window points, RXG
// from 1 to 16, RXCAND only with RXOPEN, RXPH at least 128 - p with the window closed and 256 - p with it open, RXPH
// below p + 128 with RGAIN 1 to 3 and 7p/4 - 128 with RGAIN 0 (so a cycle opens at most one window), and RXCE from
// 256 - p to -1 with RXCAND. RXPH's next value r3 is the code's while the block runs, the only cycles pe_shift's ph_d
// takes it. A bit ends the reception when bit_ends says so, or with SMODE 2 when vio2 is set and the bit equals vb.
module rk3n_f #(parameter int PW = 12, parameter int CW = 13) (
  input  logic               run,
  input  logic signed [17:0] ph,
  input  logic               op, cd, wn,
  input  logic [4:0]         g,
  input  logic signed [CW-1:0] ce,
  input  logic [PW-1:0]      p3,
  input  logic [PW-1:0]      pq3, ph3w,
  input  logic [1:0]         rgain,
  input  logic               racq, rdiff,
  input  logic               l0, l1, l2,
  input  logic               bit_ends,     // a bit now ends the reception, whatever its value (flops only)
  input  logic               vio2, vb,     // SMODE 2's stuff position: a bit of value vb (before RINV) ends it too
  output logic               bit3, blv3, endw3,
  output logic signed [17:0] r3,
  output logic               op3, cd3, wn3,
  output logic [4:0]         g3,
  output logic signed [CW-1:0] ce3,
  output logic               ran
);
  localparam int IW = PW + 3;
  localparam logic signed [IW-1:0] C128 = IW'(128);
  localparam logic signed [IW-1:0] C256 = IW'(256);
  logic signed [IW-1:0] x, p, p2, dec, wq, wh, cex;
  logic signed [IW-1:0] s_p, s_zp, s_h, s_z, s_zh, s_b0, s_bp;
  logic               e1, e2, wide0, g16, two;
  logic [4:0]         g1;

  assign x     = ph[IW-1:0];                             // RXPH stays from -3967 to 7037 (5.9.6): 15 bits
  logic _unused_ph;
  assign _unused_ph = &{ph[17:IW], 1'b0};
  assign p     = $signed({3'b000, p3});
  assign p2    = $signed({4'b0000, p3[PW-1:1]});
  assign dec   = $signed({6'b000000, p3[PW-1:3]});
  assign wq    = $signed({3'b000, pq3});
  assign wh    = $signed({3'b000, ph3w});
  assign cex   = {{(IW - CW){ce[CW-1]}}, ce};
  // the keep test's bound (5.9.6): 128, one half step, when p is below 768 (RPER 2), where p/8 (64 to 95) is less
  // than the sampling interval; p/8 from RPER 3 up
  logic signed [IW-1:0] kbd;
  assign kbd   = (p3 < PW'(768)) ? C128 : dec;
  assign e1    = l0 ^ l1;
  assign e2    = l1 ^ l2;
  assign two   = e1 & e2;                                // an edge in each half: the cycle pulls nothing
  assign g16   = g[4];                                   // RXG is 1 to 16: bit 4 is RXG = 16
  assign g1    = g16 ? g : g + 5'd1;                     // RXG after a bit
  assign wide0 = racq & (rgain != 2'd0) & ~g16;

  // chain 1: sums
  assign s_p  = x - p;                                   // the first half's phase after an opening, or less p
  assign s_zp = x + C128 - p;                            // the second half's phase after a p in either half
  assign s_h  = x - p2;                                  // a first-half boundary edge's error
  assign s_z  = x + C128;                                // the second half's phase with no p
  assign s_zh = x + C128 - p2;                           // a second-half boundary edge's error
  assign s_b0 = x + C256;                                // the base, no p
  assign s_bp = x + C256 - p;                            // the base, a p
  // chain 1: compares
  logic c_oq, c_oh, c_xd, c_zdq, c_zdh, c_zd, c_zpd, cel, x_neg, z_neg, zp_neg, p_neg;
  assign c_oq   = x >= wq;                               // x at the narrow window point
  assign c_oh   = x >= wh;                               // x at the wide one
  assign c_xd   = x >= dec;
  assign c_zdq  = x + C128 >= wq;                        // x + 128 at the window points
  assign c_zdh  = x + C128 >= wh;
  assign c_zd   = x + C128 >= dec;
  assign c_zpd  = x + C128 - p >= dec;
  logic signed [IW-1:0] s_cm8;
  assign s_cm8  = cex + kbd;
  assign cel    = ~s_cm8[IW-1];                          // RXCE at least -kbd (flops only)
  assign x_neg  = x[IW-1];
  assign z_neg  = s_z[IW-1];
  assign zp_neg = s_zp[IW-1];
  assign p_neg  = s_p[IW-1];
  // The structural rule before the window needs RXPH at least the window point less p/8 (5.9.6).
  // tsum = x + p/8 - (p - w), one sum of three from the flops; x + 128 is below the point when tsum < -128.
  logic signed [IW-1:0] wsel, tsum;
  logic               x_lt, z_lt;
  assign wsel   = wide0 ? wh : wq;
  assign tsum   = x + dec - wsel;
  assign x_lt   = tsum[IW-1];                            // x < p - w - p/8
  assign z_lt   = tsum[IW-1] & ~(&tsum[IW-2:7]);         // x + 128 < p - w - p/8 (tsum below -128)

  // the decisions, first half
  logic o1, endw1, r1, oph0, cdb, y1neg, ear1, cand1, kc1, bit1e, bit1t, bnd1, bit1, ce1, cons1;
  assign o1     = wide0 ? c_oh : c_oq;
  assign endw1  = run & o1 & op;                         // a window closes empty
  assign r1     = run & ~endw1;
  assign oph0   = o1 | op;                               // the window is open in this half step
  assign cdb    = ~o1 & cd;                              // a candidate stands (an opening clears it)
  assign y1neg  = o1 ? p_neg : x_neg;                    // the phase: x - p after an opening, else x
  assign ear1   = r1 & e1 & wn & ~cdb & (oph0 | ~x_lt); // the edge after a non-clock edge is the clock edge
  assign cand1  = r1 & e1 & ~ear1 & oph0 & y1neg;
  assign kc1    = r1 & e1 & ~o1 & op & cd & c_xd & cel;  // the candidate keeps the bit, the edge comes after it
  assign bit1e  = r1 & e1 & ~ear1 & oph0 & ~y1neg & ~kc1;
  assign bit1t  = r1 & ~e1 & ~o1 & op & cd & c_xd;       // the candidate's time is up
  assign bnd1   = r1 & e1 & ~ear1 & ~oph0;
  assign bit1   = ear1 | bit1e | bit1t | kc1;
  // the first half's bit ends the reception (rx_bit's rx_end, so the second half step doesn't run): whatever its
  // value, or with SMODE 2 at the stuff position if it equals vb. Flat by the bit's four sources, each value from
  // flops: the structural rule l1 (1 with RDIFF); a kept candidate l0 (RXWIN); an edge l1 (RXWIN or a standing
  // candidate, wn | cdb); a candidate's time l1 (RXWIN)
  logic q_e1, q_k1, q_b1, q_t1, end1;
  assign q_e1   = vb == (rdiff | l1);
  assign q_k1   = vb == (rdiff ? wn : l0);
  assign q_b1   = vb == (rdiff ? (wn | cdb) : l1);
  assign q_t1   = vb == (rdiff ? wn : l1);
  assign end1   = (bit1 & bit_ends) | (vio2 & ((ear1 & q_e1) | (kc1 & q_k1) | (bit1e & q_b1) | (bit1t & q_t1)));
  assign ce1    = ear1 & ~oph0;                          // before the window: the prediction is used up
  assign cons1  = o1 | ce1;                              // RXPH less p in the first half

  // second half: the window point is the cycle's (RXG at its start); no opening after a first-half p
  logic run2, Zc, o2, opa, cda, wna, endw2, r2, oph1, cd2, cons2, y2neg, y2dec, ear2, cand2, cel2, kc2;
  logic bit2e, bit2t, bnd2, bit2;
  assign run2   = r1 & ~end1;
  assign Zc     = wide0 ? c_zdh : c_zdq;                 // x + 128 at the window point
  assign o2     = ~cons1 & Zc;
  assign opa    = oph0 & ~bit1;                          // RXOPEN after the first half
  assign cda    = cand1 | (cdb & ~bit1);                 // RXCAND after the first half
  assign wna    = cand1 ? (wn | cdb) : bit1 ? kc1 : bnd1 ? 1'b1 : wn;   // RXWIN after the first half
  assign endw2  = run2 & o2 & opa;
  assign r2     = run2 & ~endw2;
  assign oph1   = opa | o2;
  assign cd2    = o2 ? 1'b0 : cda;
  assign cons2  = cons1 | o2;
  assign y2neg  = cons2 ? zp_neg : z_neg;                // the phase: x + 128, less p if one was taken
  assign y2dec  = cons2 ? c_zpd : c_zd;
  assign ear2   = r2 & e2 & wna & ~cd2 & ~bit1 & (oph1 | ~z_lt);   // at most one bit a cycle
  assign cand2  = r2 & e2 & ~ear2 & oph1 & y2neg;
  // The standing candidate's phase at least -p/8: for one that came in the first half it is, whenever the second
  // half reaches p/8 (its phase is then at least p/8 - 128, and p/8 >= 64)
  assign cel2   = cand1 | cel;
  assign kc2    = r2 & e2 & ~ear2 & oph1 & cd2 & y2dec & cel2;
  assign bit2e  = r2 & e2 & ~ear2 & oph1 & ~y2neg & ~kc2;
  assign bit2t  = r2 & ~e2 & oph1 & cd2 & y2dec;
  assign bnd2   = r2 & e2 & ~ear2 & ~oph1;
  assign bit2   = ear2 | bit2e | bit2t | kc2;

  // outputs
  assign bit3   = bit1 | bit2;
  assign endw3  = endw1 | endw2;
  logic b2v;
  assign b2v    = ear2 ? (rdiff | l2) : kc2 ? (rdiff ? wna : l1) : (rdiff ? (wna | (cd2 & e2)) : l2);
  assign blv3   = bit1 ? (ear1 ? (rdiff | l1) : kc1 ? (rdiff ? wn : l0) : (rdiff ? (wn | (cdb & e1)) : l1)) : b2v;
  assign ran    = run & ~endw1 & ~endw2 & ~end1 & ~(bit2 & (bit_ends | (vio2 & (b2v == vb))));

  // the error of the one pull (a one-hot choice, flat), the gain's wiring, and chain 2 (a carry-select subtract).
  // The first half's pull wins; a cycle with an edge in each half pulls nothing.
  logic signed [IW-1:0] err, sh, base;
  logic [2:0]         k;
  // Written flat in the compares and the flops (the decisions above in sequence are too deep), by the three cases of
  // the first half: A the window opens in it, B it was open, C it is closed (o1 with op is an empty window: no pull).
  // With no pull in a cycle with an edge in each half, a second-half pull on an edge has no first-half edge.
  logic               cA, cB, cC, u_x, u_p, u_ce, u_h, u_zp, u_z, u_zh;
  assign cA   = run & ~two & o1 & ~op;
  assign cB   = run & ~two & ~o1 & op;
  assign cC   = run & ~two & ~o1 & ~op;
  assign u_x  = (cB & e1 & ((wn & ~cd) | (~x_neg & ~(cd & c_xd & cel)))) |                 // first half: x
                (cB & e1 & ~(wn & ~cd) & x_neg & ~e2 & c_zd & ~Zc);                        // its candidate, second
  assign u_p  = (cA & e1 & (wn | ~p_neg)) | (cC & e1 & wn & ~x_lt) |                       // first half: x - p
                (cA & e1 & ~wn & p_neg & ~e2 & c_zpd);                                     // its candidate, second
  assign u_ce = (cB & cd & c_xd & (~e1 | cel)) |                                           // first half: RXCE
                (cB & ~e1 & cd & ~c_xd & c_zd & ~Zc & (~e2 | cel));                       // second half
  assign u_h  = cC & e1 & ~(wn & ~x_lt);                                                  // first-half boundary
  assign u_zp = ~e1 & e2 & ((cA & (wn | ~zp_neg)) | (cC & wn & ~z_lt));    // second half: x + 128 - p (Zc implies ~z_lt)
  assign u_z  = cB & ~e1 & e2 & ~Zc & ~(cd & c_xd) &                                       // second half: x + 128
                ((wn & ~cd) | (~z_neg & ~(cd & c_zd & cel)));
  assign u_zh = cC & ~e1 & e2 & ~Zc & ~(wn & ~z_lt);                                      // second-half boundary
  assign err  = (x & {IW{u_x}}) | (s_p & {IW{u_p}}) | (cex & {IW{u_ce}}) | (s_h & {IW{u_h}}) |
                (s_zp & {IW{u_zp}}) | (s_z & {IW{u_z}}) | (s_zh & {IW{u_zh}});
  assign k    = (rgain == 2'd0) ? 3'd0 : ~g[4] & (g[3:2] == 2'b00) ? 3'd1 :
                (rgain == 2'd1) ? 3'd2 : (rgain == 2'd2) ? 3'd4 : 3'd5;
  assign sh   = (k == 3'd0) ? err : (k == 3'd1) ? (err >>> 1) : (k == 3'd2) ? (err >>> 2) :
                (k == 3'd4) ? (err >>> 4) : (err >>> 5);
  logic ended1;
  assign ended1 = endw1 | end1;                          // the first half step ends the reception
  logic tkp;
  // a p taken in either half: an opening, or the structural rule before the window (first half; or second half,
  // after a first half with no edge, or with a boundary edge, which leaves RXWIN 1)
  assign tkp  = o1 | Zc | (~o1 & ~op & ((e1 & wn & ~x_lt) |
                                        (~Zc & e2 & ~z_lt & ((~e1 & wn) | (e1 & ~(wn & ~x_lt))))));
  assign base = ended1 ? (cons1 ? s_zp : s_z) : tkp ? s_bp : s_b0;
  // base - sh (sh is 0 with no pull: err is 0 then), split at bit LS: the high half both ways, chosen by the borrow
  localparam int LS = IW / 2;
  logic [LS-1:0]    lo;
  logic             bw;
  logic [IW-LS-1:0] hi0, hi1, hi;
  logic signed [IW-1:0] rn;
  assign {bw, lo} = {1'b0, base[LS-1:0]} - {1'b0, sh[LS-1:0]};
  assign hi0  = base[IW-1:LS] - sh[IW-1:LS];
  assign hi1  = base[IW-1:LS] - sh[IW-1:LS] - 1'b1;
  assign hi   = bw ? hi1 : hi0;
  assign rn   = {hi, lo};
  assign r3   = {{(18 - IW){rn[IW-1]}}, rn};            // used only while the block runs

  // RXOPEN, RXCAND, RXWIN, RXG and RXCE after the cycle
  assign op3  = ~run ? op : (oph1 & ~bit2 & ~end1);      // a bit that ends it leaves RXOPEN 0
  assign cd3  = ~run ? cd : (cand2 | (cd2 & ~bit2));
  assign wn3  = ~run ? wn : cand2 ? (wna | cd2) : bit2 ? kc2 : bnd2 ? 1'b1 : wna;
  assign g3   = (bit3 & ~g16) ? g1 : g;                  // at most one bit a cycle
  assign ce3  = cand2 ? (cons2 ? s_zp[CW-1:0] : s_z[CW-1:0]) : cand1 ? (o1 ? s_p[CW-1:0] : x[CW-1:0]) : ce;
endmodule
