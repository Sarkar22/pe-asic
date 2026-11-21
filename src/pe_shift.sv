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
 * flops of their own, and forms the decoder's arithmetic in 17 bits (the implementation note of
 * section 5.9.6).
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
 *   CUT_EOP     the end test: REOP
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
  logic [15:0] rx_sr8_q, rx_c8_q;     // RXSR8 and RXC8 (section 5.9.7, the byte end)
  logic [4:0]  rx_n8_q;               // RXN8
  logic        rx_ones_q;             // RXONES

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
  assign tpull = trun & e & resync;
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
  // The byte end (section 5.9.7): at the end test, with 1 to 6 ones after the last byte boundary and no stuff
  // violation, rx_eop() puts RXSR, RXN and the register's bits 31 to 16 back to their copies before the check. No
  // bit enters the register in that cycle, so the check of the copy is formed from flops beside the other.
  logic        eop_rest, c8_on;
  logic [31:0] crc_e;

  assign c8_on    = ~ctx & cmode_on;
  assign eop_rest = eop_hit & rx_ones_q & ~rx_vio_q & (rx_tot_q[2:0] != 3'd0) & (rx_tot_q[2:0] != 3'd7);
  assign crc_e    = {(eop_rest & c8_on) ? rx_c8_q : crc_r[31:16], crc_r[15:0]};
  assign crc_pass = cm1 & ~ctx & ((eop_rest & c8_on) ? ({rx_c8_q, crc_q[15:0]} == chk) : (crc_r == chk));

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

  // RKIND 3 (section 5.9.6): rx_manch() twice a cycle, half a cycle each (s = 128), first with the falling-edge
  // sample SYNC2F, then with SYNC2. The cycle's one edge, SYNC2 against SYNC3, falls in the first half if SYNC2F
  // already shows the new level, and in the second if it still shows the old one. RPER 0 and 1 act as 2, so p is
  // 512 and the window points 384 and 256. With p at least 512 a cycle has at most one bit and one pull (one edge),
  // and the second half step runs only while the reception goes on: a window that closes empty, or a bit that ends
  // the reception (the bit after a stuff violation, or RCNT's), ends it, and the half steps after it don't run.
  // The bit and the end come from the compares below; RXPH and the window flags from one pull, fed by the half step
  // that has the edge (the implementation note's shape).
  logic               lvlf, e1h, e2h, clamp3, bit_ends;
  logic [15:0]        p3, pq3, ph3w;
  logic signed [17:0] r3;
  logic               op3, cd3, wn3;
  logic [4:0]         g3;

  assign lvlf     = sync2f[rpin];
  assign e1h      = e & (lvlf == lvl2);
  assign e2h      = e & ~e1h;
  assign clamp3   = rper[15:1] == 15'd0;
  assign p3       = clamp3 ? 16'd512 : {rs, rfrac};
  assign pq3      = clamp3 ? 16'd384 : pwq_q;
  assign ph3w     = clamp3 ? 16'd256 : pwh_q;
  // A bit that ends the reception: the bit after a violation, or a data bit that reaches RCNT. Flops only.
  assign bit_ends = vio0 | (~stuffpos & syn0 & (rcnt != 11'd0) & (tot1 == rcnt));

  // The same cycle's bit and end, from compares that all start at the flops, for the long path behind the bit (the
  // destuffer, the CRC check, the word). Whatever the half steps do to RXPH in between is folded into thresholds:
  // - First half: the window opens if RXPH >= w1 (o1), then RXPH or RXPH - p decides as it stands.
  // - Second half after a first-half candidate (an edge before the predicted clock edge, pulled toward 0): the bit
  //   comes now if the pulled phase plus 128 is 0 or more. rx_pull moves x < 0 to x - (x >> k), which is monotone
  //   in x, so that is x >= -M with M = 257, 171, 137, 133 for k = 1, 2, 4, 5, and always with RGAIN 0. Its window
  //   can't open there: the pulled phase is at most 0, and the window points are 256 or more.
  // - Second half with no edge in the first: RXPH + 128, less p for each window opened, against w1 and 0.
  // - A first-half edge that is no candidate and no bit is a boundary edge: the window was closed, so the second
  //   half can open it but has no candidate and no edge, so no bit.
  // With p at least 512 a cycle has at most one bit, and the code's window points w <= p - 128 keep a second-half
  // bit out of a cycle whose first half gave one. The code's half steps, run one after the other, give the same bit
  // and end in every state (a combinational equivalence check, not part of this file).
  logic               f_o1, f_sr, f_sa1, f_end1, f_op1, f_cd1, f_pos1, f_bit1, f_cand1, f_run2, f_thr, f_bit2a;
  logic               f_nb, f_o2, f_end2, f_op2, f_cd2, f_pos2, f_bit2b, f_acq;
  logic signed [18:0] f_r, f_a1, f_dw, f_dwa, f_t0, f_t1;
  logic [15:0]        f_w1;
  logic [8:0]         f_m;

  assign f_acq   = rx_g_q[4:2] == 3'd0;                         // RXG <= 3
  assign f_w1    = (racq & (rgain != 2'd0) & ~rx_g_q[4]) ? ph3w : pq3;
  assign f_r     = {ph[17], ph};
  assign f_a1    = f_r - $signed({3'b000, p3});                 // RXPH - p
  assign f_dw    = f_r - $signed({3'b000, f_w1});               // RXPH - w1
  assign f_dwa   = f_r - $signed({3'b000, p3}) - $signed({3'b000, f_w1});    // RXPH - p - w1, in one sum
  assign f_m     = f_acq ? 9'd257 : (rgain == 2'd1) ? 9'd171 : (rgain == 2'd2) ? 9'd137 : 9'd133;
  assign f_t0    = f_r + $signed({10'd0, f_m});                 // RXPH + M
  assign f_t1    = f_r - $signed({3'b000, p3}) + $signed({10'd0, f_m});      // RXPH - p + M, in one sum
  assign f_o1    = ~f_dw[18];
  assign f_sr    = ~f_r[18];
  assign f_sa1   = ~f_a1[18];
  assign f_end1  = f_o1 & rx_open_q;
  assign f_op1   = rx_open_q | f_o1;
  assign f_cd1   = ~f_o1 & rx_cand_q;
  assign f_pos1  = f_o1 ? f_sa1 : f_sr;
  assign f_bit1  = ~f_end1 & f_op1 & f_pos1 & (e1h | f_cd1);
  assign f_cand1 = ~f_end1 & f_op1 & e1h & ~f_pos1;
  assign f_run2  = ~f_end1 & ~(f_bit1 & bit_ends);
  assign f_thr   = (rgain == 2'd0) | ~(f_o1 ? f_t1[18] : f_t0[18]);
  assign f_bit2a = f_run2 & f_cand1 & f_thr;
  assign f_nb    = f_run2 & ~e1h & ~f_bit1;
  assign f_o2    = f_o1 ? (f_dwa >= -19'sd128) : (f_dw >= -19'sd128);    // RXPH (- p) + 128 >= w1
  assign f_end2  = f_nb & f_o2 & f_op1;
  assign f_op2   = f_op1 | f_o2;
  assign f_cd2   = ~f_o2 & f_cd1;
  assign f_pos2  = (f_o1 | f_o2) ? (f_a1 >= -19'sd128) : (f_r >= -19'sd128);
  assign f_bit2b = f_nb & ~f_end2 & f_op2 & f_pos2 & (e2h | f_cd2);
  assign bit3    = b_run & man3 & (f_bit1 | f_bit2a | f_bit2b);
  assign endw3   = b_run & man3 & (f_end1 | f_end2);
  assign blv3    = f_bit1 ? (rdiff ? rx_win_q : lvlf) : rdiff ? (rx_win_q | (f_bit2a & f_cd1)) : lvl2;

  // RXPH, RXOPEN, RXCAND, RXWIN and RXG after the two half steps, with one rx_pull. A cycle's edge is in one half,
  // so only that half pulls: from the first half's phase if it has the edge, else from the second half's, whose
  // window opened (or not) on the first half's phase + 128 with no pull. Every phase the code can reach without the
  // pull is formed from the flops, beside the window compares: RXPH + 128 and + 256, less 0, p or 2p (one window
  // opening per half step), and the boundary pulls' errors less p/2. So the pull's error is a choice, then the
  // gain's shift, and what follows the pull (the phase + 128 or + 256, less p if the second half's window opens on
  // it, and that compare) are sums of three that run side by side. The second half's bit is the fast block's. The
  // same values as the code's half steps in every state (a combinational equivalence check, not part of this file).
  logic               s_c1, s_k1, s_b1, s_pl1, s_opa, s_cda, s_wna, s_run2, s_ob, s_o2, s_op2, s_cd2, s_w1h, s_w2h;
  logic               s_h2, s_c2, s_k2, s_b2, s_pl2, s_tp, s_oa, s_neg2b;
  logic [4:0]         s_g1, s_gp;
  logic [15:0]        s_w2;
  logic signed [18:0] q_p, q_p2, q_r128, q_a128, q_aa128, q_rq, q_aq, q_bh1, q_bh2, q_r256, q_a256, q_aa256;
  logic signed [18:0] s_r1, s_r1b, s_r2b, s_r2b2, s_x, s_t, s_d, s_sh, s_base, s_f3, s_f1, s_f2, s_fo, s_rn;

  assign q_p     = $signed({3'b000, p3});
  assign q_p2    = $signed({4'b0000, p3[15:1]});
  assign q_r128  = f_r + 19'sd128;                                      // RXPH + 128
  assign q_a128  = f_r - q_p + 19'sd128;                                // RXPH - p + 128
  assign q_aa128 = f_r - q_p - q_p + 19'sd128;                          // RXPH - 2p + 128
  assign q_rq    = f_r - $signed({3'b000, pq3}) + 19'sd128;             // RXPH + 128 - (p - p/4)
  assign q_aq    = f_r - q_p - $signed({3'b000, pq3}) + 19'sd128;       // RXPH - p + 128 - (p - p/4)
  assign q_bh1   = f_r - q_p2;                                          // RXPH - p/2
  assign q_bh2   = f_r - q_p2 + 19'sd128;                               // RXPH + 128 - p/2
  assign q_r256  = f_r + 19'sd256;
  assign q_a256  = f_r - q_p + 19'sd256;
  assign q_aa256 = f_r - q_p - q_p + 19'sd256;

  // The first half's decision, before its pull (the fast block's state).
  assign s_c1   = ~f_end1 & f_op1 & e1h & ~f_pos1;                      // a candidate
  assign s_k1   = ~f_end1 & f_op1 & f_pos1 & (e1h | f_cd1);             // a bit (f_bit1)
  assign s_b1   = ~f_end1 & ~f_op1 & e1h;                               // a boundary edge
  assign s_pl1  = ~f_end1 & e1h;                                        // the first half pulls
  assign s_r1   = f_o1 ? f_a1 : f_r;
  assign s_r1b  = f_o1 ? q_a128 : q_r128;                               // the first half's phase + 128
  assign s_opa  = f_op1 & ~s_k1;
  assign s_cda  = s_c1 | (~s_k1 & f_cd1);
  assign s_wna  = s_c1 ? (rx_win_q | f_cd1) : s_k1 ? 1'b0 : s_b1 ? 1'b1 : rx_win_q;
  assign s_g1   = (s_k1 & ~rx_g_q[4]) ? rx_g_q + 5'd1 : rx_g_q;
  assign s_run2 = ~f_end1 & ~(s_k1 & bit_ends);
  // The second half's window. Its point is the first half's unless the first half's bit ended RACQ's wide window.
  assign s_w1h  = racq & (rgain != 2'd0) & ~rx_g_q[4];
  assign s_w2h  = racq & (rgain != 2'd0) & ~s_g1[4];
  assign s_w2   = s_w2h ? ph3w : pq3;
  assign s_ob   = (s_w1h & ~s_w2h) ? ~(f_o1 ? q_aq[18] : q_rq[18]) : f_o2;   // on the unpulled phase + 128
  assign s_r2b  = f_o1 ? (s_ob ? q_aa128 : q_a128) : (s_ob ? q_a128 : q_r128);
  assign s_r2b2 = f_o1 ? (s_ob ? q_aa256 : q_a256) : (s_ob ? q_a256 : q_r256);    // + 128 more
  assign s_neg2b = s_r2b[18];
  // The one pull: x, the target (0, or p/2 for a boundary edge), the error, the gain's shift.
  assign s_tp   = s_pl1 ? ~f_op1 : ~(s_opa | s_ob);
  assign s_x    = s_pl1 ? s_r1 : s_r2b;
  assign s_t    = s_tp ? q_p2 : 19'sd0;
  assign s_d    = s_tp ? (s_pl1 ? q_bh1 : q_bh2) : s_x;
  assign s_gp   = s_pl1 ? rx_g_q : s_g1;
  assign s_sh   = (rgain == 2'd0) ? 19'sd0 : (s_gp <= 5'd3) ? (s_d >>> 1) : (rgain == 2'd1) ? (s_d >>> 2) :
                  (rgain == 2'd2) ? (s_d >>> 4) : (s_d >>> 5);
  assign s_base = (rgain == 2'd0) ? s_t : s_x;                          // RGAIN 0 pulls all the way
  assign s_f3   = s_base - s_sh + 19'sd128;                             // pulled + 128
  assign s_f1   = s_base - s_sh + 19'sd256;                             // pulled + 256
  assign s_f2   = s_base - s_sh - q_p + 19'sd256;                       // pulled + 256 - p
  assign s_fo   = s_base - s_sh + 19'sd128 - $signed({3'b000, s_w2});   // pulled + 128 - w2
  assign s_oa   = ~s_fo[18];                                            // the second half's window, after a pull
  // The second half.
  assign s_o2   = s_pl1 ? s_oa : s_ob;
  assign s_op2  = s_opa | s_o2;
  assign s_cd2  = ~s_o2 & s_cda;
  assign s_h2   = s_run2 & ~(s_o2 & s_opa);                             // it decides
  assign s_k2   = f_bit2a | f_bit2b;                                    // a bit
  assign s_c2   = s_h2 & s_op2 & e2h & s_neg2b;                         // a candidate (its edge: no first-half pull)
  assign s_b2   = s_h2 & ~s_op2 & e2h;                                  // a boundary edge
  assign s_pl2  = s_h2 & e2h;
  assign s_rn   = ~s_run2 ? (s_pl1 ? s_f3 : s_r1b) : s_pl2 ? s_f3 : s_pl1 ? (s_oa ? s_f2 : s_f1) : s_r2b2;
  assign r3     = s_rn[17:0];
  assign op3    = ~s_run2 ? s_opa : (s_op2 & ~s_k2);
  assign cd3    = ~s_run2 ? s_cda : (s_c2 | (~s_k2 & s_cd2));
  assign wn3    = ~s_run2 ? s_wna : s_c2 ? (s_wna | s_cd2) : s_k2 ? 1'b0 : s_b2 ? 1'b1 : s_wna;
  assign g3     = (s_run2 & s_k2 & ~s_g1[4]) ? s_g1 + 5'd1 : s_g1;

  // RXIC and the ends of a reception, at most one per cycle.
  logic [15:0] ic_inc, ic_run;
  logic        vio_end, rend_end, end_now;
  logic [2:0]  ef_b;

  assign ic_inc   = (rx_ic_q != 16'hFFFF) ? rx_ic_q + 16'd1 : rx_ic_q;
  assign ic_run   = (clk_hit | bit3 | (trun & e) | osamp) ? 16'd0 : ic_inc;
  assign vio_end  = vbit;
  assign rend_end = b_run & ~endw & ~endw3 & ~rcnt_end & ~vio_end & ~eop_hit & (rend != 16'd0) &
                    (ic_run >= rend);
  assign end_now  = endw | endw3 | rcnt_end | rend_end | vio_end | eop_hit;
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
  logic signed [24:0] ph_run, ph_beg;

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
  assign ph_run = man ? {{8{mph3[16]}}, mph3[16:0]} : man3 ? {{8{r3[16]}}, r3[16:0]} : tim ? tnxt[24:0] : rx_ph_q;
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
      rx_c8_q   <= 16'h0000;
      rx_ones_q <= 1'b0;
    end else begin
      // rx_byte() (section 5.9.7): the copies at the start, which rx_cbegin() takes, and after every 8th data bit;
      // RXONES clears at a 0 between. The start's own bit, for the Manchester decoders, is bit 1.
      if (do_begin) begin
        rx_sr8_q  <= 16'h0000;
        rx_n8_q   <= 5'd0;
        if (c8_on) rx_c8_q <= crc_base[31:16];
        rx_ones_q <= ~dbit | bw;
      end else if (dbit) begin
        if (tot1[2:0] == 3'd0) begin
          rx_sr8_q  <= sr_b;
          rx_n8_q   <= n_b;
          if (c8_on) rx_c8_q <= crc_r[31:16];
          rx_ones_q <= 1'b1;
        end else if (!bw) begin
          rx_ones_q <= 1'b0;
        end
      end
      if (rx_upd) begin
        rx_sr_q  <= eop_rest ? rx_sr8_q : sr_b;
        rx_n_q   <= eop_rest ? rx_n8_q : n_b;
        rx_tot_q <= tot_b;
        rx_syn_q <= syn_b;
        rx_rl_q  <= rl_b;
        rx_cov_q <= cov_b;
        rx_ef_q  <= ef_b;
      end
      if (rv_we) rx_rv_q <= rv_b;
      if (do_begin) begin
        rx_kind_q <= rkind;
        rx_ph_q   <= ph_beg;
        rx_g_q    <= 5'd1;
        rx_open_q <= 1'b0;
        rx_cand_q <= 1'b0;
        rx_ic_q   <= 16'h0000;
        rx_last_q <= lvl3;
        rx_win_q  <= 1'b0;
      end else if (b_run) begin
        rx_ph_q <= ph_run;
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
      rx_vio_q  <= (rx_stop | CUT_STUFF) ? 1'b0 : rx_upd ? (vio0 | pvio) : rx_vio_q;
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
  assign _unused = &{mph3[17], tnxt[25], r3[17], tx_h_q, ola_q, olb_q, f_t0[17:0], f_t1[17:0], q_rq[17:0],
                     q_aq[17:0], s_fo[17:0], s_rn[18], 1'b0};

endmodule
