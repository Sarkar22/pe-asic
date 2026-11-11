/*
 * The send half's run length never saturates, so the arm of pe_shift.sv's tx_rl_new that holds it at 7 is dead,
 * which is why line coverage never hits it (docs/COVERAGE.md). One unit, every input free in every cycle, from
 * reset, as in formal/shift_timing.sv.
 *
 * tx_rl_new holds TXRL at 7 only when TXRL is 7 and the bit is equal to the run (section 5.9.7, tx_run_len()). But a
 * run that reaches 7 always has its stuff bit next, since 7 is at least every SN, and a stuff bit is the complement
 * of its run: tx_symbol() sends 1 - TXRV while TXSTF. So TXRL 7 never meets an equal bit:
 *   stuff_bit   a pending stuff bit is the complement of the run: not (TXSTF and TXB == TXRV);
 *   run_7       while a send runs, a run of 7 has its stuff bit pending: not (TXRUN and TXRL == 7 and not TXSTF);
 *   no_equal_7  not (TXRL == 7 and TXB == TXRV), which rules the arm out.
 * The cover task reaches TXRL 7 itself, so the proof is not vacuous.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tx_rl (
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

  // Probes, connected by probes.ys: TXRL, TXB, TXRV, TXSTF and TXRUN of the unit.
  wire [2:0] rl;
  wire       b, rv, stf, run;

  always @* if ($initstate) assume (!rst_n);

`ifdef COVER
  always @* if (rst_n) cover (rl == 3'd7);
`else
  always @* if (rst_n) assert (!(stf && b == rv));                 // stuff_bit
  always @* if (rst_n) assert (!(run && rl == 3'd7 && !stf));      // run_7
  always @* if (rst_n) assert (!(rl == 3'd7 && b == rv));          // no_equal_7
`endif

endmodule
