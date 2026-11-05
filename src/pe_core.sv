/*
 * The protocol engine: four threads in fixed rotation sharing one four-stage pipeline.
 * With phase = TB[1:0], thread phase is in stage 4 (execute), phase + 1 in stage 3 (decode, read
 * registers), phase + 2 in stage 2 (capture the word) and phase + 3 in stage 1 (memory cycle).
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_core
  import pe_pkg::*;
  import pe_op_pkg::*;
(
    input  logic [7:0] ui_in,    // Dedicated inputs
    output logic [7:0] uo_out,   // Dedicated outputs
    input  logic [7:0] uio_in,   // IOs: Input path
    output logic [7:0] uio_out,  // IOs: Output path
    output logic [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  logic       ena,      // always 1 when the design is powered, so you can ignore it
    input  logic       clk,      // clock
    input  logic       rst_n     // reset_n - low to reset
);

  // Reset: asserted asynchronously, released synchronously.
  logic [1:0] rst_sync;
  logic       rst_n_sync;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rst_sync <= 2'b00;
    else        rst_sync <= {rst_sync[0], 1'b1};
  end
  assign rst_n_sync = rst_sync[1];

  // Timebase and input synchroniser.
  logic [15:0] tb_q, tb_next;
  logic [15:0] sync1_q, sync2_q;

  assign tb_next = tb_q + 16'd1;

  always_ff @(posedge clk or negedge rst_n_sync) begin
    if (!rst_n_sync) begin
      tb_q    <= 16'h0000;
      sync1_q <= 16'h0000;
      sync2_q <= 16'h0000;
    end else begin
      tb_q    <= tb_next;
      sync1_q <= {uio_in, ui_in};
      sync2_q <= sync1_q;
    end
  end

  // Which thread owns which stage in this cycle.
  logic [1:0] th4, th3, th1;

  assign th4 = tb_q[1:0];
  assign th3 = th4 + 2'd1;
  assign th1 = th4 + 2'd3;

  // Per-thread state, as pe_thread shows it.
  logic [8:0]  th_pc [0:3];
  logic [15:0] th_d [0:3];
  logic [15:0] th_s [0:3];
  logic [2:0]  th_mem_reg [0:3];
  logic [3:0]  th_t, th_late, th_stale, th_ill, th_run, th_p, th_waiting, th_stopreq, th_stepping;
  logic [3:0]  th_mem, th_mem_done, th_mem_kind, th_run_post, th_commit;

  // Stage 1: the memory cycle. One access per cycle, for the thread that owns the stage.
  logic        data_op, fetch, mem_free;
  logic        host_req, host_write, host_grant;
  logic [8:0]  host_addr;
  logic [15:0] host_wdata;
  logic [8:0]  ls_addr_q;
  logic [15:0] ls_data_q;
  logic        sram_en, sram_wen, sram_ren;
  logic [8:0]  sram_addr;
  logic [15:0] sram_din, sram_dout;

  assign data_op    = th_mem[th1] & ~th_mem_done[th1];
  assign fetch      = ~th_mem[th1] & th_run[th1];
  assign mem_free   = ~th_mem[th1] & ~th_run[th1];
  assign host_grant = mem_free & host_req;

  assign sram_en  = data_op | fetch | host_grant;
  assign sram_wen = (data_op & (th_mem_kind[th1] == MEM_STORE)) | (host_grant & host_write);
  assign sram_ren = (data_op & (th_mem_kind[th1] == MEM_LOAD)) | fetch | (host_grant & ~host_write);

  always_comb begin
    if (data_op) sram_addr = ls_addr_q;
    else if (fetch) sram_addr = th_pc[th1];
    else sram_addr = host_addr;
  end

  assign sram_din = host_grant ? host_wdata : ls_data_q;

  pe_mem mem (
      .clk (clk),
      .en  (sram_en),
      .wen (sram_wen),
      .ren (sram_ren),
      .addr(sram_addr),
      .din (sram_din),
      .dout(sram_dout)
  );

  // Stages 2 to 4. Each stage holds one thread, so the word needs no per-thread copy. Load data
  // takes the same path as a fetched word and is in stage 4 at the load's second slot.
  logic               valid2_q, valid3_q, valid4_q;
  logic [15:0]        ir3_q, ir4_q;
  logic [OP_BITS-1:0] op3, op4_q;
  logic               illegal3, illegal4_q;
  logic               world3, world4_q;
  logic [2:0]         ra3, rb3, ra4_q, rb4_q;
  logic [15:0]        rf_a, rf_b, a4_q, b4_q;

  logic               rd_req, rd_here;
  logic [1:0]         rd_thread;
  logic [2:0]         rd_reg;

  pe_decode decode (
      .word   (ir3_q),
      .op     (op3),
      .illegal(illegal3),
      .world  (world3),
      .ra     (ra3),
      .rb     (rb3)
  );

  assign rd_here = rd_req & (rd_thread == th3);

  logic               rf_we;
  logic [2:0]         rf_reg;
  logic [15:0]        rf_data;

  pe_regfile regfile (
      .clk    (clk),
      .rst_n  (rst_n_sync),
      .raddr_a({th3, rd_here ? rd_reg : ra3}),
      .raddr_b({th3, rb3}),
      .rdata_a(rf_a),
      .rdata_b(rf_b),
      .we     (rf_we),
      .waddr  ({th4, rf_reg}),
      .wdata  (rf_data)
  );

  logic        ls_start;
  logic [8:0]  ls_addr;
  logic [15:0] ls_data;

  always_ff @(posedge clk or negedge rst_n_sync) begin
    if (!rst_n_sync) begin
      valid2_q   <= 1'b0;
      valid3_q   <= 1'b0;
      valid4_q   <= 1'b0;
      ir3_q      <= 16'h0000;
      ir4_q      <= 16'h0000;
      op4_q      <= OP_IDLE;
      illegal4_q <= 1'b0;
      world4_q   <= 1'b0;
      ra4_q      <= 3'd0;
      rb4_q      <= 3'd0;
      a4_q       <= 16'h0000;
      b4_q       <= 16'h0000;
      ls_addr_q  <= 9'd0;
      ls_data_q  <= 16'h0000;
    end else begin
      valid2_q   <= fetch;
      valid3_q   <= valid2_q;
      valid4_q   <= valid3_q;
      ir3_q      <= sram_dout;
      ir4_q      <= ir3_q;
      op4_q      <= op3;
      illegal4_q <= illegal3;
      world4_q   <= world3;
      ra4_q      <= ra3;
      rb4_q      <= rb3;
      a4_q       <= rf_a;
      b4_q       <= rf_b;
      if (ls_start) begin
        ls_addr_q <= ls_addr;
        ls_data_q <= ls_data;
      end
    end
  end

  // Stage 4.
  logic        m_put, m_take, m_full;
  logic [15:0] m_data;
  logic [7:0]  full;
  logic [3:0]  pin;
  logic        pin_put, pin_value, pin_put8, pin_oe, pin_oe8, pin_cfg, shd_pin;

  logic        x_stop, x_step_end, x_wake, x_ill;
  logic        x_pc_we, x_t_we, x_t, x_d_we, x_late_set, x_late_clr;
  logic [8:0]  x_pc;
  logic [15:0] x_d;
  logic        x_arm, x_kind, x_due_we, x_due;
  logic        x_wait, x_bank, x_ne, x_timed, x_inbox;
  logic [7:0]  x_mask, x_pat;
  logic        x_mem_kind, x_mem_end;

  pe_exec exec (
      .valid     (valid4_q),
      .op        (op4_q),
      .illegal   (illegal4_q),
      .world     (world4_q),
      .ir        (ir4_q),
      .ra        (ra4_q),
      .rb        (rb4_q),
      .a         (a4_q),
      .b         (b4_q),
      .tid       (th4),
      .pc        (th_pc[th4]),
      .t         (th_t[th4]),
      .d         (th_d[th4]),
      .s         (th_s[th4]),
      .late      (th_late[th4]),
      .stale     (th_stale[th4]),
      .run       (th_run[th4]),
      .p         (th_p[th4]),
      .waiting   (th_waiting[th4]),
      .stopreq   (th_stopreq[th4]),
      .stepping  (th_stepping[th4]),
      .mem       (th_mem[th4]),
      .mem_done  (th_mem_done[th4]),
      .mem_kind  (th_mem_kind[th4]),
      .mem_reg   (th_mem_reg[th4]),
      .tb        (tb_q),
      .tb_next   (tb_next),
      .m_full    (m_full),
      .m_data    (m_data),
      .m_put     (m_put),
      .m_take    (m_take),
      .shd_pin   (shd_pin),
      .pin       (pin),
      .pin_put   (pin_put),
      .pin_value (pin_value),
      .pin_put8  (pin_put8),
      .pin_oe    (pin_oe),
      .pin_oe8   (pin_oe8),
      .pin_cfg   (pin_cfg),
      .rf_we     (rf_we),
      .rf_reg    (rf_reg),
      .rf_data   (rf_data),
      .ls_start  (ls_start),
      .ls_addr   (ls_addr),
      .ls_data   (ls_data),
      .x_stop    (x_stop),
      .x_step_end(x_step_end),
      .x_wake    (x_wake),
      .x_ill     (x_ill),
      .x_pc_we   (x_pc_we),
      .x_pc      (x_pc),
      .x_t_we    (x_t_we),
      .x_t       (x_t),
      .x_d_we    (x_d_we),
      .x_d       (x_d),
      .x_late_set(x_late_set),
      .x_late_clr(x_late_clr),
      .x_arm     (x_arm),
      .x_kind    (x_kind),
      .x_due_we  (x_due_we),
      .x_due     (x_due),
      .x_wait    (x_wait),
      .x_mask    (x_mask),
      .x_pat     (x_pat),
      .x_bank    (x_bank),
      .x_ne      (x_ne),
      .x_timed   (x_timed),
      .x_inbox   (x_inbox),
      .x_mem_kind(x_mem_kind),
      .x_mem_end (x_mem_end)
  );

  // The threads.
  logic [3:0] h_stop, h_start, h_step, h_setpc;
  logic       h_abort;
  logic [8:0] h_pc;

  for (genvar i = 0; i < 4; i = i + 1) begin : gen_thread
    pe_thread thread (
        .clk        (clk),
        .rst_n      (rst_n_sync),
        .tb         (tb_q),
        .sync2      (sync2_q),
        .inbox_full (full[i]),
        .ex_sel     (th4 == 2'(i)),
        .mem_sel    (th1 == 2'(i)),
        .x_stop     (x_stop),
        .x_step_end (x_step_end),
        .x_wake     (x_wake),
        .x_ill      (x_ill),
        .x_pc_we    (x_pc_we),
        .x_pc       (x_pc),
        .x_t_we     (x_t_we),
        .x_t        (x_t),
        .x_d_we     (x_d_we),
        .x_d        (x_d),
        .x_late_set (x_late_set),
        .x_late_clr (x_late_clr),
        .x_arm      (x_arm),
        .x_kind     (x_kind),
        .x_due_we   (x_due_we),
        .x_due      (x_due),
        .x_wait     (x_wait),
        .x_mask     (x_mask),
        .x_pat      (x_pat),
        .x_bank     (x_bank),
        .x_ne       (x_ne),
        .x_timed    (x_timed),
        .x_inbox    (x_inbox),
        .x_mem_start(ls_start),
        .x_mem_kind (x_mem_kind),
        .x_mem_reg  (ra4_q),
        .x_mem_end  (x_mem_end),
        .h_stop     (h_stop[i]),
        .h_start    (h_start[i]),
        .h_step     (h_step[i]),
        .h_abort    (h_abort),
        .h_setpc    (h_setpc[i]),
        .h_pc       (h_pc),
        .pc         (th_pc[i]),
        .t          (th_t[i]),
        .d          (th_d[i]),
        .s          (th_s[i]),
        .late       (th_late[i]),
        .stale      (th_stale[i]),
        .ill        (th_ill[i]),
        .run        (th_run[i]),
        .p          (th_p[i]),
        .waiting    (th_waiting[i]),
        .stopreq    (th_stopreq[i]),
        .stepping   (th_stepping[i]),
        .mem        (th_mem[i]),
        .mem_done   (th_mem_done[i]),
        .mem_kind   (th_mem_kind[i]),
        .mem_reg    (th_mem_reg[i]),
        .run_post   (th_run_post[i]),
        .commit     (th_commit[i])
    );
  end

  // Mailbox.
  logic [2:0]  hm_idx;
  logic        hm_post, hm_take, hm_full;
  logic [15:0] hm_wdata, hm_rdata;

  pe_mailbox mailbox (
      .clk    (clk),
      .rst_n  (rst_n_sync),
      .idx    (ir4_q[7:5]),
      .put    (m_put),
      .take   (m_take),
      .wdata  (a4_q),
      .rdata  (m_data),
      .rfull  (m_full),
      .full   (full),
      .h_idx  (hm_idx),
      .h_post (hm_post),
      .h_take (hm_take),
      .h_wdata(hm_wdata),
      .h_full (hm_full),
      .h_rdata(hm_rdata)
  );

  // Pins.
  logic miso;

  pe_pins pins (
      .clk     (clk),
      .rst_n   (rst_n_sync),
      .ui_in   (ui_in),
      .miso    (miso),
      .uo_out  (uo_out),
      .uio_out (uio_out),
      .uio_oe  (uio_oe),
      .commit  (th_commit),
      .tid     (th4),
      .pin     (pin),
      .put     (pin_put),
      .value   (pin_value),
      .put8    (pin_put8),
      .bank    (ir4_q[0]),
      .data    (a4_q[7:0]),
      .oe      (pin_oe),
      .oe8     (pin_oe8),
      .cfg     (pin_cfg),
      .cfg_mode(ir4_q[4:3]),
      .shd_pin (shd_pin),
      .h_abort (h_abort)
  );

  // Host link: ui0 is CS, ui1 SCK and ui2 MOSI, taken after the synchroniser.
  pe_host host (
      .clk      (clk),
      .rst_n    (rst_n_sync),
      .cs_n     (sync2_q[0]),
      .sck      (sync2_q[1]),
      .mosi     (sync2_q[2]),
      .miso     (miso),
      .run      (th_run),
      .ill      (th_ill),
      .late     (th_late),
      .stale    (th_stale),
      .run_post (th_run_post),
      .h_stop   (h_stop),
      .h_start  (h_start),
      .h_step   (h_step),
      .h_setpc  (h_setpc),
      .h_abort  (h_abort),
      .h_pc     (h_pc),
      .full     (full),
      .m_full   (hm_full),
      .m_rdata  (hm_rdata),
      .m_idx    (hm_idx),
      .m_post   (hm_post),
      .m_take   (hm_take),
      .m_wdata  (hm_wdata),
      .mem_req  (host_req),
      .mem_write(host_write),
      .mem_addr (host_addr),
      .mem_wdata(host_wdata),
      .mem_grant(host_grant),
      .mem_rdata(sram_dout),
      .rd_req   (rd_req),
      .rd_thread(rd_thread),
      .rd_reg   (rd_reg),
      .rd_here  (rd_here),
      .ex_a     (a4_q),
      .ex_d     (th_d[th4]),
      .ex_s     (th_s[th4]),
      .ex_pc    (th_pc[th4]),
      .ex_flags ({th_waiting[th4], th_stale[th4], th_late[th4], th_t[th4]})
  );

  // List all unused inputs to prevent warnings
  logic _unused;
  assign _unused = &{ena, 1'b0};

endmodule
