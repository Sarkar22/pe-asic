/*
 * One hardware thread: its PC, flags, D, S and armed wait, and the event logic that watches them
 * on every clock. pe_exec works out what the slot does. Host commands land last, on what the
 * cycle has left.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_thread
  import pe_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic [15:0] tb,
    input  logic [15:0] sync2,
    input  logic        inbox_full,     // FULL of this thread's own mailbox word
    input  logic        ex_sel,         // this thread owns stage 4
    input  logic        mem_sel,        // this thread owns stage 1
    // What the slot does, when ex_sel is set.
    input  logic        x_stop,         // stop()
    input  logic        x_step_end,     // a retire ends a STEP
    input  logic        x_wake,         // a wait retires
    input  logic        x_ill,
    input  logic        x_pc_we,
    input  logic [8:0]  x_pc,
    input  logic        x_t_we,
    input  logic        x_t,
    input  logic        x_d_we,
    input  logic [15:0] x_d,
    input  logic        x_late_set,
    input  logic        x_late_clr,
    input  logic        x_arm,          // P := 1
    input  logic        x_kind,
    input  logic        x_due_we,
    input  logic        x_due,
    input  logic        x_wait,         // a wait arms
    input  logic [7:0]  x_mask,
    input  logic [7:0]  x_pat,
    input  logic        x_bank,
    input  logic        x_ne,
    input  logic        x_timed,
    input  logic        x_inbox,
    input  logic        x_mem_start,
    input  logic        x_mem_kind,
    input  logic [2:0]  x_mem_reg,
    input  logic        x_mem_end,
    // Host commands. They act in the cycle they are set.
    input  logic        h_stop,
    input  logic        h_start,
    input  logic        h_step,
    input  logic        h_abort,
    input  logic        h_setpc,
    input  logic [8:0]  h_pc,
    // State.
    output logic [8:0]  pc,
    output logic        t,
    output logic [15:0] d,
    output logic [15:0] s,
    output logic        late,
    output logic        stale,
    output logic        ill,
    output logic        run,
    output logic        p,
    output logic        waiting,
    output logic        stopreq,
    output logic        stepping,
    output logic        mem,
    output logic        mem_done,
    output logic        mem_kind,
    output logic [2:0]  mem_reg,
    output logic        run_post,       // RUN after this cycle's slot, as a host command sees it
    output logic        commit          // this thread's prepared pins commit in this cycle
);

  logic [8:0]  pc_q, pc_d;
  logic        t_q, t_d;
  logic [15:0] d_q, d_d;
  logic [15:0] s_q;
  logic        late_q, late_d;
  logic        stale_q, stale_d;
  logic        ill_q, ill_d;
  logic        run_q, run_d;
  logic        p_q, p_d;
  logic        kind_q, due_q;
  logic        waiting_q, waiting_d;
  logic [7:0]  mask_q, pat_q;
  logic        bank_q, ne_q, timed_q, inbox_q;
  logic        stopreq_q, stopreq_d;
  logic        stepping_q, stepping_d;
  logic        mem_q, mem_done_q, mem_kind_q;
  logic [2:0]  mem_reg_q;

  assign pc       = pc_q;
  assign t        = t_q;
  assign d        = d_q;
  assign s        = s_q;
  assign late     = late_q;
  assign stale    = stale_q;
  assign ill      = ill_q;
  assign run      = run_q;
  assign p        = p_q;
  assign waiting  = waiting_q;
  assign stopreq  = stopreq_q;
  assign stepping = stepping_q;
  assign mem      = mem_q;
  assign mem_done = mem_done_q;
  assign mem_kind = mem_kind_q;
  assign mem_reg  = mem_reg_q;

  // Step 2. D == TB and the STALE test D xor 0x8000 == TB share the low 15 bits.
  logic       low_eq, at_d, hit, deadline;
  logic [7:0] watched;
  logic       match, inbox_hit;
  logic       tick_fire, wait_match, wait_inbox, wait_timeout, fire, event_d;

  assign low_eq   = d_q[14:0] == tb[14:0];
  assign at_d     = low_eq & (d_q[15] == tb[15]);
  assign hit      = low_eq & (d_q[15] != tb[15]);
  assign deadline = due_q | at_d;

  assign watched   = bank_q ? sync2[15:8] : sync2[7:0];
  assign match     = (((watched ^ pat_q) & mask_q) == 8'h00) ^ ne_q;
  assign inbox_hit = inbox_q & inbox_full;

  assign tick_fire    = p_q & (kind_q == KIND_TICK) & deadline;
  assign wait_match   = p_q & (kind_q == KIND_WAIT) & match;
  assign wait_inbox   = p_q & (kind_q == KIND_WAIT) & ~match & inbox_hit;
  assign wait_timeout = p_q & (kind_q == KIND_WAIT) & ~match & ~inbox_hit & timed_q & deadline;

  assign fire    = tick_fire | wait_match | wait_inbox | wait_timeout;
  assign commit  = tick_fire | wait_match;
  assign event_d = wait_match | wait_inbox;

  logic stop_now, step_end;

  assign stop_now = ex_sel & x_stop;
  assign step_end = ex_sel & x_step_end;

  assign run_post = run_q & ~stop_now & ~step_end;

  always_comb begin
    // Step 2: the event.
    t_d = t_q;
    if (wait_match) t_d = 1'b1;
    if (wait_inbox | wait_timeout) t_d = 1'b0;
    d_d       = event_d ? tb : d_q;
    p_d       = p_q & ~fire;
    late_d    = late_q | (tick_fire & due_q);
    pc_d      = pc_q;
    ill_d     = ill_q;
    run_d     = run_post;
    waiting_d = waiting_q;

    // Step 3: the slot. stop() is the one read that sees step 2, through p_d.
    stopreq_d  = stopreq_q & ~stop_now & ~step_end;
    stepping_d = stepping_q & ~stop_now & ~step_end;
    if (ex_sel) begin
      if (x_pc_we) pc_d = x_pc;
      if (x_t_we) t_d = x_t;
      if (x_d_we) d_d = x_d;
      if (x_late_set) late_d = 1'b1;
      if (x_late_clr) late_d = 1'b0;
      if (x_ill) ill_d = 1'b1;
      if (x_wake) waiting_d = 1'b0;
      if (x_stop) begin
        if (!p_d) waiting_d = 1'b0;
        p_d = 1'b0;
      end
      if (x_step_end && stopreq_q) p_d = 1'b0;
      if (x_arm) p_d = 1'b1;
      if (x_wait) waiting_d = 1'b1;
    end

    // Step 5.
    stale_d = stale_q;
    if (event_d | (ex_sel & x_d_we)) stale_d = 1'b0;
    else if (hit) stale_d = 1'b1;

    // After step 5: the host, which may overwrite any of it.
    if (h_stop) begin
      if (run_post) stopreq_d = 1'b1;
      else p_d = 1'b0;
    end else if (h_start && !run_post) begin
      run_d     = 1'b1;
      d_d       = tb;
      p_d       = 1'b0;
      waiting_d = 1'b0;
      stopreq_d = 1'b0;
      late_d    = 1'b0;
      ill_d     = 1'b0;
      stale_d   = 1'b0;
    end
    if (h_step && !run_post) begin
      run_d      = 1'b1;
      stepping_d = 1'b1;
      waiting_d  = 1'b0;
    end
    if (h_abort) begin
      p_d       = 1'b0;
      waiting_d = 1'b0;
      if (run_post) stopreq_d = 1'b1;
    end
    if (h_setpc && !run_post) pc_d = h_pc;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pc_q       <= 9'd0;
      t_q        <= 1'b0;
      d_q        <= 16'h0000;
      s_q        <= 16'h0000;
      late_q     <= 1'b0;
      stale_q    <= 1'b0;
      ill_q      <= 1'b0;
      run_q      <= 1'b0;
      p_q        <= 1'b0;
      kind_q     <= KIND_TICK;
      due_q      <= 1'b0;
      waiting_q  <= 1'b0;
      mask_q     <= 8'h00;
      pat_q      <= 8'h00;
      bank_q     <= 1'b0;
      ne_q       <= 1'b0;
      timed_q    <= 1'b0;
      inbox_q    <= 1'b0;
      stopreq_q  <= 1'b0;
      stepping_q <= 1'b0;
      mem_q      <= 1'b0;
      mem_done_q <= 1'b0;
      mem_kind_q <= MEM_LOAD;
      mem_reg_q  <= 3'd0;
    end else begin
      pc_q       <= pc_d;
      t_q        <= t_d;
      d_q        <= d_d;
      late_q     <= late_d;
      stale_q    <= stale_d;
      ill_q      <= ill_d;
      run_q      <= run_d;
      p_q        <= p_d;
      waiting_q  <= waiting_d;
      stopreq_q  <= stopreq_d;
      stepping_q <= stepping_d;
      if (fire) s_q <= sync2;
      if (ex_sel) begin
        if (x_arm) kind_q <= x_kind;
        if (x_due_we) due_q <= x_due;
        if (x_wait) begin
          mask_q  <= x_mask;
          pat_q   <= x_pat;
          bank_q  <= x_bank;
          ne_q    <= x_ne;
          timed_q <= x_timed;
          inbox_q <= x_inbox;
        end
        if (x_mem_start) begin
          mem_q      <= 1'b1;
          mem_kind_q <= x_mem_kind;
          if (x_mem_kind == MEM_LOAD) mem_reg_q <= x_mem_reg;
        end
        if (x_mem_end) begin
          mem_q      <= 1'b0;
          mem_done_q <= 1'b0;
        end
      end
      if (mem_sel && mem_q) mem_done_q <= 1'b1;
    end
  end

endmodule
