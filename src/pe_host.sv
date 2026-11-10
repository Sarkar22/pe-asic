/*
 * Host commands, byte by byte, on top of the bit level in pe_spi. A command acts in the cycle its
 * last argument bit is sampled, on what that cycle has left. Memory and register reads wait for a
 * stopped thread's slot, so their results land a few cycles later.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_host
  import pe_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        cs_n,
    input  logic        sck,
    input  logic        mosi,
    output logic        miso,
    // Thread status.
    input  logic [3:0]  run,
    input  logic [3:0]  ill,
    input  logic [3:0]  late,
    input  logic [3:0]  stale,
    input  logic [3:0]  run_post,   // RUN as this cycle's commands see it
    // Thread commands, in force for the one cycle that matters.
    output logic [3:0]  h_stop,
    output logic [3:0]  h_start,
    output logic [3:0]  h_step,
    output logic [3:0]  h_setpc,
    output logic        h_abort,
    output logic [8:0]  h_pc,
    // Mailbox.
    input  logic [7:0]  full,
    input  logic        m_full,     // the word below, after this cycle's thread
    input  logic [15:0] m_rdata,
    output logic [2:0]  m_idx,
    output logic        m_post,
    output logic        m_take,
    output logic [15:0] m_wdata,
    // Memory. mem_grant says this memory cycle is free for the request.
    output logic        mem_req,
    output logic        mem_write,
    output logic [8:0]  mem_addr,
    output logic [15:0] mem_wdata,
    input  logic        mem_grant,
    input  logic [15:0] mem_rdata,  // valid the cycle after a granted read
    // RDREG. rd_req asks for register rd_reg of thread rd_thread on port A of stage 3. The
    // thread's state is at stage 4 in the cycle rd_valid is set.
    output logic        rd_req,
    output logic [1:0]  rd_thread,
    output logic [2:0]  rd_reg,
    output logic [4:0]  rd_unit,    // reg8 bits 4:0, for the shift units' RDREG 12 to 31
    input  logic        rd_here,    // stage 3 serves the request in this cycle
    input  logic [15:0] ex_a,
    input  logic [15:0] ex_d,
    input  logic [15:0] ex_s,
    input  logic [8:0]  ex_pc,
    input  logic [3:0]  ex_flags,
    input  logic [15:0] ex_unit     // what the unit of the thread in stage 4 reads for rd_unit
);

  // What the transaction is doing. C_NONE ignores everything until CS rises.
  localparam logic [3:0] C_NONE   = 4'd0;
  localparam logic [3:0] C_CAPS   = 4'd1;
  localparam logic [3:0] C_WRMEM  = 4'd2;
  localparam logic [3:0] C_RDMEM  = 4'd3;
  localparam logic [3:0] C_RUN    = 4'd4;
  localparam logic [3:0] C_SETPC  = 4'd5;
  localparam logic [3:0] C_STATUS = 4'd6;
  localparam logic [3:0] C_STEP   = 4'd7;
  localparam logic [3:0] C_RDREG  = 4'd8;
  localparam logic [3:0] C_POST   = 4'd9;
  localparam logic [3:0] C_TAKE   = 4'd10;

  function automatic logic [3:0] command_of(input logic [7:0] code);
    casez (code)
      8'h00:       command_of = C_CAPS;
      8'h01:       command_of = C_WRMEM;
      8'h02:       command_of = C_RDMEM;
      8'h10:       command_of = C_RUN;
      8'h11:       command_of = C_SETPC;
      8'h12:       command_of = C_STATUS;
      8'h13:       command_of = C_STEP;
      8'h14:       command_of = C_RDREG;
      8'b00100???: command_of = C_POST;
      8'b00101???: command_of = C_TAKE;
      default:     command_of = C_NONE;
    endcase
  endfunction

  logic       rx_valid;
  logic [7:0] rx_byte;
  logic       rx_idle;
  logic [7:0] tx_byte;

  pe_spi spi (
      .clk     (clk),
      .rst_n   (rst_n),
      .cs_n    (cs_n),
      .sck     (sck),
      .mosi    (mosi),
      .tx_byte (tx_byte),
      .rx_valid(rx_valid),
      .rx_byte (rx_byte),
      .rx_idle (rx_idle),
      .miso    (miso)
  );

  logic [2:0]  idx_q;       // byte of the transaction, 6 and 7 repeat
  logic [3:0]  cmd_q;
  logic [2:0]  n_q;         // mailbox word of POST
  logic [7:0]  arg_q;       // the byte before this one
  logic [1:0]  thread_q;
  logic [4:0]  reg_q;
  logic        reg_ok_q;    // reg8 is below 32
  logic [8:0]  addr_q;
  logic [15:0] hold_q;      // reply word, or the word WRMEM is writing
  logic        mem_req_q;
  logic        mem_write_q;
  logic        mem_read_q;  // SRAM data arrives in this cycle
  logic        rd_req_q;
  logic        rd_cap_q;    // stage 4 holds what RDREG asked for
  logic        busy_q;

  logic [3:0] cmd_now;
  logic       first;
  logic       do_run, do_setpc, do_step, do_post, do_take, do_rdreg, do_abort;
  logic       status_end, word_done, addr_done, read_next;
  logic       all_running;
  logic       mem_fail;
  logic       rd_fail;

  assign first   = idx_q == 3'd0;
  assign cmd_now = first ? command_of(rx_byte) : cmd_q;

  assign do_take    = rx_valid & first & (cmd_now == C_TAKE);
  assign do_abort   = rx_valid & first & (rx_byte == 8'h3F);
  assign do_run     = rx_valid & (cmd_q == C_RUN) & (idx_q == 3'd2);
  assign do_setpc   = rx_valid & (cmd_q == C_SETPC) & (idx_q == 3'd3);
  assign do_step    = rx_valid & (cmd_q == C_STEP) & (idx_q == 3'd1);
  assign do_post    = rx_valid & (cmd_q == C_POST) & (idx_q == 3'd2);
  assign do_rdreg   = rx_valid & (cmd_q == C_RDREG) & (idx_q == 3'd2);
  assign status_end = rx_valid & (cmd_q == C_STATUS) & (idx_q == 3'd4);
  assign addr_done  = rx_valid & ((cmd_q == C_WRMEM) | (cmd_q == C_RDMEM)) & (idx_q == 3'd2);
  assign word_done  = rx_valid & (cmd_q == C_WRMEM) & (idx_q >= 3'd4) & ~idx_q[0];
  assign read_next  = rx_valid & (cmd_q == C_RDMEM) & (idx_q >= 3'd5) & idx_q[0];

  assign all_running = &run;
  assign mem_fail    = mem_req_q & all_running;
  assign rd_fail     = run_post[thread_q];

  for (genvar i = 0; i < 4; i = i + 1) begin : gen_thread_cmd
    assign h_stop[i]  = do_run & arg_q[i];
    assign h_start[i] = do_run & rx_byte[i];
    assign h_step[i]  = do_step & (rx_byte[1:0] == 2'(i));
    assign h_setpc[i] = do_setpc & (thread_q == 2'(i));
  end

  assign h_abort = do_abort;
  assign h_pc    = {arg_q[0], rx_byte} & ADDR_MASK;

  assign m_idx   = first ? rx_byte[2:0] : n_q;
  assign m_post  = do_post;
  assign m_take  = do_take;
  assign m_wdata = {arg_q, rx_byte};

  assign mem_req   = mem_req_q;
  assign mem_write = mem_write_q;
  assign mem_addr  = addr_q;
  assign mem_wdata = hold_q;

  assign rd_req    = rd_req_q;
  assign rd_thread = thread_q;
  assign rd_reg    = reg_q[2:0];
  assign rd_unit   = reg_q;

  // The reply byte for the byte that follows this one. MISO must move in this very cycle, so a
  // reply that depends on the command byte is worked out from the byte still arriving.
  always_comb begin
    tx_byte = 8'h00;
    case (cmd_now)
      C_CAPS: begin
        if (first) tx_byte = ISA_VERSION;
        else if (idx_q == 3'd1) tx_byte = {THREADS, LOG2_DEPTH};
      end
      C_STATUS: begin
        case (idx_q)
          3'd0:    tx_byte = {run, ill};
          3'd1:    tx_byte = {late, stale};
          3'd2:    tx_byte = full;
          3'd3:    tx_byte = {7'd0, busy_q};
          default: ;
        endcase
      end
      C_TAKE: begin
        case (idx_q)
          3'd0:    tx_byte = {7'd0, m_full};
          3'd1:    tx_byte = hold_q[15:8];
          3'd2:    tx_byte = hold_q[7:0];
          default: ;
        endcase
      end
      C_POST: begin
        if (idx_q == 3'd2) tx_byte = {7'd0, m_full};
      end
      C_RDREG: begin
        if (idx_q == 3'd3) tx_byte = hold_q[15:8];
        else if (idx_q == 3'd4) tx_byte = hold_q[7:0];
      end
      C_RDMEM: begin
        if (idx_q >= 3'd4) tx_byte = idx_q[0] ? hold_q[7:0] : hold_q[15:8];
      end
      default: ;
    endcase
  end

  // RDREG flags: T, LATE, STALE and WAITING from low to high. 12 to 31 are the shift unit's
  // (section 9.2), and 32 and up read 0.
  logic [15:0] rd_value;
  logic        rd_of_unit;

  assign rd_of_unit = reg_q[4] | reg_q[3];    // 12 to 31, with 8 to 11 taken below

  always_comb begin
    case (reg_q)
      5'd8:    rd_value = ex_d;
      5'd9:    rd_value = ex_s;
      5'd10:   rd_value = {7'd0, ex_pc};
      5'd11:   rd_value = {12'd0, ex_flags};
      default: rd_value = rd_of_unit ? ex_unit : ex_a;
    endcase
    if (!reg_ok_q) rd_value = 16'h0000;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      idx_q       <= 3'd0;
      cmd_q       <= C_NONE;
      n_q         <= 3'd0;
      arg_q       <= 8'h00;
      thread_q    <= 2'd0;
      reg_q       <= 5'd0;
      reg_ok_q    <= 1'b0;
      addr_q      <= 9'd0;
      hold_q      <= 16'h0000;
      mem_req_q   <= 1'b0;
      mem_write_q <= 1'b0;
      mem_read_q  <= 1'b0;
      rd_req_q    <= 1'b0;
      rd_cap_q    <= 1'b0;
      busy_q      <= 1'b0;
    end else begin
      // The bytes of a transaction.
      if (rx_idle) begin
        idx_q <= 3'd0;
        cmd_q <= C_NONE;
      end else if (rx_valid) begin
        idx_q <= (idx_q == 3'd7) ? 3'd6 : idx_q + 3'd1;
        arg_q <= rx_byte;
        if (first) begin
          cmd_q <= cmd_now;
          n_q   <= rx_byte[2:0];
        end
      end

      // The arguments.
      if (rx_valid) begin
        if (first) hold_q <= do_take ? (m_full ? m_rdata : 16'h0000) : 16'h0000;
        if ((cmd_q == C_SETPC || cmd_q == C_RDREG) && idx_q == 3'd1) thread_q <= rx_byte[1:0];
        if (addr_done) addr_q <= h_pc;
        if (word_done) hold_q <= {arg_q, rx_byte};
      end

      // Memory commands wait for a free memory cycle. A word is complete before it counts.
      if (addr_done && cmd_q == C_RDMEM) begin
        mem_req_q   <= 1'b1;
        mem_write_q <= 1'b0;
      end
      if (read_next) begin
        mem_req_q   <= 1'b1;
        mem_write_q <= 1'b0;
      end
      if (word_done) begin
        mem_req_q   <= 1'b1;
        mem_write_q <= 1'b1;
      end
      if (mem_fail) begin
        mem_req_q <= 1'b0;
        cmd_q     <= C_NONE;
      end
      if (mem_grant) begin
        mem_req_q <= 1'b0;
        addr_q    <= (addr_q + 9'd1) & ADDR_MASK;
      end
      mem_read_q <= mem_grant & ~mem_write_q;
      if (mem_read_q) hold_q <= mem_rdata;

      // RDREG waits for its thread's stage 3, and takes the result in stage 4.
      if (do_rdreg) begin
        reg_q    <= rx_byte[4:0];
        reg_ok_q <= rx_byte < 8'd32;
        rd_req_q <= ~rd_fail;
      end
      if (rd_here) rd_req_q <= 1'b0;
      rd_cap_q <= rd_here;
      if (rd_cap_q) hold_q <= rd_value;

      // BUSY.
      if (status_end) busy_q <= 1'b0;
      if ((do_setpc & run_post[thread_q]) | (do_step & run_post[rx_byte[1:0]]) | (do_rdreg & rd_fail) |
          mem_fail) begin
        busy_q <= 1'b1;
      end
    end
  end

endmodule
