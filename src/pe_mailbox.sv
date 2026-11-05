/*
 * Eight mailbox words with full flags, shared by the threads and the host. When both touch one
 * word in a cycle the thread goes first and the host works on what the thread left.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_mailbox (
    input  logic        clk,
    input  logic        rst_n,
    // The thread in stage 4.
    input  logic [2:0]  idx,
    input  logic        put,        // M[idx] := wdata, FULL[idx] := 1
    input  logic        take,       // FULL[idx] := 0
    input  logic [15:0] wdata,
    output logic [15:0] rdata,
    output logic        rfull,
    output logic [7:0]  full,
    // The host. h_full and h_rdata show the word after the thread's operation.
    input  logic [2:0]  h_idx,
    input  logic        h_post,
    input  logic        h_take,
    input  logic [15:0] h_wdata,
    output logic        h_full,
    output logic [15:0] h_rdata
);

  logic [15:0] m_q [0:7];
  logic [7:0]  full_q;
  logic [7:0]  full_mid;
  logic [7:0]  full_d;

  assign full  = full_q;
  assign rdata = m_q[idx];
  assign rfull = full_q[idx];

  always_comb begin
    full_mid = full_q;
    if (put) full_mid[idx] = 1'b1;
    if (take) full_mid[idx] = 1'b0;
  end

  assign h_full  = full_mid[h_idx];
  assign h_rdata = (put && idx == h_idx) ? wdata : m_q[h_idx];

  always_comb begin
    full_d = full_mid;
    if (h_post) full_d[h_idx] = 1'b1;
    if (h_take) full_d[h_idx] = 1'b0;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 8; i = i + 1) m_q[i] <= 16'h0000;
      full_q <= 8'h00;
    end else begin
      full_q <= full_d;
      if (put) m_q[idx] <= wdata;
      if (h_post && !h_full) m_q[h_idx] <= h_wdata;
    end
  end

endmodule
