// rk3n_f (src/pe_shift.sv's block for RKIND 3, PW 12, CW 13, read from that file) against rk3_code (rk3_code.sv, the
// code of 5.9.6 as written), every output, in every state that satisfies J' (5.9.6, the Period bullet), the states a
// reception of RKIND 3 reaches (proved inductive on the code in rk3_inv.sv, and on the whole unit in rk3_unit.sv),
// with the end of rx_bit free: bit_ends, vio2 and vb:
//   J1  p from 512 to 4095 with its window points; RXG 1 to 16; RXCAND only with RXOPEN; RXWIN any;
//   J2  RXPH at least 128 - p with the window closed, at least 256 - p with it open;
//   J3  RXPH below p + 128 with RGAIN 1 to 3, below 7p/4 - 128 with RGAIN 0;
//   J4  RXCE from 256 - p to -1 with RXCAND.
// Combinational: prove at depth 1.
module rk3_equiv (input logic clk, input logic run, op, cd, wn, racq, rdiff, l0, l1, l2, be, vio2, vb,
               input logic signed [17:0] ph, input logic [4:0] g, input logic signed [12:0] ce,
               input logic [11:0] p3, input logic [1:0] rgain);
  logic [11:0] pq3, ph3w;
  assign pq3  = p3 - {2'b00, p3[11:2]};
  assign ph3w = p3 - {1'b0, p3[11:1]};
  logic b_t, v_t, e_t, r_t, o_t, c_t, w_t, b_n, v_n, e_n, r_n, o_n, c_n, w_n;
  logic signed [20:0] x_t, ce_t;
  logic signed [17:0] x_n;
  logic [4:0] g_t, g_n;
  logic signed [12:0] ce_n;
  rk3_code T (.run(run), .ph(ph), .op(op), .cd(cd), .wn(wn), .g(g), .ce({{3{ce[12]}}, ce}), .p3({4'd0, p3}),
              .pq3({4'd0, pq3}), .ph3w({4'd0, ph3w}), .rgain(rgain), .racq(racq), .rdiff(rdiff), .l0(l0), .l1(l1),
              .l2(l2), .bit_ends(be), .vio2(vio2), .vb(vb), .bit3(b_t), .blv3(v_t), .endw3(e_t), .r3(x_t), .op3(o_t), .cd3(c_t),
              .wn3(w_t), .g3(g_t), .ce3(ce_t), .ran(r_t));
  rk3n_f #(.PW(12), .CW(13)) N (.run(run), .ph(ph), .op(op), .cd(cd), .wn(wn), .g(g), .ce(ce), .p3(p3),
              .pq3(pq3), .ph3w(ph3w), .rgain(rgain), .racq(racq), .rdiff(rdiff), .l0(l0), .l1(l1), .l2(l2),
              .bit_ends(be), .vio2(vio2), .vb(vb), .bit3(b_n), .blv3(v_n), .endw3(e_n), .r3(x_n), .op3(o_n), .cd3(c_n), .wn3(w_n),
              .g3(g_n), .ce3(ce_n), .ran(r_n));
  logic signed [19:0] xs, p, lo, hi;
  assign xs = {{2{ph[17]}}, ph};
  assign p  = $signed({8'd0, p3});
  assign lo = op ? (20'sd256 - p) : (20'sd128 - p);
  assign hi = (rgain != 2'd0) ? (p + 20'sd128) : (((p * 7) >>> 2) - 20'sd128);
  always_comb begin
    assume (p3 >= 12'd512);
    assume (g >= 5'd1 && g <= 5'd16);
    assume (!cd || op);
    assume (xs >= lo && xs < hi);
    if (cd) assume ($signed({{7{ce[12]}}, ce}) >= 20'sd256 - p && ce < 13'sd0);
    assert (b_n == b_t);
    assert (e_n == e_t);
    assert (r_n == r_t);
    if (b_t) assert (v_n == v_t);
    if (run) assert ($signed(x_n) == x_t);   // r3 is RXPH's next value only while the block runs (pe_shift's ph_d)
    assert (o_n == o_t);
    assert (c_n == c_t);
    assert (w_n == w_t);
    assert (g_n == g_t);
    assert ($signed(ce_n) == ce_t);
  end
endmodule
