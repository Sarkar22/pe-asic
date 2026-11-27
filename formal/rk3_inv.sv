// J' (rk3_equiv.sv; 5.9.6, the Period bullet) holds in every state that a reception of RKIND 3 reaches, on the code as
// written (rk3_code.sv), with p from 512 to 4095, RGAIN, RACQ and RDIFF as the reception found them (RXP3, RXRG,
// RXRA), and any input in every cycle: the pin's levels at both edges, and whether a bit ends the reception (bit_ends,
// or SMODE 2's violation, vio2 and vb). A reception that
// ends starts again at rx_begin's state (RXPH 128 or 256, the window closed, RXG 1, RXWIN 0; RXCE as it was).
// Proved by k-induction.
module rk3_inv (input logic clk, input logic l0, l1, l2, be, vio2, vb, h, input logic [11:0] p_in,
                 input logic [1:0] rg_in,
                 input logic racq_in, rdiff_in);
  logic [11:0] p3;
  logic [1:0]  rgain;
  logic        racq, rdiff, first;
  initial first = 1'b1;
  always_ff @(posedge clk) begin
    first <= 1'b0;
    if (first) begin
      p3 <= p_in; rgain <= rg_in; racq <= racq_in; rdiff <= rdiff_in;
    end
  end
  logic signed [20:0] x, ce;
  logic               op, cd, wn;
  logic [4:0]         g;
  logic signed [20:0] x_n, ce_n;
  logic               op_n, cd_n, wn_n, b_n, v_n, e_n, r_n;
  logic [4:0]         g_n;
  logic [11:0]        pq3, ph3w;
  assign pq3  = p3 - {2'b00, p3[11:2]};
  assign ph3w = p3 - {1'b0, p3[11:1]};
  rk3_code T (.run(1'b1), .ph(x[17:0]), .op(op), .cd(cd), .wn(wn), .g(g), .ce(ce[15:0]), .p3({4'd0, p3}),
              .pq3({4'd0, pq3}), .ph3w({4'd0, ph3w}), .rgain(rgain), .racq(racq), .rdiff(rdiff), .l0(l0), .l1(l1),
              .l2(l2), .bit_ends(be), .vio2(vio2), .vb(vb), .bit3(b_n), .blv3(v_n), .endw3(e_n), .r3(x_n), .op3(op_n), .cd3(cd_n),
              .wn3(wn_n), .g3(g_n), .ce3(ce_n), .ran(r_n));
  always_ff @(posedge clk) begin
    if (first || !r_n) begin               // rx_begin(): the first reception, or the next after an end
      x  <= h ? 21'sd128 : 21'sd256;
      op <= 1'b0; cd <= 1'b0; wn <= 1'b0; g <= 5'd1;
      if (first) ce <= 21'sd0;
    end else begin
      x <= x_n; op <= op_n; cd <= cd_n; wn <= wn_n; g <= g_n; ce <= ce_n;
    end
  end
  logic signed [20:0] p, lo, hi;
  assign p  = $signed({9'd0, p3});
  assign lo = op ? (21'sd256 - p) : (21'sd128 - p);
  assign hi = (rgain != 2'd0) ? (p + 21'sd128) : (((p * 7) >>> 2) - 21'sd128);
  always_comb begin
    if (!first) begin
      assume (p3 >= 12'd512);
      assert (g >= 5'd1 && g <= 5'd16);
      assert (!cd || op);
      assert (x >= lo && x < hi);
      if (cd) assert (ce >= 21'sd256 - p && ce < 21'sd0);
      cover (!op && x < 21'sd0);
      cover (op && x < -(p >>> 1));
    end
  end
endmodule
