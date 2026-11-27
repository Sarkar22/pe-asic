// rx_manch3 of section 5.9.6 (docs/ISA.md) on one cycle, transcribed from its code in wide arithmetic: two half
// steps one after the other, each in a reception that still runs, and the cycle's one pull at the end, none in a cycle
// with an edge in both halves. The reference for rk3n_f in every state, the ends included: a window that closes empty
// still opens (RXOPEN 1, RXCAND 0, RXPH less p) and its half step still adds 128, and a half step after the end doesn't
// run. p, RGAIN and RACQ are the ones the reception found (RXP3, RXRG, RXRA). rx_bit(b) ends the reception when
// bit_ends says so, or with SMODE 2 at the stuff position (vio2) when b equals vb, a stuff violation (5.9.7).
module rk3_code (
  input  logic               run,          // a reception runs and RXKIND is 3
  input  logic signed [17:0] ph,           // RXPH
  input  logic               op, cd, wn,   // RXOPEN, RXCAND, RXWIN
  input  logic [4:0]         g,            // RXG
  input  logic signed [15:0] ce,           // RXCE
  input  logic [15:0]        p3,           // p
  input  logic [15:0]        pq3, ph3w,    // the window points p - p/4 and p - p/2
  input  logic [1:0]         rgain,
  input  logic               racq, rdiff,
  input  logic               l0, l1, l2,   // RPIN in SYNC3, SYNC2F, SYNC2
  input  logic               bit_ends,     // a bit now ends the reception (rx_bit's rx_end), whatever its value
  input  logic               vio2, vb,     // SMODE 2's stuff position: a bit equal to vb (before RINV) ends it too
  output logic               bit3, blv3, endw3,
  output logic signed [20:0] r3,
  output logic               op3, cd3, wn3,
  output logic [4:0]         g3,
  output logic signed [20:0] ce3,
  output logic               ran
);
  logic signed [20:0] x, p, w, q, cc, d8, kb;
  logic               o, c, wv, e, u, v, rn, pulled, wide, nb, two, kc;
  logic [4:0]         gg;
  integer             h, k;

  always_comb begin
    x = ph; o = op; c = cd; wv = wn; gg = g; cc = ce;
    p = $signed({5'd0, p3});
    d8 = p >>> 3;
    kb = (p < 21'sd768) ? 21'sd128 : d8;
    wide = racq & (rgain != 2'd0) & (g < 5'd16);                 // RXG at the start of the cycle
    w = wide ? $signed({5'd0, ph3w}) : $signed({5'd0, pq3});      // p - w
    k = (rgain == 2'd0) ? 0 : (g <= 5'd3) ? 1 : (rgain == 2'd1) ? 2 : (rgain == 2'd2) ? 4 : 5;
    rn = run; pulled = 1'b0; q = 21'sd0; nb = 1'b0;
    two = (l0 != l1) && (l1 != l2);
    bit3 = 1'b0; blv3 = 1'b0; endw3 = 1'b0; kc = 1'b0;
    for (h = 0; h < 2; h = h + 1) begin
      u = (h == 0) ? l0 : l1;
      v = (h == 0) ? l1 : l2;
      e = (u != v);
      if (rn) begin
        if (x >= w) begin
          if (o) begin                       // rx_mend(), then rx_end()
            endw3 = 1'b1;
            rn = 1'b0;
          end
          o = 1'b1;
          c = 1'b0;
          x = x - p;
        end
        if (rn && e && wv && !c && !nb && (o || x >= w - d8)) begin   // RXPH >= p - w - (p >> 3)
          // the edge after a non-clock edge is the clock edge, before the window or in it
          if (!o) x = x - p;
          blv3 = rdiff ? 1'b1 : v;
          if (!pulled) begin
            pulled = 1'b1;
            q = x >>> k;
          end
          o = 1'b0; wv = 1'b0; nb = 1'b1;
          if (gg < 5'd16) gg = gg + 5'd1;
          bit3 = 1'b1;
          if (bit_ends || (vio2 && blv3 == vb)) rn = 1'b0;   // rx_bit(b) ends the reception
        end else if (rn && o) begin
          if (e && x < 0) begin
            if (c) wv = 1'b1;
            c = 1'b1;
            cc = x;
          end else if (e || (c && x >= d8)) begin
            kc = c && (x >= d8) && (cc >= -kb);                    // kb = 128 at RPER 2, else p/8
            blv3 = rdiff ? (wv | (c & e & !kc)) : (kc ? u : v);
            if (!pulled) begin
              pulled = 1'b1;
              q = (e && !kc) ? (x >>> k) : (cc >>> k);
            end
            o = 1'b0; c = 1'b0; wv = e & kc; nb = 1'b1;
            if (gg < 5'd16) gg = gg + 5'd1;
            bit3 = 1'b1;
            if (bit_ends || (vio2 && blv3 == vb)) rn = 1'b0;   // rx_bit(b) ends the reception
          end
        end else if (rn && e) begin
          wv = 1'b1;
          if (!pulled) begin
            pulled = 1'b1;
            q = (x - (p >>> 1)) >>> k;
          end
        end
        x = x + 21'sd128;
      end
    end
    x = x - (two ? 21'sd0 : q);
    r3 = x;
    op3 = o; cd3 = c; wn3 = wv; g3 = gg; ce3 = cc;
    ran = rn;
  end
endmodule
