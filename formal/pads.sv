/*
 * The pad equations of section 5.2, in every state of the chip and for every input:
 *
 *   V(p)       = LA[OWN[p]] if SEL[p] == 1 else LB[OWN[p]] if SEL[p] == 2 else OUT[p]
 *   uo_out[3]  = host data out (section 9)
 *   uo_out[i]  = BYP[i] ? ui_in[SRC(i)] : V(i)       for i != 3
 *   uio_out[i] = OD[i] ? 0        : V(8 + i)
 *   uio_oe[i]  = OD[i] ? !V(8 + i) : OE[i]
 *
 * with bypass lanes only on uo0 (SRC 3) and uo4 to uo7 (SRC i). From them, two properties stand alone:
 *   od_*   An open-drain pin can never drive high: with OD[i], uio_out[i] is 0, and the pin drives only
 *          while V(8 + i) is 0 (sections 5.2 and 5.3, "Open-drain safety is structural").
 *   byp_*  A lane in bypass shows its source input in the same cycle (sections 5.2 and 5.8). ui_in is a
 *          free input in every cycle, so the equality in the same cycle leaves no room for a flop.
 * V(p) is formed here from the flops, by the equation above, not taken from the design.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pads (
    input wire       clk,
    input wire       rst_n,
    input wire       ena,
    input wire [7:0] ui_in,
    input wire [7:0] uio_in
);

  wire [7:0] uo_out, uio_out, uio_oe;

  tt_um_sarkar22_pe dut (
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  // Probes: flops inside the chip, connected by the proof script (pads.sby).
  wire [15:0] out_q;   // OUT
  wire [7:0]  oe_q;    // OE
  wire [7:0]  od_q;    // OD
  wire [7:0]  byp_q;   // BYP (bits 0 and 4 to 7)
  wire [31:0] own_q;   // OWN, two bits per pin
  wire [31:0] sel_q;   // SEL, two bits per pin
  wire [3:0]  la, lb;  // LA[t], LB[t] of each thread's unit
  wire        miso;    // the host link's data out
  wire [3:0]  run;     // RUN of each thread, for the covers

  wire [15:0] v;

  genvar p;
  generate
    for (p = 0; p < 16; p = p + 1) begin : g_v
      wire [1:0] s = sel_q[2*p+1:2*p];
      wire [1:0] o = own_q[2*p+1:2*p];
      assign v[p] = (s == 2'd1) ? la[o] : (s == 2'd2) ? lb[o] : out_q[p];
    end
  endgenerate

  // Every trace starts in reset: the core's flops hold their reset values in step 0 (section 8).
  always @* if ($initstate) assume (!rst_n);

  reg past_ok = 1'b0;
  always @(posedge clk) past_ok <= 1'b1;

  // uo pins.
  always @* begin
    eq_uo3: assert (uo_out[3] == miso);
    eq_uo1: assert (uo_out[1] == v[1]);
    eq_uo2: assert (uo_out[2] == v[2]);
    eq_uo0: assert (uo_out[0] == (byp_q[0] ? ui_in[3] : v[0]));
    byp_uo0: assert (!byp_q[0] || uo_out[0] == ui_in[3]);
  end

  genvar i;
  generate
    for (i = 4; i < 8; i = i + 1) begin : g_lane
      always @* begin
        assert (uo_out[i] == (byp_q[i] ? ui_in[i] : v[i]));  // eq_uo
        assert (!byp_q[i] || uo_out[i] == ui_in[i]);  // byp_uo
      end
      // A lane in bypass while its owner runs, following an input that moves.
      always @(posedge clk) begin
        if (past_ok) cover (byp_q[i] && run[own_q[2*i+1-:2]] && uo_out[i] != $past(uo_out[i]));  // cov_byp_follows
      end
    end
  endgenerate

  always @(posedge clk) begin
    if (past_ok) cov_byp0_follows: cover (byp_q[0] && run[own_q[1:0]] && uo_out[0] != $past(uo_out[0]));
  end

  // uio pins.
  generate
    for (i = 0; i < 8; i = i + 1) begin : g_uio
      wire [1:0] s = sel_q[2*(8+i)+1-:2];
      always @* begin
        assert (uio_out[i] == (od_q[i] ? 1'b0 : v[8+i]));  // eq_out
        assert (uio_oe[i] == (od_q[i] ? !v[8+i] : oe_q[i]));  // eq_oe
        assert (!(od_q[i] && uio_out[i]));  // od_never_high
        assert (!(od_q[i] && uio_oe[i] && v[8+i]));  // od_low_only
        // An open-drain pin pulled low by OUT. An open-drain pin on a shift output, pulled low and released by
        // it, needs a longer trace: covers_deep.sv.
        cover (od_q[i] && s == 2'd0 && uio_oe[i]);  // cov_od_out_low
      end
    end
  endgenerate

endmodule

`default_nettype wire
