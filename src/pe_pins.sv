/*
 * Output flops, their shadow, pin ownership, the output switches and the pad equations.
 * Instructions in stage 4 write the shadow. An event commits a thread's own pins at once.
 * A pin's output switch (SEL, section 5.9.2) puts it on shift output A or B of its owner's unit
 * instead of OUT; smap writes the switch's shadow SHSEL, and a commit copies it like OUT.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_pins
  import pe_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic [7:0]  ui_in,
    input  logic        miso,
    output logic [7:0]  uo_out,
    output logic [7:0]  uio_out,
    output logic [7:0]  uio_oe,
    // Events.
    input  logic [3:0]  commit,
    // The thread in stage 4.
    input  logic [1:0]  tid,
    input  logic [3:0]  pin,        // pin field of the instruction
    input  logic        put,        // shadow(pin, value)
    input  logic        value,
    input  logic        put8,       // put8(bank, data)
    input  logic        bank,
    input  logic [7:0]  data,
    input  logic        oe,         // shadow_oe(pin, value)
    input  logic        oe8,        // oe8(data)
    input  logic        cfg,        // pcfg of pin
    input  logic [1:0]  cfg_mode,
    input  logic        smap,       // shadow_sel(pin, smap_l)
    input  logic [1:0]  smap_l,
    output logic        shd_pin,    // shadow value of pin
    // The shift outputs A and B of each thread's unit.
    input  logic [3:0]  la,
    input  logic [3:0]  lb,
    // The host.
    input  logic        h_abort
);

  logic [15:0] out_q, out_d;
  logic [15:0] shd_q, shd_d;
  logic [7:0]  oe_q, oe_d;
  logic [7:0]  shoe_q, shoe_d;
  logic [7:0]  od_q, od_d;
  logic [7:0]  byp_q, byp_d;
  logic [7:0]  shbyp_q, shbyp_d;
  logic [31:0] own_q, own_d;        // two bits per pin
  logic [31:0] sel_q, sel_d;        // SEL, two bits per pin: 0 OUT, 1 shift output A, 2 shift output B
  logic [31:0] shsel_q, shsel_d;    // SHSEL, the same way
  logic [15:0] moves;               // pins an event commits in this cycle
  logic [31:0] moves2;              // the same, two bits per pin
  logic [15:0] mine;                // pins the thread in stage 4 owns
  logic [15:0] v;                   // V(p) of section 5.2: what each output pin shows

  localparam logic [7:0] LANES = 8'hF1;

  always_comb begin
    for (int p = 0; p < 16; p = p + 1) begin
      moves[p] = commit[own_q[2*p+:2]];
      mine[p]  = own_q[2*p+:2] == tid;
    end
  end

  for (genvar p = 0; p < 16; p = p + 1) begin : g_pin
    assign moves2[2*p+1:2*p] = {2{moves[p]}};
    assign v[p] = (sel_q[2*p+1:2*p] == 2'd1) ? la[own_q[2*p+1:2*p]] :
                  (sel_q[2*p+1:2*p] == 2'd2) ? lb[own_q[2*p+1:2*p]] : out_q[p];
  end

  assign shd_pin = shd_q[pin];

  always_comb begin
    // Step 2: commits. These read flops only, so an instruction below overrides them.
    out_d   = (out_q & ~moves) | (shd_q & moves);
    oe_d    = (oe_q & ~moves[15:8]) | (shoe_q & moves[15:8]);
    byp_d   = (byp_q & ~(moves[7:0] & LANES)) | (shbyp_q & moves[7:0] & LANES);
    shd_d   = shd_q;
    shoe_d  = shoe_q;
    od_d    = od_q;
    shbyp_d = shbyp_q;
    own_d   = own_q;
    sel_d   = (sel_q & ~moves2) | (shsel_q & moves2);
    shsel_d = shsel_q;

    // Step 3: the instruction.
    if (put && mine[pin]) shd_d[pin] = value;
    if (put8) begin
      for (int i = 0; i < 8; i = i + 1) begin
        if (mine[{bank, 3'(i)}]) shd_d[{bank, 3'(i)}] = data[i];
      end
    end
    if (oe && mine[pin] && !od_q[pin[2:0]]) shoe_d[pin[2:0]] = value;
    if (oe8) begin
      for (int i = 0; i < 8; i = i + 1) begin
        if (mine[8+i] && !od_q[i]) shoe_d[i] = data[i];
      end
    end
    if (smap && mine[pin]) shsel_d[{pin, 1'b0}+:2] = smap_l;
    if (cfg) begin
      // claim() clears the output switch, so the pin shows OUT at once.
      own_d[{pin, 1'b0}+:2]   = tid;
      sel_d[{pin, 1'b0}+:2]   = 2'd0;
      shsel_d[{pin, 1'b0}+:2] = 2'd0;
      case (cfg_mode)
        CFG_IN: begin
          od_d[pin[2:0]]   = 1'b0;
          shoe_d[pin[2:0]] = 1'b0;
        end
        CFG_PP: begin
          if (pin[3]) begin
            od_d[pin[2:0]]   = 1'b0;
            shoe_d[pin[2:0]] = 1'b1;
          end else begin
            shbyp_d[pin[2:0]] = 1'b0;
          end
        end
        CFG_OD: begin
          od_d[pin[2:0]]   = 1'b1;
          oe_d[pin[2:0]]   = 1'b0;
          shoe_d[pin[2:0]] = 1'b0;
          out_d[pin]       = 1'b1;
          shd_d[pin]       = 1'b1;
        end
        CFG_BYP: shbyp_d[pin[2:0]] = 1'b1;
        default: ;
      endcase
    end

    // After step 5: the host. ABORT takes the uio pins off the shift outputs (section 9.3).
    if (h_abort) begin
      oe_d        = 8'h00;
      shoe_d      = 8'h00;
      out_d[15:8] = out_d[15:8] | od_d;
      shd_d[15:8] = shd_d[15:8] | od_d;
      sel_d       = sel_d & 32'h0000_FFFF;
      shsel_d     = shsel_d & 32'h0000_FFFF;
    end

    // uo3 belongs to the host link and only the lanes have a bypass switch.
    out_d[3]     = 1'b0;
    shd_d[3]     = 1'b0;
    byp_d[3:1]   = 3'b000;
    shbyp_d[3:1] = 3'b000;
    sel_d        = sel_d & 32'hFFFF_FF3F;
    shsel_d      = shsel_d & 32'hFFFF_FF3F;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      out_q   <= 16'h0000;
      shd_q   <= 16'h0000;
      oe_q    <= 8'h00;
      shoe_q  <= 8'h00;
      od_q    <= 8'h00;
      byp_q   <= 8'h00;
      shbyp_q <= 8'h00;
      own_q   <= 32'h0000_0000;
      sel_q   <= 32'h0000_0000;
      shsel_q <= 32'h0000_0000;
    end else begin
      out_q   <= out_d;
      shd_q   <= shd_d;
      oe_q    <= oe_d;
      shoe_q  <= shoe_d;
      od_q    <= od_d;
      byp_q   <= byp_d;
      shbyp_q <= shbyp_d;
      own_q   <= own_d;
      sel_q   <= sel_d;
      shsel_q <= shsel_d;
    end
  end

  // Pad equations. A lane in bypass has no flop in its path.
  always_comb begin
    uo_out    = v[7:0];
    uo_out[3] = miso;
    if (byp_q[0]) uo_out[0] = ui_in[3];
    for (int i = 4; i < 8; i = i + 1) begin
      if (byp_q[i]) uo_out[i] = ui_in[i];
    end
    uio_out = v[15:8] & ~od_q;
    uio_oe  = (~v[15:8] & od_q) | (oe_q & ~od_q);
  end

endmodule
