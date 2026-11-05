/*
 * Output flops, their shadow, pin ownership and the pad equations.
 * Instructions in stage 4 write the shadow. An event commits a thread's own pins at once.
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
    output logic        shd_pin,    // shadow value of pin
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
  logic [15:0] moves;               // pins an event commits in this cycle
  logic [15:0] mine;                // pins the thread in stage 4 owns

  localparam logic [7:0] LANES = 8'hF1;

  always_comb begin
    for (int p = 0; p < 16; p = p + 1) begin
      moves[p] = commit[own_q[2*p+:2]];
      mine[p]  = own_q[2*p+:2] == tid;
    end
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
    if (cfg) begin
      own_d[{pin, 1'b0}+:2] = tid;
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

    // After step 5: the host.
    if (h_abort) begin
      oe_d        = 8'h00;
      shoe_d      = 8'h00;
      out_d[15:8] = out_d[15:8] | od_d;
      shd_d[15:8] = shd_d[15:8] | od_d;
    end

    // uo3 belongs to the host link and only the lanes have a bypass switch.
    out_d[3]     = 1'b0;
    shd_d[3]     = 1'b0;
    byp_d[3:1]   = 3'b000;
    shbyp_d[3:1] = 3'b000;
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
    end else begin
      out_q   <= out_d;
      shd_q   <= shd_d;
      oe_q    <= oe_d;
      shoe_q  <= shoe_d;
      od_q    <= od_d;
      byp_q   <= byp_d;
      shbyp_q <= shbyp_d;
      own_q   <= own_d;
    end
  end

  // Pad equations. A lane in bypass has no flop in its path.
  always_comb begin
    uo_out    = out_q[7:0];
    uo_out[3] = miso;
    if (byp_q[0]) uo_out[0] = ui_in[3];
    for (int i = 4; i < 8; i = i + 1) begin
      if (byp_q[i]) uo_out[i] = ui_in[i];
    end
    uio_out = out_q[15:8] & ~od_q;
    uio_oe  = (~out_q[15:8] & od_q) | (oe_q & ~od_q);
  end

endmodule
