/*
 * Design core: send uio_in as one 8N1 byte on each rising edge of ui_in[0].
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_core
  import pe_pkg::*;
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
    if (!rst_n) rst_sync <= '0;
    else        rst_sync <= {rst_sync[0], 1'b1};
  end
  assign rst_n_sync = rst_sync[1];

  // 2-FF synchronizer and rising-edge detect on the send input.
  logic [2:0] send_sync;
  logic       send_pulse;

  always_ff @(posedge clk or negedge rst_n_sync) begin
    if (!rst_n_sync) send_sync <= '0;
    else             send_sync <= {send_sync[1:0], ui_in[0]};
  end
  assign send_pulse = send_sync[1] & ~send_sync[2];

  logic tx;
  logic busy;

  uart_tx uart (
      .clk  (clk),
      .rst_n(rst_n_sync),
      .data (uio_in),
      .start(send_pulse),
      .tx   (tx),
      .busy (busy)
  );

  uo_pins_t uo_pins;

  always_comb begin
    uo_pins         = '0;
    uo_pins.uart_tx = tx;
    uo_pins.busy    = busy;
  end

  assign uo_out  = uo_pins;
  assign uio_out = '0;
  assign uio_oe  = '0;

  // List all unused inputs to prevent warnings
  logic _unused;
  assign _unused = &{ena, ui_in[7:1], 1'b0};

endmodule
