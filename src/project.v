/*
 * Toolchain smoke test: send uio_in as one 8N1 byte on each rising edge of ui_in[0].
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_sarkar22_pe (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // Reset: asserted asynchronously, released synchronously.
  reg [1:0] rst_sync;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rst_sync <= 2'b00;
    else        rst_sync <= {rst_sync[0], 1'b1};
  end
  wire rst_n_sync = rst_sync[1];

  // 2-FF synchronizer and rising-edge detect on the send input.
  reg [2:0] send_sync;
  always @(posedge clk or negedge rst_n_sync) begin
    if (!rst_n_sync) send_sync <= 3'b000;
    else             send_sync <= {send_sync[1:0], ui_in[0]};
  end
  wire send_pulse = send_sync[1] & ~send_sync[2];

  wire tx;
  wire busy;

  uart_tx uart (
      .clk  (clk),
      .rst_n(rst_n_sync),
      .data (uio_in),
      .start(send_pulse),
      .tx   (tx),
      .busy (busy)
  );

  // uo_out[4] is the demo board's RP2350 UART1 RX pin.
  assign uo_out  = {3'b000, tx, 3'b000, busy};
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, ui_in[7:1], 1'b0};

endmodule
