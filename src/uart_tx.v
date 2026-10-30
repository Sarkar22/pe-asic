/*
 * 8N1 UART transmitter.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module uart_tx #(
    parameter CLKS_PER_BIT = 434  // 50 MHz / 115200 baud
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       start,  // one-cycle pulse; ignored while busy
    output reg        tx,
    output wire       busy
);

  reg [15:0] clk_count;
  reg [3:0]  bits_left;  // start + 8 data + stop still to shift out
  reg [8:0]  shifter;

  assign busy = (bits_left != 4'd0);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx        <= 1'b1;
      clk_count <= 16'd0;
      bits_left <= 4'd0;
      shifter   <= 9'h1ff;
    end else if (!busy) begin
      if (start) begin
        tx        <= 1'b0;  // start bit
        shifter   <= {1'b1, data};  // stop bit, then data LSB first
        bits_left <= 4'd10;
        clk_count <= CLKS_PER_BIT - 1;
      end
    end else if (clk_count != 16'd0) begin
      clk_count <= clk_count - 16'd1;
    end else begin
      bits_left <= bits_left - 4'd1;
      if (bits_left != 4'd1) begin
        tx        <= shifter[0];
        shifter   <= {1'b1, shifter[8:1]};
        clk_count <= CLKS_PER_BIT - 1;
      end
    end
  end

endmodule
