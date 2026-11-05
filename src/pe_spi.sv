/*
 * Bit level of the host link: SPI mode 0, MSB first, oversampled through the input synchroniser.
 * A rising SCK at the pad in cycle p reaches this block in cycle p + 2. In that cycle the bit is
 * taken, and MISO moves at the end of it, so the pad changes in cycle p + 3.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_spi (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       cs_n,
    input  logic       sck,
    input  logic       mosi,
    input  logic [7:0] tx_byte,     // next reply byte, taken as rx_valid pulses
    output logic       rx_valid,    // the last bit of a byte arrives in this cycle
    output logic [7:0] rx_byte,
    output logic       rx_idle,     // CS is high
    output logic       miso
);

  logic       sck_q;
  logic [2:0] bit_q;
  logic [6:0] shift_q;
  logic [7:0] tx_q;
  logic       rise;

  assign rise     = sck & ~sck_q & ~cs_n;
  assign rx_idle  = cs_n;
  assign rx_valid = rise & (bit_q == 3'd7);
  assign rx_byte  = {shift_q, mosi};
  assign miso     = tx_q[7];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sck_q   <= 1'b0;
      bit_q   <= 3'd0;
      shift_q <= 7'd0;
      tx_q    <= 8'h00;
    end else begin
      sck_q <= sck;
      if (cs_n) begin
        bit_q   <= 3'd0;
        shift_q <= 7'd0;
        tx_q    <= 8'h00;
      end else if (rise) begin
        bit_q   <= bit_q + 3'd1;
        shift_q <= rx_byte[6:0];
        tx_q    <= rx_valid ? tx_byte : {tx_q[6:0], 1'b0};
      end
    end
  end

endmodule
