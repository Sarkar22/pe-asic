/*
 * Stands in for pe_spi in the cycle-exact harness. The wrapper writes these flops directly, so a
 * host command can land in the very cycle a byte-level simulator puts it.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_spi (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       cs_n,
    input  logic       sck,
    input  logic       mosi,
    input  logic [7:0] tx_byte,
    output logic       rx_valid,
    output logic [7:0] rx_byte,
    output logic       rx_idle,
    output logic       miso
);

  logic       stub_valid;
  logic [7:0] stub_byte;

  assign rx_valid = stub_valid;
  assign rx_byte  = stub_byte;
  assign rx_idle  = 1'b0;
  assign miso     = 1'b0;

  logic _unused;
  assign _unused = &{clk, rst_n, cs_n, sck, mosi, tx_byte, 1'b0};

endmodule
