/*
 * Behavioural stand-ins for the PDK standard cells that src/ instantiates by name, for RTL simulation.
 * Same names and ports. The gate-level test uses the PDK's own models instead.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module sg13cmos5l_buf_2 (
    input  wire A,
    output wire X
);
  assign X = A;
endmodule

`default_nettype wire
