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

// Level latch, open while GATE is high: Q follows D, and holds from GATE's fall.
module sg13cmos5l_dlhq_1 (
    input  wire D,
    input  wire GATE,
    output reg  Q
);
  always_latch begin
    if (GATE) Q = D;
  end
endmodule

// Level latch, open while GATE_N is low, cleared while RESET_B is low.
module sg13cmos5l_dllrq_1 (
    input  wire D,
    input  wire GATE_N,
    input  wire RESET_B,
    output reg  Q
);
  always_latch begin
    if (!RESET_B) Q = 1'b0;
    else if (!GATE_N) Q = D;
  end
endmodule

`default_nettype wire
