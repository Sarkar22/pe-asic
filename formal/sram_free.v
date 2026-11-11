/*
 * The SRAM macro for the proofs: the same name and ports as the PDK's 512x16 SRAM, with read data that is
 * a free input in every cycle. A proof that holds with it holds for every memory content, so for every
 * program and every value a load can return. The macro has no reset (section 8).
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module RM_IHPSG13_1P_512x16_c2_bm_bist (
    input  wire         A_CLK,
    input  wire         A_MEN,
    input  wire         A_WEN,
    input  wire         A_REN,
    input  wire  [8:0]  A_ADDR,
    input  wire  [15:0] A_DIN,
    input  wire         A_DLY,
    output wire  [15:0] A_DOUT,
    input  wire  [15:0] A_BM,
    input  wire         A_BIST_CLK,
    input  wire         A_BIST_EN,
    input  wire         A_BIST_MEN,
    input  wire         A_BIST_WEN,
    input  wire         A_BIST_REN,
    input  wire  [8:0]  A_BIST_ADDR,
    input  wire  [15:0] A_BIST_DIN,
    input  wire  [15:0] A_BIST_BM
);

  (* anyseq *) wire [15:0] free_dout;

  assign A_DOUT = free_dout;

endmodule

`default_nettype wire
