/*
 * The SRAM as an empty box, for reading the design without the macro.
 * SPDX-License-Identifier: Apache-2.0
 */

(* blackbox *)
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
endmodule
