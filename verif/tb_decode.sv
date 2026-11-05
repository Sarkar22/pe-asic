/*
 * Prints the decoder's answer for all 65,536 words, for a check against the instruction table.
 * Use: vvp ... +out=decode.txt. Each line is: word op illegal world ra rb, in hex.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`timescale 1ns / 1ps

module tb_decode;
  import pe_op_pkg::*;

  logic [15:0]        word;
  logic [OP_BITS-1:0] op;
  logic               illegal;
  logic               world;
  logic [2:0]         ra;
  logic [2:0]         rb;

  pe_decode dut (.word, .op, .illegal, .world, .ra, .rb);

  initial begin
    integer fd;
    string  path;
    if (!$value$plusargs("out=%s", path)) path = "decode.txt";
    fd = $fopen(path, "w");
    for (int i = 0; i < 65536; i = i + 1) begin
      word = 16'(i);
      #1;
      $fdisplay(fd, "%04x %02x %x %x %x %x", word, op, illegal, world, ra, rb);
    end
    $fclose(fd);
    $finish;
  end

endmodule
