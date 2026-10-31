/*
 * 8N1 UART transmitter.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module uart_tx
  import pe_pkg::*;
(
    input  logic  clk,
    input  logic  rst_n,
    input  byte_t data,
    input  logic  start,  // one-cycle pulse; ignored while busy
    output logic  tx,
    output logic  busy
);

  localparam baud_count_t BIT_PERIOD = baud_count_t'(CLKS_PER_BIT - 1);
  localparam bit_index_t  LAST_BIT   = bit_index_t'(DATA_BITS - 1);

  tx_regs_t q, d;
  logic     bit_done;

  assign bit_done = (q.baud_count == '0);
  assign tx       = q.line;
  assign busy     = (q.state != TX_IDLE);

  always_comb begin
    d = q;

    unique case (q.state)
      TX_IDLE: begin
        if (start) begin
          d.state      = TX_START;
          d.line       = 1'b0;  // start bit
          d.shifter    = data;
          d.baud_count = BIT_PERIOD;
        end
      end

      TX_START: begin
        if (bit_done) begin
          d.state      = TX_DATA;
          d.line       = q.shifter[0];  // data, LSB first
          d.shifter    = {1'b1, q.shifter[DATA_BITS-1:1]};
          d.bit_index  = '0;
          d.baud_count = BIT_PERIOD;
        end else begin
          d.baud_count = q.baud_count - 1'b1;
        end
      end

      TX_DATA: begin
        if (!bit_done) begin
          d.baud_count = q.baud_count - 1'b1;
        end else if (q.bit_index == LAST_BIT) begin
          d.state      = TX_STOP;
          d.line       = 1'b1;  // stop bit
          d.baud_count = BIT_PERIOD;
        end else begin
          d.line       = q.shifter[0];
          d.shifter    = {1'b1, q.shifter[DATA_BITS-1:1]};
          d.bit_index  = q.bit_index + 1'b1;
          d.baud_count = BIT_PERIOD;
        end
      end

      TX_STOP: begin
        if (bit_done) begin
          d.state = TX_IDLE;
        end else begin
          d.baud_count = q.baud_count - 1'b1;
        end
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) q <= TX_REGS_RESET;
    else        q <= d;
  end

endmodule
