/*
 * Types and constants shared by the design.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

package pe_pkg;

  // 50 MHz / 115200 baud.
  localparam int unsigned CLKS_PER_BIT = 434;
  localparam int unsigned DATA_BITS    = 8;

  typedef logic [DATA_BITS-1:0]         byte_t;
  typedef logic [15:0]                  baud_count_t;
  typedef logic [$clog2(DATA_BITS)-1:0] bit_index_t;

  // Position within one 8N1 frame.
  typedef enum logic [1:0] {
    TX_IDLE,
    TX_START,
    TX_DATA,
    TX_STOP
  } tx_state_t;

  // Everything the transmitter keeps between clock edges.
  typedef struct packed {
    tx_state_t   state;
    baud_count_t baud_count;
    bit_index_t  bit_index;
    byte_t       shifter;
    logic        line;
  } tx_regs_t;

  // Field order: state, baud_count, bit_index, shifter, line.
  localparam tx_regs_t TX_REGS_RESET = tx_regs_t'({TX_IDLE, 16'd0, 3'd0, 8'hff, 1'b1});

  // Dedicated outputs. uart_tx is the demo board's RP2350 UART1 RX pin.
  typedef struct packed {
    logic [2:0] unused_hi;
    logic       uart_tx;
    logic [2:0] unused_lo;
    logic       busy;
  } uo_pins_t;

endpackage
