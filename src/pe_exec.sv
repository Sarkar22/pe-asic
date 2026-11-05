/*
 * Stage 4: what the thread that owns this cycle does in its slot. Combinational. The flops it
 * writes are in pe_thread, pe_regfile, pe_mailbox and pe_pins.
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module pe_exec
  import pe_pkg::*;
  import pe_op_pkg::*;
(
    // The word in stage 4 and the registers stage 3 read for it.
    input  logic               valid,
    input  logic [OP_BITS-1:0] op,
    input  logic               illegal,
    input  logic               world,
    input  logic [15:0]        ir,
    input  logic [2:0]         ra,
    input  logic [2:0]         rb,
    input  logic [15:0]        a,
    input  logic [15:0]        b,
    // The thread that owns stage 4.
    input  logic [1:0]         tid,
    input  logic [8:0]         pc,
    input  logic               t,
    input  logic [15:0]        d,
    input  logic [15:0]        s,
    input  logic               late,
    input  logic               stale,
    input  logic               run,
    input  logic               p,
    input  logic               waiting,
    input  logic               stopreq,
    input  logic               stepping,
    input  logic               mem,
    input  logic               mem_done,
    input  logic               mem_kind,
    input  logic [2:0]         mem_reg,
    // The timebase in this cycle, and in the next.
    input  logic [15:0]        tb,
    input  logic [15:0]        tb_next,
    // The mailbox word this instruction names.
    input  logic               m_full,
    input  logic [15:0]        m_data,
    output logic               m_put,
    output logic               m_take,
    // Pins.
    input  logic               shd_pin,
    output logic [3:0]         pin,
    output logic               pin_put,
    output logic               pin_value,
    output logic               pin_put8,
    output logic               pin_oe,
    output logic               pin_oe8,
    output logic               pin_cfg,
    // Register write.
    output logic               rf_we,
    output logic [2:0]         rf_reg,
    output logic [15:0]        rf_data,
    // Memory: taken in the cycle after this one.
    output logic               ls_start,
    output logic [8:0]         ls_addr,
    output logic [15:0]        ls_data,
    // Thread state.
    output logic               x_stop,
    output logic               x_step_end,
    output logic               x_wake,
    output logic               x_ill,
    output logic               x_pc_we,
    output logic [8:0]         x_pc,
    output logic               x_t_we,
    output logic               x_t,
    output logic               x_d_we,
    output logic [15:0]        x_d,
    output logic               x_late_set,
    output logic               x_late_clr,
    output logic               x_arm,
    output logic               x_kind,
    output logic               x_due_we,
    output logic               x_due,
    output logic               x_wait,
    output logic [7:0]         x_mask,
    output logic [7:0]         x_pat,
    output logic               x_bank,
    output logic               x_ne,
    output logic               x_timed,
    output logic               x_inbox,
    output logic               x_mem_kind,
    output logic               x_mem_end
);

  // The branches of slot() in section 3.6, in order.
  logic finish, stop_req, have_word, wake, normal, word_runs, p_word;
  logic x_go;       // the row runs: not stalled and not illegal

  assign finish    = run & mem & mem_done;
  assign stop_req  = run & ~finish & stopreq;
  assign have_word = run & ~finish & ~stopreq & valid;
  assign wake      = have_word & waiting & ~p;
  assign normal    = have_word & ~waiting;
  assign word_runs = normal | (wake & ~stepping);
  assign p_word    = normal & p;

  assign x_ill = word_runs & illegal;
  assign x_go  = word_runs & ~illegal & ~(world & p_word);

  // Operands.
  logic [15:0] imm;
  logic [15:0] y;
  logic [16:0] sum;
  logic        sub_op;
  logic        dec_op;
  logic        eq;
  logic        bit_a;
  logic [8:0]  pc_inc;

  assign imm    = {8'h00, ir[7:0]};
  assign y      = (ir[15:12] == 4'b0111) ? b : imm;
  assign sub_op = (op == OP_SUB) | (op == OP_SUBI) | (op == OP_LT);
  assign dec_op = op == OP_DJNZ;
  assign sum    = {1'b0, a} + {1'b0, dec_op ? 16'hFFFF : (sub_op ? ~y : y)} + 17'(sub_op);
  assign eq     = a == y;
  assign bit_a  = a[ir[8:5]];
  assign pc_inc = (pc + 9'd1) & ADDR_MASK;

  // schedule(step). The deadline test is one three-operand expression, so it becomes a
  // carry-save adder and not two adders in a row.
  logic [15:0] step;
  logic [15:0] d_sched;
  logic [15:0] delta;
  logic        sched;

  assign d_sched = d + step;
  assign delta   = d + step - tb;

  // Shifts: T goes in, the outgoing bit comes back to T.
  logic        t_in;
  logic        shift_out;
  logic [15:0] shift_res;

  assign t_in = (ir[7:4] != 4'd0) ? s[ir[7:4]] : t;

  always_comb begin
    case (ir[11:10])
      2'b00:   shift_out = a[7];
      2'b01:   shift_out = a[15];
      default: shift_out = a[0];
    endcase
    case (ir[11:10])
      2'b00:   shift_res = {a[15:8], a[6:0], t_in};
      2'b01:   shift_res = {a[14:0], t_in};
      2'b10:   shift_res = {a[15:8], t_in, a[7:1]};
      default: shift_res = {t_in, a[15:1]};
    endcase
  end

  // CRC steps run in order, so with Rd = Rs the xor sees the shifted value.
  logic        crc_fb;
  logic [15:0] crc_shift;
  logic [15:0] crc_res;

  assign crc_fb    = t ^ ((op == OP_CRCR) ? a[0] : a[15]);
  assign crc_shift = (op == OP_CRCR) ? {1'b0, a[15:1]} : {a[14:0], 1'b0};
  assign crc_res   = crc_fb ? (crc_shift ^ ((ra == rb) ? crc_shift : b)) : crc_shift;

  logic [7:0] one_hot;

  always_comb begin
    one_hot = 8'h00;
    one_hot[ir[9:7]] = 1'b1;
  end

  // What the row does, if it runs.
  logic        row_hold;        // no retire: PC stays, or a wait moves it
  logic        row_wait;
  logic        row_halt;
  logic        row_ls;
  logic        row_jump;
  logic [8:0]  row_target;
  logic        row_rf;
  logic [15:0] row_data;
  logic [2:0]  row_reg;
  logic        row_t_we;
  logic        row_t;
  logic        row_d_we;
  logic [15:0] row_d;
  logic        row_arm;
  logic        row_put;
  logic        row_value;
  logic        row_put8;
  logic        row_oe;
  logic        row_oe8;
  logic        row_cfg;
  logic        row_m_put;
  logic        row_m_take;
  logic        row_due_we;
  logic        row_tlate;

  always_comb begin
    row_hold   = 1'b0;
    row_wait   = 1'b0;
    row_halt   = 1'b0;
    row_ls     = 1'b0;
    row_jump   = 1'b0;
    row_target = ir[8:0] & ADDR_MASK;
    row_rf     = 1'b0;
    row_data   = sum[15:0];
    row_reg    = ra;
    row_t_we   = 1'b0;
    row_t      = 1'b0;
    row_d_we   = 1'b0;
    row_d      = tb_next;
    row_arm    = 1'b0;
    row_put    = 1'b0;
    row_value  = 1'b0;
    sched      = 1'b0;
    step       = {4'h0, ir[11:0]};
    row_put8   = 1'b0;
    row_oe     = 1'b0;
    row_oe8    = 1'b0;
    row_cfg    = 1'b0;
    row_m_put  = 1'b0;
    row_m_take = 1'b0;
    row_due_we = 1'b0;
    row_tlate  = 1'b0;

    case (op)
      OP_HALT: row_halt = 1'b1;
      OP_NOW: begin
        row_d_we   = 1'b1;
        row_arm    = 1'b1;
        row_due_we = 1'b1;
      end
      OP_CLRT: row_t_we = 1'b1;
      OP_SETT: begin
        row_t_we = 1'b1;
        row_t    = 1'b1;
      end
      OP_NOTT: begin
        row_t_we = 1'b1;
        row_t    = ~t;
      end
      OP_TLATE: begin
        row_t_we  = 1'b1;
        row_t     = late;
        row_tlate = 1'b1;
      end
      OP_JR: begin
        row_jump   = 1'b1;
        row_target = a[8:0] & ADDR_MASK;
      end
      OP_SWAP: begin
        row_rf   = 1'b1;
        row_data = {a[7:0], a[15:8]};
      end
      OP_NOT: begin
        row_rf   = 1'b1;
        row_data = ~a;
      end
      OP_RDS: begin
        row_rf   = 1'b1;
        row_data = s;
      end
      OP_RDD: begin
        row_rf   = 1'b1;
        row_data = d;
      end
      OP_WRD: begin
        row_d_we = 1'b1;
        row_d    = a;
      end
      OP_RDTB: begin
        row_rf   = 1'b1;
        row_data = tb;
      end
      OP_PUT8: row_put8 = 1'b1;
      OP_OE8:  row_oe8  = 1'b1;
      OP_TID: begin
        row_rf   = 1'b1;
        row_data = {14'd0, tid};
      end
      OP_GOTO: row_jump = 1'b1;
      OP_JT:   row_jump = t;
      OP_JF:   row_jump = ~t;
      OP_CALL: begin
        row_jump = 1'b1;
        row_rf   = 1'b1;
        row_reg  = 3'd7;
        row_data = {7'd0, pc_inc};
      end
      OP_DJNZ: begin
        row_rf   = 1'b1;
        row_jump = sum[15:0] != 16'h0000;
      end
      OP_LDI: begin
        row_rf   = 1'b1;
        row_data = imm;
      end
      OP_LUI: begin
        row_rf   = 1'b1;
        row_data = {ir[7:0], a[7:0]};
      end
      OP_ADDI, OP_SUBI, OP_ADD, OP_SUB: row_rf = 1'b1;
      OP_CMPI, OP_CMP: begin
        row_t_we = 1'b1;
        row_t    = eq;
      end
      OP_TSTI: begin
        row_t_we = 1'b1;
        row_t    = (a & imm) != 16'h0000;
      end
      OP_ANDI, OP_AND: begin
        row_rf   = 1'b1;
        row_data = a & y;
      end
      OP_ORI, OP_OR: begin
        row_rf   = 1'b1;
        row_data = a | y;
      end
      OP_CP: begin
        row_rf   = 1'b1;
        row_data = b;
      end
      OP_XOR: begin
        row_rf   = 1'b1;
        row_data = a ^ b;
      end
      OP_XORT: begin
        row_rf   = t;
        row_data = a ^ b;
      end
      OP_CRCR, OP_CRCL: begin
        row_rf   = 1'b1;
        row_data = crc_res;
      end
      OP_LT: begin
        row_t_we = 1'b1;
        row_t    = ~sum[16];
      end
      OP_BT: begin
        row_t_we = 1'b1;
        row_t    = bit_a;
      end
      OP_BN: begin
        row_t_we = 1'b1;
        row_t    = ~bit_a;
      end
      OP_BX: begin
        row_t_we = 1'b1;
        row_t    = t ^ bit_a;
      end
      OP_BA: begin
        row_t_we = 1'b1;
        row_t    = t & bit_a;
      end
      OP_BO: begin
        row_t_we = 1'b1;
        row_t    = t | bit_a;
      end
      OP_BS: begin
        row_rf            = 1'b1;
        row_data          = a;
        row_data[ir[8:5]] = t;
      end
      OP_PUT_0: row_put = 1'b1;
      OP_PUT_1: begin
        row_put   = 1'b1;
        row_value = 1'b1;
      end
      OP_PUT_T: begin
        row_put   = 1'b1;
        row_value = t;
      end
      OP_PUT_NT: begin
        row_put   = 1'b1;
        row_value = ~t;
      end
      OP_PUT_XT: begin
        row_put   = 1'b1;
        row_value = shd_pin ^ t;
      end
      OP_PUT_XNT: begin
        row_put   = 1'b1;
        row_value = ~(shd_pin ^ t);
      end
      OP_OE: begin
        row_oe    = 1'b1;
        row_value = ir[3];
      end
      OP_PCFG_IN, OP_PCFG_PP, OP_PCFG_OD, OP_PCFG_BYP: row_cfg = 1'b1;
      OP_GET: begin
        row_t_we = 1'b1;
        row_t    = s[ir[9:6]];
      end
      OP_GETN: begin
        row_t_we = 1'b1;
        row_t    = ~s[ir[9:6]];
      end
      OP_XGET: begin
        row_t_we = 1'b1;
        row_t    = t ^ s[ir[9:6]];
      end
      OP_AGET: begin
        row_t_we = 1'b1;
        row_t    = t & s[ir[9:6]];
      end
      OP_RECV: begin
        row_hold   = ~m_full;
        row_rf     = m_full;
        row_data   = m_data;
        row_m_take = m_full;
      end
      OP_SEND: begin
        row_hold  = m_full;
        row_m_put = ~m_full;
      end
      OP_TRYRECV: begin
        row_t_we   = 1'b1;
        row_t      = m_full;
        row_rf     = m_full;
        row_data   = m_data;
        row_m_take = m_full;
      end
      OP_TRYSEND: begin
        row_t_we  = 1'b1;
        row_t     = ~m_full;
        row_m_put = ~m_full;
      end
      OP_LD, OP_ST: begin
        row_hold = 1'b1;
        row_ls   = 1'b1;
      end
      OP_LDP, OP_STP: begin
        row_hold = 1'b1;
        row_ls   = 1'b1;
        row_rf   = 1'b1;
        row_reg  = rb;
        row_data = b + 16'd1;
      end
      OP_UNTIL, OP_UNTILM: begin
        row_hold   = 1'b1;
        row_wait   = 1'b1;
        row_due_we = 1'b1;
      end
      OP_UNTIL_RT, OP_UNTILM_RT: begin
        row_hold   = 1'b1;
        row_wait   = 1'b1;
        row_due_we = 1'b1;
        sched      = 1'b1;
        step       = b;
      end
      OP_TICK: begin
        row_arm    = 1'b1;
        row_due_we = 1'b1;
        sched      = 1'b1;
      end
      OP_EDGE_I, OP_EDGE_RR: begin
        row_arm    = 1'b1;
        row_due_we = 1'b1;
        sched      = 1'b1;
        step       = (op == OP_EDGE_I) ? {11'd0, ir[4:0]} : a;
        row_put    = 1'b1;
        case (ir[7:6])
          2'd0:    row_value = 1'b0;
          2'd1:    row_value = 1'b1;
          2'd2:    row_value = t;
          default: row_value = ~t;
        endcase
      end
      OP_SHL8, OP_SHL16, OP_SHR8, OP_SHR16: begin
        row_rf    = 1'b1;
        row_data  = shift_res;
        row_t_we  = 1'b1;
        row_t     = shift_out;
        row_put   = 1'b1;
        row_value = shift_out;
      end
      default: ;
    endcase

    if (sched) begin
      row_d_we = 1'b1;
      row_d    = stale ? tb_next : d_sched;
    end
  end

  // shadow() ignores pin 3, which is how "no pin" works.
  logic is_shift;

  assign is_shift  = ir[15:12] == 4'b1111;
  assign pin       = is_shift ? ir[3:0] : ir[11:8];
  assign pin_put   = x_go & row_put & (pin != 4'd3);
  assign pin_value = row_value;
  assign pin_put8  = x_go & row_put8;
  assign pin_oe    = x_go & row_oe;
  assign pin_oe8   = x_go & row_oe8;
  assign pin_cfg   = x_go & row_cfg;
  assign m_put     = x_go & row_m_put;
  assign m_take    = x_go & row_m_take;

  // The retire of a row, and of a wait in the slot after its event.
  logic row_retires;

  assign row_retires = x_go & ~row_hold & ~row_halt;

  assign x_stop     = stop_req | x_ill | (x_go & row_halt);
  assign x_step_end = stepping & (finish | wake | row_retires);
  assign x_wake     = wake;

  assign x_pc_we = finish | (x_go & (row_retires | row_wait | row_halt));
  assign x_pc    = (x_go & row_jump) ? row_target : pc_inc;

  assign x_t_we = x_go & row_t_we;
  assign x_t    = row_t;

  assign x_d_we = x_go & row_d_we;
  assign x_d    = row_d;

  assign x_late_set = x_go & sched & stale;
  assign x_late_clr = x_go & row_tlate;
  assign x_arm      = x_go & (row_arm | row_wait);
  assign x_kind     = row_wait ? KIND_WAIT : KIND_TICK;
  assign x_due_we   = x_go & row_due_we;
  assign x_due      = sched & ~stale & ((delta == 16'h0000) | delta[15]);

  // wait(): until names a pin, untilm takes the mask and pattern from Rr.
  logic is_until;

  assign is_until = ~ir[11];
  assign x_wait   = x_go & row_wait;
  assign x_mask   = is_until ? one_hot : a[15:8];
  assign x_pat    = is_until ? (ir[6] ? one_hot : 8'h00)
                  : (ir[9:8] == 2'd2) ? (ir[10] ? s[15:8] : s[7:0]) : a[7:0];
  assign x_bank   = ir[10];
  assign x_ne     = ~is_until & (ir[9:8] != 2'd0);
  assign x_timed  = sched;
  assign x_inbox  = ir[0];

  assign ls_start   = x_go & row_ls;
  assign ls_addr    = b[8:0] & ADDR_MASK;
  assign ls_data    = a;
  assign x_mem_kind = ir[10] ? MEM_STORE : MEM_LOAD;
  assign x_mem_end  = finish;

  assign rf_we   = (finish & (mem_kind == MEM_LOAD)) | (x_go & row_rf);
  assign rf_reg  = finish ? mem_reg : row_reg;
  assign rf_data = finish ? ir : row_data;

endmodule
