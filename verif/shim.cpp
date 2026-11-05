// C interface to the Verilator model, for rtl_chip.py.
//
// The clock edge that ends a cycle is held back until something needs it. A host call made after
// cycle(c) therefore still lands inside cycle c, as it does in a byte-level simulator.
//
// SPDX-License-Identifier: Apache-2.0

#include "Vpe_core.h"
#include "Vpe_core___024root.h"

#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#define R(x) pe_core__DOT__##x
#define TH(i, x) pe_core__DOT__gen_thread__BRA__##i##__KET____DOT__thread__DOT__##x

namespace {

struct Field {
  void *p;
  int bytes;
};

struct LogEntry {
  uint64_t cycle;
  uint32_t tid, pc, word;
};

// One context for every chip, never freed: Verilator's scope cleanup goes through a thread-local
// context pointer. Without threads(1) it would also start a thread pool sized to the machine.
VerilatedContext *context() {
  static VerilatedContext *ctx = [] {
    auto *c = new VerilatedContext;
    c->threads(1);
    Verilated::threadContextp(c);
    return c;
  }();
  return ctx;
}

struct Chip {
  Vpe_core top;
  Vpe_core___024root *r;
  bool pending = false;  // the edge ending the current cycle has not been applied
  bool settled = false;  // comb logic is up to date, with clk low
  bool used = false;     // a host command already took this edge
  bool log_on = false;
  uint64_t c = 0;
  std::vector<Field> fields;
  std::vector<LogEntry> log;
  uint64_t late_events[4] = {0, 0, 0, 0};
  int64_t late_at[4] = {-1, -1, -1, -1};
  int64_t host_mem_at = -1;  // cycle of the last memory access granted to the host
  uint32_t host_mem_addr = 0, host_mem_data = 0, host_mem_write = 0;
  explicit Chip(const char *name) : top(context(), name), r(top.rootp) {}
};

std::map<std::string, int> g_names;

template <typename T>
void add(Chip *c, const char *name, T *p) {
  g_names.emplace(name, (int)c->fields.size());
  c->fields.push_back(Field{(void *)p, (int)sizeof(T)});
}

template <typename T>
void add4(Chip *c, const char *name, T *p0, T *p1, T *p2, T *p3) {
  add(c, name, p0);
  add(c, (std::string(name) + "#1").c_str(), p1);
  add(c, (std::string(name) + "#2").c_str(), p2);
  add(c, (std::string(name) + "#3").c_str(), p3);
}

#define ADD4(name) add4(c, #name, &r->TH(0, name), &r->TH(1, name), &r->TH(2, name), &r->TH(3, name))

void build_fields(Chip *c) {
  auto r = c->r;
  ADD4(pc_q);
  ADD4(t_q);
  ADD4(d_q);
  ADD4(s_q);
  ADD4(late_q);
  ADD4(stale_q);
  ADD4(ill_q);
  ADD4(run_q);
  ADD4(p_q);
  ADD4(kind_q);
  ADD4(due_q);
  ADD4(waiting_q);
  ADD4(mask_q);
  ADD4(pat_q);
  ADD4(bank_q);
  ADD4(ne_q);
  ADD4(timed_q);
  ADD4(inbox_q);
  ADD4(stopreq_q);
  ADD4(stepping_q);
  ADD4(mem_q);
  ADD4(mem_done_q);
  ADD4(mem_kind_q);
  ADD4(mem_reg_q);
  ADD4(run_post);
  add(c, "tb_q", &r->R(tb_q));
  add(c, "sync1_q", &r->R(sync1_q));
  add(c, "sync2_q", &r->R(sync2_q));
  add(c, "valid2_q", &r->R(valid2_q));
  add(c, "valid3_q", &r->R(valid3_q));
  add(c, "valid4_q", &r->R(valid4_q));
  add(c, "ir3_q", &r->R(ir3_q));
  add(c, "ir4_q", &r->R(ir4_q));
  add(c, "ls_addr_q", &r->R(ls_addr_q));
  add(c, "ls_data_q", &r->R(ls_data_q));
  add(c, "dr_r", &r->R(mem__DOT__sram__DOT__dr_r));
  add(c, "out_q", &r->R(pins__DOT__out_q));
  add(c, "shd_q", &r->R(pins__DOT__shd_q));
  add(c, "oe_q", &r->R(pins__DOT__oe_q));
  add(c, "shoe_q", &r->R(pins__DOT__shoe_q));
  add(c, "od_q", &r->R(pins__DOT__od_q));
  add(c, "byp_q", &r->R(pins__DOT__byp_q));
  add(c, "shbyp_q", &r->R(pins__DOT__shbyp_q));
  add(c, "own_q", &r->R(pins__DOT__own_q));
  add(c, "full_q", &r->R(mailbox__DOT__full_q));
  add(c, "idx_q", &r->R(host__DOT__idx_q));
  add(c, "cmd_q", &r->R(host__DOT__cmd_q));
  add(c, "n_q", &r->R(host__DOT__n_q));
  add(c, "arg_q", &r->R(host__DOT__arg_q));
  add(c, "thread_q", &r->R(host__DOT__thread_q));
  add(c, "addr_q", &r->R(host__DOT__addr_q));
  add(c, "hold_q", &r->R(host__DOT__hold_q));
  add(c, "mem_req_q", &r->R(host__DOT__mem_req_q));
  add(c, "mem_write_q", &r->R(host__DOT__mem_write_q));
  add(c, "mem_read_q", &r->R(host__DOT__mem_read_q));
  add(c, "rd_req_q", &r->R(host__DOT__rd_req_q));
  add(c, "rd_cap_q", &r->R(host__DOT__rd_cap_q));
  add(c, "busy_q", &r->R(host__DOT__busy_q));
  add(c, "tx_byte", &r->R(host__DOT__tx_byte));
  for (int i = 0; i < 32; i++) {
    add(c, ("reg" + std::to_string(i)).c_str(), &r->R(regfile__DOT__regs_q)[i]);
  }
  for (int i = 0; i < 8; i++) {
    add(c, ("m" + std::to_string(i)).c_str(), &r->R(mailbox__DOT__m_q)[i]);
  }
}

inline uint32_t read_field(const Field &f) {
  switch (f.bytes) {
    case 1: return *(uint8_t *)f.p;
    case 2: return *(uint16_t *)f.p;
    default: return *(uint32_t *)f.p;
  }
}

inline void write_field(const Field &f, uint32_t v) {
  switch (f.bytes) {
    case 1: *(uint8_t *)f.p = (uint8_t)v; break;
    case 2: *(uint16_t *)f.p = (uint16_t)v; break;
    default: *(uint32_t *)f.p = v; break;
  }
}

inline void settle(Chip *c) {
  if (!c->settled) {
    c->top.clk = 0;
    c->top.eval();
    c->settled = true;
  }
}

inline uint32_t th_late_mask(Chip *c) {
  auto r = c->r;
  return (uint32_t)r->TH(0, late_q) | ((uint32_t)r->TH(1, late_q) << 1) |
         ((uint32_t)r->TH(2, late_q) << 2) | ((uint32_t)r->TH(3, late_q) << 3);
}

// Applies the edge that ends the current cycle.
void flush(Chip *c) {
  if (!c->pending) return;
  auto r = c->r;
  settle(c);
  uint32_t th = r->R(tb_q) & 3;
  if (c->log_on && r->R(exec__DOT__x_go)) {
    uint32_t pc = th == 0 ? r->TH(0, pc_q) : th == 1 ? r->TH(1, pc_q) : th == 2 ? r->TH(2, pc_q) : r->TH(3, pc_q);
    c->log.push_back(LogEntry{c->c - 1, th, pc, r->R(ir4_q)});
  }
  if (r->R(exec__DOT__x_late_set)) c->late_events[th]++;
  if (r->TH(0, tick_fire) && r->TH(0, due_q)) c->late_events[0]++;
  if (r->TH(1, tick_fire) && r->TH(1, due_q)) c->late_events[1]++;
  if (r->TH(2, tick_fire) && r->TH(2, due_q)) c->late_events[2]++;
  if (r->TH(3, tick_fire) && r->TH(3, due_q)) c->late_events[3]++;
  if (r->R(host_grant)) {
    c->host_mem_at = (int64_t)c->c - 1;
    c->host_mem_addr = r->R(sram_addr);
    c->host_mem_data = r->R(sram_din);
    c->host_mem_write = r->R(sram_wen);
  }
  c->top.clk = 1;
  c->top.eval();
  c->top.clk = 0;
  c->settled = false;
  c->pending = false;
#ifndef PE_PINS
  r->R(host__DOT__spi__DOT__stub_valid) = 0;
#endif
  uint32_t late = th_late_mask(c);
  if (late) {
    for (int i = 0; i < 4; i++) {
      if (((late >> i) & 1) && c->late_at[i] < 0) c->late_at[i] = (int64_t)c->c - 1;
    }
  }
}

}  // namespace

extern "C" {

Chip *pe_new(void) {
  static uint64_t serial = 0;
  Chip *c = new Chip(("pe" + std::to_string(serial++)).c_str());
  auto &t = c->top;
  auto r = c->r;
#ifndef PE_PINS
  r->R(host__DOT__spi__DOT__stub_valid) = 0;
  r->R(host__DOT__spi__DOT__stub_byte) = 0;
#endif
  t.ena = 1;
  t.ui_in = 0;
  t.uio_in = 0;
  t.clk = 0;
  t.rst_n = 0;
  t.eval();
  for (int i = 0; i < 4; i++) {
    t.clk = 1;
    t.eval();
    t.clk = 0;
    t.eval();
  }
  t.rst_n = 1;
  t.eval();
  for (int i = 0; i < 2; i++) {
    t.clk = 1;
    t.eval();
    t.clk = 0;
    t.eval();
  }
  build_fields(c);
  c->settled = true;
  return c;
}

void pe_free(Chip *c) { delete c; }

int pe_field(Chip *, const char *name) {
  auto it = g_names.find(name);
  return it == g_names.end() ? -1 : it->second;
}

void pe_cycle(Chip *c, unsigned ui, unsigned uio) {
  flush(c);
  c->top.ui_in = (uint8_t)ui;
  c->top.uio_in = (uint8_t)uio;
  c->pending = true;
  c->used = false;
  c->c++;
}

// Pads for the cycle about to run. Bypass lanes see ui, so outputs settle with it.
unsigned pe_pads(Chip *c, unsigned ui) {
  flush(c);
  c->top.ui_in = (uint8_t)ui;
  c->top.clk = 0;
  c->top.eval();
  c->settled = true;
  return (unsigned)c->top.uo_out | ((unsigned)c->top.uio_out << 8) | ((unsigned)c->top.uio_oe << 16);
}

void pe_flush(Chip *c) { flush(c); }

uint64_t pe_c(Chip *c) { return c->c; }

unsigned pe_get(Chip *c, int f) {
  flush(c);
  return read_field(c->fields[f]);
}

void pe_set(Chip *c, int f, unsigned v) {
  flush(c);
  write_field(c->fields[f], v);
  c->settled = false;
}

// Writes a flop before the held edge is applied. For setting up a command, with pe_host.
void pe_poke(Chip *c, int f, unsigned v) {
  write_field(c->fields[f], v);
  c->settled = false;
}

// A comb signal, as the cycle under way has it. Does not apply the edge.
unsigned pe_peek(Chip *c, int f) {
  settle(c);
  return read_field(c->fields[f]);
}

// All flops of interest, in table order, without leaving C once per field.
void pe_snapshot(Chip *c, uint32_t *out) {
  flush(c);
  size_t n = c->fields.size();
  for (size_t i = 0; i < n; i++) out[i] = read_field(c->fields[i]);
}

int pe_field_count(Chip *c) { return (int)c->fields.size(); }

unsigned pe_mem_get(Chip *c, unsigned a) {
  flush(c);
  return c->r->R(mem__DOT__sram__DOT__memory)[a & 511];
}

void pe_mem_set(Chip *c, unsigned a, unsigned v) {
  flush(c);
  c->r->R(mem__DOT__sram__DOT__memory)[a & 511] = (uint16_t)v;
}

void pe_mem_fill(Chip *c, const uint16_t *words, unsigned n) {
  flush(c);
  for (unsigned i = 0; i < n && i < 512; i++) c->r->R(mem__DOT__sram__DOT__memory)[i] = words[i];
}

void pe_mem_dump(Chip *c, uint16_t *words, unsigned n) {
  flush(c);
  for (unsigned i = 0; i < n && i < 512; i++) words[i] = c->r->R(mem__DOT__sram__DOT__memory)[i];
}

// Feeds the last byte of a command into the real command block, which sees the earlier bytes as
// already received. Returns the reply byte the block offers, or -1 if the edge has gone.
int pe_host(Chip *c, unsigned cmd, unsigned idx, unsigned arg, unsigned n, unsigned thread, unsigned byte) {
#ifdef PE_PINS
  (void)c; (void)cmd; (void)idx; (void)arg; (void)n; (void)thread; (void)byte;
  return -1;
#else
  if (!c->pending || c->used) return -1;
  auto r = c->r;
  c->used = true;
  r->R(host__DOT__cmd_q) = (uint8_t)cmd;
  r->R(host__DOT__idx_q) = (uint8_t)idx;
  r->R(host__DOT__arg_q) = (uint8_t)arg;
  r->R(host__DOT__n_q) = (uint8_t)n;
  r->R(host__DOT__thread_q) = (uint8_t)thread;
  r->R(host__DOT__spi__DOT__stub_valid) = 1;
  r->R(host__DOT__spi__DOT__stub_byte) = (uint8_t)byte;
  c->top.clk = 0;
  c->top.eval();
  c->settled = true;
  return r->R(host__DOT__tx_byte);
#endif
}

// Drives rst_n between cycles, for tests of a reset in mid-run. The cycle count does not move.
void pe_rst(Chip *c, unsigned v) {
  flush(c);
  c->top.rst_n = (uint8_t)v;
  c->top.clk = 0;
  c->top.eval();
  c->settled = true;
}

// One raw clock edge with no cycle count, for the edges while rst_n is low.
void pe_raw_edge(Chip *c) {
  flush(c);
  c->top.clk = 1;
  c->top.eval();
  c->top.clk = 0;
  c->top.eval();
  c->settled = true;
}

void pe_log_enable(Chip *c, int on) {
  flush(c);
  c->log_on = on != 0;
  c->log.clear();
}

// Moves up to max entries out, as four words each. Returns how many.
int pe_log_drain(Chip *c, uint64_t *out, int max) {
  flush(c);
  int n = (int)c->log.size() < max ? (int)c->log.size() : max;
  for (int i = 0; i < n; i++) {
    out[4 * i] = c->log[i].cycle;
    out[4 * i + 1] = c->log[i].tid;
    out[4 * i + 2] = c->log[i].pc;
    out[4 * i + 3] = c->log[i].word;
  }
  c->log.erase(c->log.begin(), c->log.begin() + n);
  return n;
}

uint64_t pe_late_events(Chip *c, int t) {
  flush(c);
  return c->late_events[t];
}

int64_t pe_late_at(Chip *c, int t) {
  flush(c);
  return c->late_at[t];
}

// The last memory access the host was granted: {cycle, addr, data, write}.
void pe_host_mem(Chip *c, int64_t *out) {
  flush(c);
  out[0] = c->host_mem_at;
  out[1] = c->host_mem_addr;
  out[2] = c->host_mem_data;
  out[3] = c->host_mem_write;
}

}  // extern "C"
