# ISA v0.1

Draft, 2026-11-23. This instruction set can change until the hardware freezes.

**Status of the evidence.** Where this document says something ran, it ran on Python simulators: two hand-written, and one compiled directly from the instruction table and code blocks of this file. They agree on every cycle compared. Sections 7 and 11.2 list what ran. Treat anything they don't list as not run. The cycle figures in section 9 come from the RTL in `src/`, which has been simulated, synthesized and laid out, but not built. No real hardware is behind this document. One of the simulators is in `sim/`. The programs of section 11.2 are in `programs/`, with benches that run each of them on that simulator: `python3 programs/check.py`. The RTL is in `src/`, with its tests in `test/`.

## 1. Overview

A thread changes the outside world only at an **event**. It prepares pin values in a shadow copy, then names the cycle, or the input pattern, at which they should take effect. In that one cycle the hardware does three things at once:

- commits all of that thread's prepared pin changes;
- snapshots all inputs;
- records the cycle number.

Between events a thread is ordinary software working on registers and a one-bit wire called T. A protocol then reads like a small synchronous circuit: an event is its clock edge, and the code between events is its next-state logic.

A chip has four threads in fixed rotation, one memory for code and data, 16 output pins, 16 input pins, a 16-bit free-running timebase and an SPI link to a host. Each thread also has a shift unit, which sends and receives bit streams in hardware, with line coding, a CRC and bit stuffing (section 5.9).

## 2. Programmer's model

**Threads.** Four hardware threads share one datapath in fixed rotation. Thread *i* owns every cycle where `cycle mod 4 = i`, so each thread runs one instruction every 4 cycles, 12.5 million per second at 50 MHz. A slot is never lent to another thread. A build with fewer threads keeps the four-slot rotation and leaves the missing threads' slots idle, so firmware timing doesn't change.

Nothing divides instruction issue: every thread issues one instruction every 4 cycles, whatever its shift unit does. TB and D only schedule events, and a shift unit's counters only time its own symbols and samples (section 5.9).

**Per-thread state that firmware or the host can see:**

| State | Bits | Purpose |
|---|---|---|
| PC | 9 | Program counter. Always less than DEPTH (section 4) |
| R0..R7 | 16 each | General registers. `call` writes the return address to R7. Only R0..R3 can be shifted |
| T | 1 | The bit wire. A single bit moving between a pin, a register and a CRC passes through it. `rds`, `put8` and `oe8` move whole words and don't touch it |
| D | 16 | Reference cycle: the last event, or the next one if an event is pending |
| S | 16 | Snapshot of the inputs at the last event. `S[7:0]` is `ui`, `S[15:8]` is `uio` |
| LATE | 1 | Sticky. Set when a deadline was missed. Cleared by `tlate` and by a host RUN that starts the thread |
| STALE | 1 | D is more than half the timebase range behind TB (section 3.5) |
| ILL | 1 | Sticky. The thread stopped on an illegal word. Cleared by a host RUN that starts the thread |
| RUN | 1 | The thread is running |
| WAITING | 1 | Host only, through RDREG. A wait instruction has armed and has not retired. On a stopped thread, the wait was abandoned before its event fired |
| CFG[0..15], CRC, RXPE | 16 each, 32, 3 | The shift unit's configuration slots, CRC register and end flags (section 5.9.2) |

**Per-thread sequencing state.** Neither firmware nor the host can read these, but they are needed to define behaviour. RTL may hold them in any form that behaves the same.

| State | Purpose |
|---|---|
| P | An event is pending |
| KIND | The pending event is a `tick` or a `wait` |
| DUE | The deadline had already passed when it was armed |
| MASK, PAT, BANK, NE, TIMED, INBOX | The armed wait |
| MEM, MEM_DONE, MEM_KIND, MEM_REG, MEM_ADDR, MEM_DATA | A `ld` or `st` between its two slots |
| STOPREQ, STEPPING | Host stop and single-step requests |
| IR, IV | The fetched word and whether it is valid |
| HIT, D_WRITTEN | Inputs to the STALE update. These last one cycle, so they aren't really state |
| The shift unit's halves | Section 5.9.2. Its CMT, a commit in this cycle, lasts one cycle like HIT |

**Global state:**

| State | Bits | Purpose |
|---|---|---|
| TB | 16 | Free-running timebase, one count per clock, never stopped or divided |
| M0..M7 | 16 each | Mailbox words, reachable by every thread and by the host |
| FULL0..FULL7 | 1 each | Mailbox full flags |
| SYNC1, SYNC2 | 16 each | Two-flop input synchroniser |
| SYNC3 | 16 | SYNC2 one cycle older, so that an edge on any pin is `SYNC2 ^ SYNC3` (section 5.9) |
| SYNC1F, SYNC2F | 16 each | The falling-edge samples: each pad's level at the falling edge in the middle of a cycle, through two flops, so SYNC2F is the pad half a cycle after SYNC3's sample and half a cycle before SYNC2's (section 3.1) |
| OUT, SHD | 16 each | Pin output flops and their shadow. Bit *p* belongs to output pin *p* (section 5.1) |
| OE, SHOE, OD | 8 each | `uio` output enable, its shadow, and open-drain mode |
| BYP, SHBYP | 5 each | Bypass switch per lane and its shadow |
| SEL, SHSEL | 2 x 16 each | Output switch of each output pin and its shadow: OUT, or shift output A or B of the pin's owner (section 5.9.2) |
| OWN | 2 x 16 | Owning thread of each output pin |
| BUSY | 1 | Sticky. A host command could not run (section 9) |

Arithmetic is modulo 2^16 on registers and D. T is always 0 or 1.

## 3. Cycle model and events

### 3.1 Cycles and slots

`c` counts clock cycles from 0, the first cycle after reset ends. `TB = c mod 65536`.

- The **execute cycle** of thread *i* is any cycle with `c mod 4 = i`.
- The **memory cycle** of thread *i* is any cycle with `c mod 4 = (i + 1) mod 4`, one cycle after its execute cycle.

An **alignment** is the value of *h* mod 4 for the RUN that starts a thread, where *h* is the cycle of that RUN (section 9.3). Each thread has 4, so `16 alignments` means 4 threads x 4 alignments.

All state is flops that update on the rising edge ending a cycle. A value written in cycle *c* is visible from cycle *c + 1*.

**Half cycles.** The falling clock edge splits each cycle into two halves: the first from the rising edge that starts the cycle, the second from the falling edge to the rising edge that ends it. Only the pads use the falling edge, and no state changes there:

- **Inputs.** Each input pad is sampled twice in cycle *c*: at its falling edge, into SYNC1F, and at the rising edge that ends it, into SYNC1. Both samples move on through flops at rising edges, so in any cycle SYNC3, SYNC2F and SYNC2 hold one pad's level at three times, each half a cycle after the one before. Only the Manchester decoder with RKIND 3 reads SYNC2F (section 5.9.6).
- **Outputs.** Each output pad has a level in each half (section 5.2). The two differ only on a pin switched to a shift output that its send half moved at the falling edge (THALF, section 5.9.5). Everything else moves at rising edges only.
- **The chip's own edges.** A pad that the chip moves at a clock edge is sampled at that edge with its old level, because the pad's delays come after the edge. So a change written in cycle *w* is in SYNC2 at *w* + 3 whether it moved the pad at the rising or the falling edge of *w* + 1, and after a move at the falling edge SYNC2F still shows the old level at *w* + 3.
- **One level per cycle.** A test, simulator or board that gives an input one level per cycle gives it for the whole cycle: the falling-edge sample of cycle *c* is the level of cycle *c*, so every input edge falls in the first half. One that reads an output once per cycle reads the first half, and sees a move at a falling edge at the next rising edge, half a cycle late. Without THALF and RKIND 3, both halves of every output agree, SYNC2F is read by nothing, and everything behaves as with the rising edge alone.
- **The duty cycle.** The half cycles are only as exact as the clock's duty cycle: a falling edge *d* ns late moves every falling-edge sample and every falling-edge move by *d* ns (section 7.3).

### 3.2 Order inside one cycle

A simulator does these steps for cycle *c*, in this order. Every read of a flop sees its value from the start of the cycle, with one exception: `stop()` reads P as step 2 left it. Inside one step, a block also sees what it wrote itself a line earlier.

1. **Pads.** Compute the pad outputs from flop state (section 5.2), for both halves of the cycle, then take the pad inputs for cycle *c*: the levels at its falling edge and at the rising edge that ends it (section 3.1). A lane in bypass has no flop in its path, so it depends on `ui_in` too. A simulator either takes inputs first or shows `ui_in` of cycle *c - 1*, as section 10 allows.
2. **Events.** For every thread, set HIT to whether TB = D xor 0x8000. Then, for every thread with P = 1, run `events()` (section 3.4).

   2b. **Shift units.** Every thread runs `tx_step()`, then `rx_step()` (section 5.9).

3. **Execute.** The thread owning cycle *c* runs `slot(P_start)` (section 3.6), where P_start is its P flag from before step 2.
4. **Memory.** The thread whose memory cycle is *c* runs `memory()` (section 4.2).
5. **Update.** Every thread runs `update_stale()` (section 3.5), and CMT := 0. Then `SYNC3 := SYNC2`, `SYNC2 := SYNC1`, `SYNC1 := {uio_in, ui_in}`, `SYNC2F := SYNC1F`, `SYNC1F :=` the same pads' levels at the falling edge of cycle *c*, and *c* advances.

A host command takes effect after step 5 of its cycle. A memory access and the read of RDREG come later (section 9.1).

Step 2b reads CMT, which step 2 sets in the same cycle. Like HIT, CMT lasts one cycle, so it isn't really state. Otherwise step 2b reads only values from the start of the cycle. That includes configuration: the unit sees an `scfg` from the cycle after it, even one in the same cycle as a start, a symbol or a sample.

**The two halves of a unit share no flop in step 2b.** Each half writes only its own state, and reads only that, CMT, the configuration and the synchroniser. The CRC register belongs to the half that CTX names in that cycle: the send half reads and writes it only with CTX 1, and the receive half only with CTX 0 (section 5.9.7). So the order of `tx_step` and `rx_step` can't change a result, and an implementation may run them at once. Inside one half the code runs in order, so a line reads what an earlier line of the same step wrote.

**Step 3 reads the unit's flops as they were at the start of the cycle, before step 2b.** That is the first rule of section 3.2, and it matters here because step 2b, unlike the steps before it, writes flops that step 3 reads in the same cycle: TXFULL (`tx_load`), RXFULL, RXBUF, RXBN, RXBL and RXBE (`rx_deliver`), CRC (`crc_init`, `crc_bit`, the append and the byte end of `rx_eop`), and every flop of the status word, TXUR, RXOVR, RXSEEN, RXRUN, RXEP, LA and LB among them. So a `txw` in the cycle the unit takes the buffer still blocks, an `rxw` in the cycle a word arrives still blocks, an `sstat` in the cycle a flag sets reads it as 0 and leaves it set, and an `srd` in the cycle a bit enters the CRC register reads the register as it was before that bit. A simulator that runs the steps one after another on live state must keep these flops' values from before step 2b for step 3, or run step 3 on a copy of the start-of-cycle state and apply its writes after step 2b's.

Apart from that one read, and from step 3's reads of the shift-unit flops that step 2b writes, which see the start of the cycle like every read (section 5.9), no step reads a flop that an earlier step wrote in the same cycle, so the steps behave like simultaneous flop updates. World instructions and blocked waits stall on P_start, which is what keeps this true. `stop()` is the exception because a stop in the event's own cycle must see the wait as finished. In hardware, the P it reads is the P flop at the start of the cycle and not firing in this cycle. The overlaps are write conflicts, and the instruction wins: `pcfg p, OD` on a pin that its old owner commits in that cycle; any `pcfg`, which clears the pin's output switch, on a pin whose old owner commits that switch in that cycle; `sstop` against a shift unit half that is moving; and `sstat` clearing a sticky flag that was already set and sets again.

Of these, the `sstop` and `sstat` cases are the only ones where steps 2b and 3 write one flop in a cycle. `txw` and `rxw` never do, because each writes only a buffer state that step 2b leaves alone in that cycle: a `txw` runs only on a buffer that was empty at the start of the cycle, which `tx_load` doesn't touch, and an `rxw` only on a full one, into which `rx_deliver` can't write; RXPE, which `rxw` writes, is written by nothing else. `srd` writes no unit state. `sgo` never meets a start, because a world instruction can't run in the cycle its own event fires.

### 3.3 Events

Each thread can have one event pending.

| Instruction | The event fires | Commits shadow | Snapshots S | D afterwards | T afterwards |
|---|---|---|---|---|---|
| `tick`, `edge` | When TB = D, or next cycle if DUE | Yes | Yes | The scheduled value | Unchanged |
| `now` | On the next cycle | Yes | Yes | That cycle | Unchanged |
| wait, input matches | On the match cycle | Yes | Yes | The match cycle | 1 |
| wait, inbox fills | On the cycle M[TID] is seen full | **No** | Yes | That cycle | 0 |
| wait, timeout | When TB = D, or next cycle if DUE | **No** | Yes | What arming set: old D + Rt, or that cycle if D was stale. The firing doesn't change it | 0 |

`tick`, `edge` and `now` don't block. They arm the event and the thread moves on, so it can compute the next bit while the current one waits to fire. A wait (`until`, `untilm`) blocks, and the next instruction runs in the thread's first slot after the event.

If a wait could end in more than one way in the same cycle, an input match wins, then the inbox, then the timeout. An inbox exit leaves the word full, for `recv` to take.

A timeout and an inbox exit leave the prepared pins uncommitted, because a pre-armed reaction should happen only if the awaited input really occurred. They stay prepared, though. The thread's next event commits them, even an unrelated one, so firmware that gives up on a reaction must write the old values back into the shadow first.

### 3.4 Arming and firing

The code blocks in this document are the normative definition. They use the notation of section 6.1, and blocks of this form are machine-read. *a* is the current cycle, and `next` is `(PC + 1) % DEPTH`. A name in capitals is state of the thread running the code, or global state if section 2 lists it as global.

The functions the table in section 6.3 calls:

```
schedule(step):
    if STALE:
        D := a + 1
        LATE := 1
    else:
        D := D + step
    delta := (D - a) % 65536
    DUE := 1 if delta == 0 or delta >= 0x8000 else 0

arm(step):
    schedule(step)
    P := 1
    KIND := tick

now():
    D := a + 1
    P := 1
    KIND := tick
    DUE := 0

wait(mask, pat, bank, ne, rt, inbox):
    MASK := mask & 255
    PAT := pat & 255
    BANK := bank
    NE := ne
    INBOX := inbox
    TIMED := rt is not None
    DUE := 0
    if TIMED:
        schedule(rt)
    P := 1
    KIND := wait
    WAITING := 1
    PC := next
```

The event logic, for every thread with P = 1 in every cycle:

```
events():
    if KIND == tick:
        if DUE or D == TB:
            if DUE:
                LATE := 1
            commit()
            S := SYNC2
            P := 0
    elif KIND == wait:
        byte := (SYNC2 >> (8 * BANK)) & 255
        match := ((byte ^ PAT) & MASK) == 0
        if NE:
            match := not match
        if match:
            commit()
            S := SYNC2
            D := TB
            T := 1
            P := 0
        elif INBOX and FULL[TID]:
            S := SYNC2
            D := TB
            T := 0
            P := 0
        elif TIMED and (DUE or D == TB):
            S := SYNC2
            T := 0
            P := 0
```

The comparison runs every clock, not just on the thread's slot, and the instruction's slot has no effect on when an on-time event lands. `commit()` is in section 5.4.

### 3.5 Late and stale

**Late.** A deadline must be 1 to 32767 cycles after the cycle that arms it: 655 µs at 50 MHz. Otherwise the arming sets DUE, the event fires on the next cycle, and LATE sets. D keeps the scheduled value, so one overrun doesn't shift the rest of a bit stream. Later ticks stay late until the thread catches up.

A wait whose timeout has already passed times out on the next cycle unless the input matches, and doesn't set LATE. So `until p, l, Rt` with Rt = 0 is a legal way to sample now and test.

**Stale.** D is compared on a 16-bit ring, so a D left idle for half the ring could look like a future deadline. D_WRITTEN says D was written since the last update, by an instruction, an event or the host. In step 5 of every cycle:

```
update_stale():
    if D_WRITTEN:
        STALE := 0
    elif HIT:
        STALE := 1
    D_WRITTEN := 0
```

An event instruction that finds STALE set can't trust D, so `schedule` replaces it with the next cycle and sets LATE. So:

- `tick` or `edge` from a stale D fires on the next cycle with LATE set. Its step is discarded, and D is that cycle.
- A timed wait from a stale D times out on the next cycle unless the input matches, with LATE set.
- A wait with no timeout doesn't use D, so it doesn't set LATE. A match or inbox exit writes D, which clears STALE.

Firmware should run `now` after any unbounded wait such as `recv`. If it doesn't, the result is a LATE flag, not a silent stall of up to 655 µs.

Longer delays use a loop of ticks, which adds no error because D is pure arithmetic.

### 3.6 Executing a slot

In step 3, the thread owning the cycle does this.

```
slot(P_start):
    if not RUN:
        pass
    elif MEM and MEM_DONE:
        finish_mem()
    elif STOPREQ:
        stop()
    elif not IV:
        pass
    elif WAITING:
        if not P_start:
            WAITING := 0
            retire()
            if RUN:
                execute(IR, 0)
    else:
        execute(IR, P_start)

execute(word, P_start):
    if illegal(word):
        ILL := 1
        stop()
    elif world(word) and P_start:
        pass
    else:
        run_row(word)

stop():
    RUN := 0
    if not P:
        WAITING := 0
    P := 0
    STOPREQ := 0
    STEPPING := 0

retire():
    if STEPPING:
        RUN := 0
        STEPPING := 0
        if STOPREQ:
            P := 0
        STOPREQ := 0
```

- `illegal(word)` is section 10. `world(word)` is the W column of the table.
- `run_row(word)` runs the row's semantics. If they fail a `need`, the thread does nothing more, so the same word runs next slot. If they call `load` or `store`, PC stays put, and the instruction retires later. If they call `wait`, PC moves on when the wait arms, and the wait retires in its first slot after the event, where the next instruction also runs. Otherwise the row retires now: `PC := next`, unless the row assigned PC, then `retire()`.
- The `pass` under `world(word)` is a stall. The slot is used up and the same word runs next slot.
- `memory()` fetches at PC in every memory cycle, so IR already holds the next instruction when a wait retires. `if RUN` is there because `retire()` can end a STEP (section 9.3).

**World instructions.** An instruction with W = `y` in the table touches the outside world. While an event is pending it stalls, using up its slot, and the same instruction runs again next slot. That covers pin writes and reads, `rds`, `rdd`, `wrd`, `tlate`, `halt`, every event instruction, every mailbox operation, and the shift-unit instructions that change what reaches a pad: `scfg`, `smap`, `sgo` and `sstop` (section 5.9.4). `txw`, `rxw`, `sstat` and `srd` are not world instructions, so a thread can keep a send fed while an event is pending. A shift with W = `pin` stalls only if it names a pin. ALU work, branches and memory access keep running. So program order is the order the world sees: a mailbox message can never overtake a pin edge.

**Visibility.** An event firing in cycle E is visible to instructions from E + 1. An instruction in cycle E still sees P = 1, so a world instruction in that cycle stalls, and a wait in that cycle doesn't retire yet. Instructions have no same-cycle bypass, in the simulators or in RTL, because lockstep needs one behaviour. The one exception is `stop()` (section 3.2). A stop acting in cycle E sees the wait as finished, so WAITING clears. In RTL its term is P at the start of the cycle and not firing in this cycle. The P flop alone would leave WAITING set on a wait that ended.

## 4. Memory model

### 4.1 One memory

One single-port memory holds code and data: a 128x16 latch array, or a 512x16 SRAM. DEPTH is 128 or 512. Firmware that fits 128 words runs unchanged on 512, and the pipeline and timing are the same for both.

PC, branch targets and `ld`/`st` addresses are taken modulo DEPTH when they reach memory. The assembler rejects a branch target that doesn't fit.

| Address | In v0.1 |
|---|---|
| 0 to DEPTH - 1 | Code, constants, tables and data, freely mixed |
| DEPTH to 0x1FF | Wraps. Reserved: firmware must not rely on it |
| 0x200 to 0xFFFF | Wraps. Reserved for a data RAM |

`ld` and `st` already take a 16-bit address, so adding a data RAM at 0x200 needs no new instructions and leaves existing firmware untouched.

Memory contents are undefined until written (section 10).

### 4.2 Memory cycles

Each cycle makes exactly one memory access, on behalf of the thread that owns it as a memory cycle:

```
memory():
    if MEM:
        if not MEM_DONE:
            if MEM_KIND == load:
                MEM_DATA := mem[MEM_ADDR]
            else:
                mem[MEM_ADDR] := MEM_DATA
            MEM_DONE := 1
    elif RUN:
        IR := mem[PC]
        IV := 1
    else:
        IV := 0

load(d, A):
    MEM := 1
    MEM_KIND := load
    MEM_REG := d
    MEM_ADDR := A % DEPTH

store(V, A):
    MEM := 1
    MEM_KIND := store
    MEM_DATA := V
    MEM_ADDR := A % DEPTH

finish_mem():
    if MEM_KIND == load:
        R[MEM_REG] := MEM_DATA
    MEM := 0
    MEM_DONE := 0
    PC := next
    retire()
```

A memory cycle that reaches `IV := 0` is free for the host.

So a word fetched in memory cycle *m* executes in cycle *m + 3*, and a stalled instruction is fetched again each slot. A thread executes in cycle *c* only if it was running at the fetch in cycle *c - 3*. An access never uses another thread's memory cycle, so memory use by one thread can't change another's timing.

**Load and store.** `ld` and `st` take the thread's next fetch for data, so they cost 2 slots and steal nothing from anyone. Executing in cycle *a*:

| Cycle | `ld Rd, Ra` | `st Rd, Ra` |
|---|---|---|
| *a* (slot 1) | Take the address from Ra. `ldp` does Ra += 1 | Take the address from Ra and the data from Rd. `stp` does Ra += 1 |
| *a + 1* (memory cycle) | Read memory | Write memory |
| *a + 4* (slot 2) | Rd := data. PC := next | PC := next |
| *a + 5* | Next fetch | Next fetch |

Two results follow. `stp R1, R1` stores the old value of R1. `ldp R1, R1` leaves the loaded data in R1. A load and a store in different threads are ordered by their memory cycles, and a store is seen by any fetch or load in a later memory cycle, including the fetch of the next instruction.

### 4.3 Data storage

| Store | Size | Notes |
|---|---|---|
| Registers | 8x16 per thread | The working store, 32 words in total |
| Mailbox | 8x16, shared | One word per slot, no FIFO behind it |
| Spare memory | Whatever code doesn't use | Read and written by `ld`/`st`. Constants and tables live here too |

At 128 words every data word is a code word lost. Long frames that don't fit in spare memory have to stream through the mailbox one word at a time. There is no capture buffer.

## 5. Pin model

### 5.1 Pin numbers

A pin field is 4 bits. Output and input numbering share the `uio` half:

| Field value | As an output | As an input |
|---|---|---|
| 0..7 | `uo0..uo7` | `ui0..ui7` |
| 8..15 | `io0..io7` | `io0..io7` |

Reading `uo` or writing `ui` can't be encoded.

Four pads belong to the host link: `ui0`, `ui1` and `ui2` as inputs, and `uo3` as an output. Firmware can read the `ui` ones. Output field 3 and shift-input field 0 double as "no pin", written `-`, so the host's pads can't be disturbed and shifts can run with no pin named.

### 5.2 Pad equations

```
V(p)       = LA[OWN[p]] if SEL[p] == 1 else LB[OWN[p]] if SEL[p] == 2 else OUT[p]
uo_out[3]  = host data out (section 9)
uo_out[i]  = BYP[i] ? ui_in[SRC(i)] : V(i)       for i != 3
uio_out[i] = OD[i] ? 0        : V(8 + i)
uio_oe[i]  = OD[i] ? !V(8 + i) : OE[i]
```

Bypass lanes exist only for `uo0` and `uo4..uo7`, with `SRC(0) = 3` and `SRC(i) = i` for the others. `uo1` and `uo2` always show OUT.

**Two halves.** These equations give each pad in the second half of a cycle, from its falling edge (section 3.1). In the first half, from the rising edge, the same equations hold with V0 in place of V:

```
V0(p)      = A0[OWN[p]] if SEL[p] == 1 else B0[OWN[p]] if SEL[p] == 2 else OUT[p]
A0[t]      = OLA[t] if TXH[t] else LA[t]
B0[t]      = OLB[t] if TXH[t] else LB[t]
```

TXH is set for the one cycle after a send half has moved its outputs half a cycle late (THALF, section 5.9.5), and OLA and OLB hold the levels from before that move. So in that cycle a pin on the shift output shows the old level until the falling edge and the new one after it, and the enable of a pin in `OD` moves the same way. In every other cycle V0 is V and the halves agree, as they always do on a pin that shows OUT. A bypass lane follows `ui_in` in both halves.

Pads have no pulls and no hysteresis. `uo` pads can't tri-state. Only `uio` pads can do open drain, turnaround or read-back. A `uio` input reads the pad even while the chip drives it, so a thread can read another thread's output with no outside wire, which gives a loopback self-test.

`LA[t]` and `LB[t]` are the shift outputs of thread *t*'s unit. SEL[p] is pin *p*'s output switch. A shift output replaces OUT and nothing else:

- **Open drain** works unchanged. A shift output on a pin in `OD` releases the pin on 1 and pulls it low on 0, and the pin still can't drive high.
- **OE** stays firmware's. A shift output on a `uio` pin in `PP` drives only while OE is 1, so a turnaround is still a committed `oe`.
- **Bypass wins.** On a pin in `BYP`, the output switch has no effect until the bypass switch is committed off.
- **Idle.** A pin switched to a shift output shows that output's level: the idle level when the send half is idle (after `sstop tx` or a tail), or the end level during a tail. After reset every shift output is 0.
- **Inputs** need no switch. Any unit can read any input pin, the pins of its own outputs included: a `uio` pin reads its own drive (section 5.2), so a shift output looped back to its own unit is a self-test with no outside wire. A change of a shift output written in cycle *w* is on the pad at *w* + 1, from its falling edge if THALF put it there, and in SYNC2 at *w* + 3 either way (section 3.1).

**Implementation note.** This is not normative. A shift output's two halves can come from a flop P that moves its pads at the rising edge and a flop N seen through a latch that is open while the clock is low, with the pads driven from P XOR the latch. For levels *v1* and *v2* in the two halves of cycle *c*, P takes *v1* XOR N's old value and N takes P's new value XOR *v2*, so N keeps its reset value 0 until a half moves, and the two inputs of the XOR never change in the same half cycle, so the pad doesn't glitch. The falling-edge sample is a pair of latches: the first, open while the clock is high, holds the pad's level from the falling edge, and the second, open while the clock is low, passes it on and holds it from the rising edge, where the flop SYNC1F takes it. So the first latch opening at that same edge never reaches SYNC1F, with real delays or none. No latch is state: at reset P, N and SYNC1F are 0, and the output latch and the second input latch are cleared with them.

### 5.3 Claiming pins

Every `uio` and `uo` pin has one owner. `pcfg` claims a pin for the thread running it and sets how the pin behaves:

```
claim(p):
    OWN[p] := TID
    SEL[p] := 0
    SHSEL[p] := 0

pcfg_in(p):
    claim(p)
    OD[p-8] := 0
    shadow_oe(p, 0)

pcfg_pp(p):
    claim(p)
    if p >= 8:
        OD[p-8] := 0
        shadow_oe(p, 1)
    elif p in LANES:
        SHBYP[p] := 0

pcfg_od(p):
    claim(p)
    OD[p-8] := 1
    OE[p-8] := 0
    SHOE[p-8] := 0
    OUT[p] := 1
    SHD[p] := 1

pcfg_byp(p):
    claim(p)
    SHBYP[p] := 1
```

`LANES` is 0, 4, 5, 6 and 7. `claim()` clears the pin's output switch (section 5.9), so any `pcfg` takes a pin off its shift output at once, even the owner's own `pcfg`, and the pin shows OUT.

| Mode | Applies to | Behaviour |
|---|---|---|
| `IN` | `uio` | Not driven, from the next commit. Direction can flip later with `oe`. A pin leaving `OD` releases at once |
| `PP` | `uo`, `uio` | Push-pull. On `uio`, drive starts at the next commit. On a lane, the switch leaves bypass at the next commit. A pin leaving `OD` releases at once and floats until that commit |
| `OD` | `uio` | Open drain. A prepared 1 releases the pin, a 0 pulls it low. Takes effect at once, released, and clears OE and SHOE |
| `BYP` | `uo0`, `uo4..uo7` | Output follows its lane source combinationally, from the next commit |

At reset thread 0 owns every pin. A thread's writes to pins it doesn't own are dropped, and its commit moves only pins it owns. `pcfg` takes a pin with no check, even from a running thread, so two protocols are kept off each other's lines only if firmware claims each pin once.

A hand-over the threads agree on is safe for a pin in `PP`. SHD is global, so the pin keeps its level when the owner changes, and `pcfg p, PP` on a pin already in `PP` moves nothing, unless the pin is on a shift output: then it moves to OUT at once. The new owner's next commit moves the pin to whatever SHD holds, including a value the old owner prepared and never committed, so the old owner should commit first. A pin on a shift output also needs the level it must keep: the old owner puts that level, switches the pin back with `smap p, -` and commits, all before the new owner's `pcfg`, which would otherwise move the pin to whatever OUT holds. `pcfg p, OD` is different: it releases the pin at once, even one held low.

**Open-drain safety is structural.** In `OD` mode the output data to the pad is forced to 0 and only the enable moves, so the pin can't drive high whatever firmware does. Entering `OD` always starts released, so `pcfg` doesn't pull a line low during init. OE and SHOE stay 0 for as long as a pin is in `OD`, so leaving `OD` can't drive a stale value either.

### 5.4 Shadow and commit

Nothing reaches a pad until an event commits it, and all of a thread's changes commit on the same cycle. So data and clock can change together, and a direction turnaround lands exactly where it should. The exceptions are `pcfg` into or out of `OD`, and `pcfg` of a pin on a shift output, which act at once; `sstop tx`, which moves the shift outputs to their idle levels at once; host ABORT (section 9.3); and a send, which a commit starts and which then moves its outputs by itself on the cycle grid that commit fixed.

```
shadow(p, v):
    if p != 3 and OWN[p] == TID:
        SHD[p] := v

shadow_oe(p, v):
    if OWN[p] == TID and not OD[p-8]:
        SHOE[p-8] := v

put8(b, v):
    for i in range(8):
        shadow(8 * b + i, bit(v, i))

oe8(v):
    for i in range(8):
        shadow_oe(8 + i, bit(v, i))

commit():
    for p in range(16):
        if OWN[p] == TID:
            OUT[p] := SHD[p]
            SEL[p] := SHSEL[p]
            if p >= 8:
                OE[p-8] := SHOE[p-8]
            elif p in LANES:
                BYP[p] := SHBYP[p]
    CMT := 1
```

OD and OWN have no shadow. `oe` and `oe8` do nothing on a pin in `OD` mode. `shadow_sel()` is in section 5.9.4.

### 5.5 Reading

`get p` puts `S[p]` in T, and `getn`, `xget` and `aget` combine it instead. `xget` compares a driven bit with the line, which is the check arbitration needs. `rds Rn` takes all 16 bits. Reads always come from the snapshot, and the snapshot takes SYNC2, so S holds what the pads showed 2 cycles before the event. Nothing reads a live pin except an event.

A pin committed at E reaches the pad at E + 1, so it shows in a snapshot taken from E + 3, and a wait compares it from E + 3 too. A read-back with `xget`, or a wait armed straight after your own edge, has to leave that gap, plus the bus rise time on an open-drain line.

### 5.6 Waits

`until p, l` waits on one pin. `untilm Rr, b, c` takes the mask in `Rr[15:8]` and the pattern in `Rr[7:0]`, over one 8-pin bank (`ui` or `io`):

| Mode `c` | Fires when |
|---|---|
| `eq` | Masked pins equal the pattern |
| `ne` | Masked pins differ from the pattern |
| `chg` | Masked pins differ from S as it was when the wait armed |

`chg` is the general edge detector. Because the reference is the last snapshot, a change that happens before the wait arms still fires. Masks are checked on one bank, so a wait can't combine a `ui` pin with a `uio` pin.

Rt adds a timeout at D + Rt, and T reads 0 if it expires. `inbox` also ends the wait when this thread's own mailbox word M[TID] is full, with T = 0. Firmware tells a timeout from an inbox exit with `tryrecv`. A thread's inbox word is M0 for thread 0 up to M3 for thread 3. M4..M7 belong to no thread.

### 5.7 Input latency and reaction

Inputs pass a two-flop synchroniser with no glitch filter. A pad change in cycle *p* is in SYNC2 at *p + 2*. A commit in cycle E reaches the pad at *E + 1*. The falling-edge sample of cycle *p* is in SYNC2F at *p + 2* too, half a cycle before SYNC2's sample (section 3.1); only RKIND 3 reads it.

- **Pre-armed reaction.** Prepare pins, then wait: the match commits them. An input edge at the pad moves an output 3 cycles later, exactly, on every thread and phase run.
- **Timed reaction.** After a match in cycle E, `edge p, v, n` fires at E + n. The smallest step on time on every alignment is n = 5, which is 8 cycles pad to pad with no jitter.
- **Sampling.** The start edge and later samples pass through the same synchroniser, so the 2 cycles cancel. After a wait matches, `tick n` snapshots what the pad showed exactly *n* cycles after the edge.
- **Sampling on your own clock edge.** The event that commits a clock edge at E also snapshots, but the pad moves at E + 1 and S holds the pad at E - 2, so the sample looks 3 cycles ahead of the edge at the pad. An SPI target's clock-to-out therefore has to fit in the half period minus 3 cycles, which is 6 cycles at half period 9.

### 5.8 Bypass

Mode `BYP` gives a pad-to-pad path with no flop in it. The switch is committed state like any other pin, so it changes at an event, on an exact cycle. There is no crossbar: pins are used where they are.

### 5.9 Shift units

#### 5.9.1 Overview

Each thread has one shift unit with a send half and a receive half.

- The **send half** turns 16-bit words into a stream of symbols on two **shift outputs**, A and B. Each symbol lasts a set number of cycles and 256ths of a cycle, placed to the cycle or, with THALF, to the half cycle, or ends at an edge of an outside clock. An output's value per symbol comes from a small table, so one mechanism gives NRZ, NRZI, Manchester, biphase mark, return to zero, a data line with its clock, and a complement for a differential pair.
- The **receive half** turns one input pin into 16-bit words. It samples at a set period, decodes a Manchester-family code against a predicted clock edge that every input edge corrects, with edges timed to the cycle or, with RKIND 3, to the half cycle, or samples at the edges of an outside clock. It can wait for a sync pattern before it starts a word, and it ends a reception after a set number of bits, a set time without edges, or, for Manchester, the first bit with no clock edge.
- A **CRC register** of 32 bits serves one half at a time (section 5.9.7). As a CRC of any width from 1 to 32 bits and any polynomial, it takes one bit per data bit in wire order, appends the CRC after a send's covered words, and checks a reception against a set residue. As an LFSR it scrambles a send or descrambles a reception, which also gives a PRBS. A **bit stuffer** inserts a bit after a set number of equal bits on send, and the **destuffer** removes it on receive and ends a reception on a stuff violation.

A unit starts only at a commit of its own thread, so its first symbol lands on the exact cycle of an event (section 3.3). From then on it runs by itself on a cycle grid fixed by that commit. The thread only fills and empties its buffers, through instructions that block like `send` and `recv`. A pin follows a shift output only if its owner switches it to that output, and the switch is committed like any other pin state (section 5.4). In this section "lane" keeps the meaning of section 5.2, a bypass lane.

The unit counts cycles per symbol. It never changes when a thread issues instructions, so every thread still runs one instruction every 4 cycles. Units share nothing with each other, so one thread's unit can't change another thread's timing.

#### 5.9.2 State

**Per-thread state that firmware or the host can see:**

| State | Bits | Purpose |
|---|---|---|
| CFG[0..15] | 16 each | Configuration slots, written by `scfg` (section 5.9.3). Every bit is a field, so all 16 are stored |
| CRC | 32 | The CRC register, or the LFSR (section 5.9.7). `srd` and RDREG read it |
| RXPE | 3 | End flags of the word `rxw` took last (section 5.9.7). `srd` and RDREG read them |

The status word that `sstat` and RDREG read (section 5.9.4) is not stored: both compute it from the flags below.

**Per-thread sequencing state.** Firmware sees it only through the status word. RTL may hold it in any form that behaves the same.

| State | Bits | Purpose |
|---|---|---|
| TXARM | 1 | The send half starts at the thread's next commit |
| TXRUN | 1 | The send half is sending symbols |
| TXTL | 1 | The send half is in its tail |
| TXSR, TXN, TXLW | 16, 5, 1 | Shift register, bits left in it (0 to 16), and whether its word is the last |
| TXBUF, TXBN, TXBL, TXFULL | 16, 5, 1, 1 | The word waiting behind the shift register, its bit count, its last flag, and whether it is there |
| TXSYM | 2 | Symbol of the current bit, 0 to SYMS |
| TXCD | 17 | Cycles left in the current symbol or tail |
| TXACC | 8 | Fraction accumulator for TFRAC |
| LA, LB | 1 each | The levels of shift outputs A and B |
| TXH | 1 | The last move of LA and LB is half a cycle late: set in the cycle of a move that THALF puts at the next falling edge, so that in the next cycle the pads show OLA and OLB until that falling edge (section 5.2) |
| OLA, OLB | 1 each | LA and LB as they were at the start of the cycle, written by every `tx_step` |
| TXUR | 1 | Sticky. A send ran out of words before its last word |
| TXK, TXBK | 2 each | The kind (section 5.9.7) of the word in the shift register, and of the word in the buffer |
| TXB | 1 | The bit of the current symbol, as the line coder takes it |
| TXSTF | 1 | The current bit is a stuff bit |
| TXAN | 6 | CRC bits still to append, 0 to 32 |
| TXRL, TXRV | 3, 1 | The stuffer's run: equal bits in a row, 0 to 7, and their value |
| RXARM | 1 | The receive half starts, or starts hunting, at the thread's next commit |
| RXHUNT | 1 | Waiting for an edge on its pin to start a reception |
| RXRUN | 1 | A reception is in progress |
| RXEP | 1 | A reception has ended, and its last word waits for the buffer |
| RXSR, RXN | 16, 5 | Shift register, and bits in it (0 to 16) |
| RXTOT | 11 | Bits received since the sync pattern, modulo 2048 |
| RXSYN | 1 | The sync pattern has been found, in this reception or the last one |
| RXKIND | 2 | The decoder of the reception in progress: RKIND as it was at the start |
| RXPH | 25 | The receive phase, signed, in 256ths of a cycle. Timed samples: time to the next sample. Manchester decoder: time since the predicted clock edge |
| RXG | 5 | Reference edges since the start, counting the start as 1, up to 16, and for timed samples only the edges that pull. It sets the gain and ends RACQ's wide window |
| RXOPEN | 1 | The Manchester decoder's window is open: the predicted clock edge has had no edge yet |
| RXCAND | 1 | An edge came in the open window before the predicted clock edge |
| RXCE | 16 | RKIND 3: the phase of that edge, RXPH when it came, signed (always below 0) |
| RXP3 | 12 | RKIND 3's p as the reception found it |
| RXRG, RXRA | 2, 1 | RGAIN and RACQ as the reception found them. Only RKIND 3 reads them: timed samples and RKIND 1 read RGAIN and RACQ as they are in each cycle, like their other options |
| RXIC | 16 | Cycles since the last reference edge, or, for the Manchester decoder, since the last bit. It stops at 65535 |
| RXLAST | 1 | The last level sampled, for NRZI |
| RXWIN | 1 | An edge other than the clock edge came since the last clock edge, for RDIFF |
| RXBUF, RXBN, RXBL, RXFULL | 16, 5, 1, 1 | The received word waiting for `rxw`, its bit count, its last flag, and whether it is there |
| RXPN | 5 | Bit count of the word `rxw` took last |
| RXOVR | 1 | Sticky. A full word was lost because the buffer was full |
| RXSEEN | 1 | Sticky. A reception started |
| RXRL, RXRV | 3, 1 | The destuffer's run: equal bits in a row, 0 to 7, and their value |
| RXVIO | 1 | The destuffer has seen a stuff violation with SMODE 1, and the next bit ends the reception. With SMODE 2 the violating bit ends it, and RXVIO stays 0 |
| RXCOV | 1 | The CRC covers the bits of this reception from here on |
| RXEF, RXBE | 3 each | End flags (section 5.9.7) of the reception in progress, and of the word in the buffer |
| RXSR8, RXN8 | 16, 5 | RXSR and RXN at the last byte boundary: at the start, and after every 8th bit after the sync pattern (section 5.9.7) |
| RXC8 | 32 | The CRC register at the last byte boundary |
| RXONES | 1 | Every bit since the last byte boundary is a 1 |
| CMT | 1 | The thread's prepared pins commit in this cycle. Lasts one cycle, like HIT |

**Global state:**

| State | Bits | Purpose |
|---|---|---|
| SYNC3 | 16 | SYNC2 one cycle older, so that an edge on any pin is `SYNC2 ^ SYNC3` |
| SYNC2F | 16 | The falling-edge sample, half a cycle after SYNC3's and half a cycle before SYNC2's (section 3.1), for RKIND 3 |
| SEL, SHSEL | 2 x 16 each | The output switch of each output pin, and its shadow: 0 is OUT, 1 is shift output A, 2 is shift output B of the pin's owner |

#### 5.9.3 Configuration

`scfg Rs, k` writes Rs into slot k. In the code blocks of section 5.9, a field name stands for its value: the slot shifted right by the field's low bit and masked to its width.

<!-- shift-fields-begin -->
| Field | Slot | Bits | Meaning |
|---|---|---|---|
| SYMS | 0 `TXMODE` | 1:0 | Symbols per bit, minus 1 |
| TMSB | 0 | 2 | 1: send bit n - 1 of an n-bit word first. 0: bit 0 first |
| TGLA | 0 | 3 | 1: a symbol value of 1 toggles output A, and 0 holds it. 0: output A takes the value |
| TGLB | 0 | 4 | The same for output B |
| IDLEA | 0 | 5 | Output A while the send half is idle |
| IDLEB | 0 | 6 | Output B while idle |
| ENDA | 0 | 7 | Output A during the tail |
| ENDB | 0 | 8 | Output B during the tail |
| TEXT | 0 | 9 | 1: a symbol ends at an edge of pin TCLK. 0: after TPER cycles |
| TFALL | 0 | 10 | 1: that edge is falling. 0: rising |
| TCLK | 0 | 14:11 | Input pin of the outside clock, numbered as `ipin` |
| THALF | 0 | 15 | 1: a symbol whose start the fraction puts half a cycle or more past a rising edge starts at the falling edge in the middle of that cycle, so symbols lie on a grid of half cycles (5.9.5). TEXT and TPER 0 ignore it. 0: every symbol starts at a rising edge |
| MAPA | 1 `TXMAP` | 7:0 | Output A's code for symbols 0 to 3, two bits each, symbol 0 in bits 1:0 |
| MAPB | 1 | 15:8 | Output B's codes, the same way |
| TPER | 2 `TXPER` | 15:0 | Cycles per symbol. 0 acts as 1 and ignores TFRAC |
| TAIL | 3 `TXTAIL` | 7:0 | Cycles of the end levels after the last symbol. 0: no tail |
| TFRAC | 3 | 15:8 | Fraction of a cycle added to each symbol, in 256ths. TPER 0 ignores it. With THALF the fraction also places each symbol to the half cycle |
| RKIND | 4 `RXMODE` | 1:0 | 0: timed samples. 1: Manchester decoder. 2: outside clock. 3: Manchester decoder on half cycles, which times every edge to the half cycle with SYNC2F (5.9.6). A reception keeps the decoder it started with |
| RMSB | 4 | 2 | 1: the first bit of a word ends up in bit n - 1. 0: in bit 0 |
| RINV | 4 | 3 | 1: invert every decoded bit |
| RDIFF | 4 | 4 | Timed and outside clock: 1 decodes NRZI, where a change is 0. Manchester decoder: 1 makes the bit "an edge other than a clock edge came between the last two clock edges" |
| HUNT | 4 | 5 | 1: after its commit, and after each reception, the receive half waits for an edge to HLVL on RPIN and starts there |
| HLVL | 4 | 6 | The level that edge goes to |
| RFALL | 4 | 7 | Outside clock: 1 samples at a falling edge of RCLK, 0 at a rising one |
| RPIN | 4 | 11:8 | Input pin of the data, numbered as `ipin` |
| RCLK | 4 | 15:12 | Input pin of the outside clock, numbered as `ipin`. With REOP, timed samples read it as the second line of the end test (section 5.9.7) |
| RPER | 5 `RXPER` | 15:0 | Timed: cycles per sample. Manchester decoder: cycles per bit. Both add RFRAC. 0 acts as 1 and ignores RFRAC. For the Manchester decoder 256 and up are reserved and act as 255, and with RKIND 3, 0 and 1 are reserved and act as 2, with no RFRAC, and 16 and up are reserved and act as 15 |
| FIRST | 6 `RXFIRST` | 15:0 | Timed: cycles from the start to the first sample. With RESYNC, an edge pulls the next sample toward FIRST cycles after it, all the way with RGAIN 0. 0 acts as 1 |
| REND | 7 `RXEND` | 15:0 | Cycles since the last reference edge, or for the Manchester decoder since the last bit, that end a reception. 0: never |
| RCNT | 8 `RXCNT` | 10:0 | Bits after the sync pattern that end a reception. 0: no limit |
| SYNCLEN | 8 | 15:11 | Bits in the sync pattern, 0 to 16. 0: no sync, words start at once. 17 to 31 are reserved and act as 16 |
| SYNC | 9 `RXSYNC` | 15:0 | The sync pattern, laid out as a received word (section 5.9.6) |
| RFRAC | 10 `RXOPT` | 7:0 | Fraction of a cycle added to RPER, in 256ths |
| RESYNC | 10 | 8 | Timed: 1 makes every edge of RPIN pull the sample countdown toward FIRST, or with RACQ every edge to HLVL |
| RGAIN | 10 | 10:9 | How far an edge corrects the receive phase: by 2^-k of its error. 0: k = 0, a full correction. 1, 2, 3: k = 1 up to and including the third reference edge after the start, then k = 2, 4 or 5 |
| RACQ | 10 | 11 | Manchester decoder with RGAIN 1 to 3: 1 opens the window half a bit before the predicted clock edge, not a quarter, until RXG reaches 16. For receptions that begin with alternating bits. RGAIN 0 ignores it. Timed samples with RESYNC: 1 makes only the edges of RPIN to HLVL pull, as a CAN receiver resynchronizes only on recessive-to-dominant edges (5.9.6). An outside clock ignores it |
| CSKIP | 10 | 15:12 | Bits after the sync pattern that the receive half's CRC or LFSR leaves out, 0 to 15 (section 5.9.7) |
| POLYH | 11 `POLYHI` | 15:0 | The CRC or LFSR polynomial, left-aligned, without its top term: bits 31 to 16 (section 5.9.7) |
| POLYL | 12 `POLYLO` | 15:0 | Its bits 15 to 0 |
| CHKH | 13 `CHKHI` | 15:0 | The residue that a good reception leaves in CRC, left-aligned: bits 31 to 16 |
| CHKL | 14 `CHKLO` | 15:0 | Its bits 15 to 0 |
| CW | 15 `CODING` | 4:0 | The CRC's width *w*, minus 1: 0 to 31 for widths 1 to 32 |
| CINIT | 15 | 5 | 1: CRC starts with its top *w* bits all ones and the rest 0. 0: all zeros |
| CXOR | 15 | 6 | 1: the appended CRC is complemented. 0: sent as it is |
| CMODE | 15 | 8:7 | 0: the CRC register is off. 1: a CRC. 2: an LFSR, which scrambles or descrambles. 3: reserved, acts as 0 |
| CTX | 15 | 9 | 1: the send half uses the CRC register. 0: the receive half |
| SMODE | 15 | 11:10 | 0: no stuffing. 1: a run is of ones, and its stuff bit is a 0 (USB, HDLC). 2: a run is of equal bits, and its stuff bit is the complement (CAN). 3: reserved, acts as 0 |
| SN | 15 | 14:12 | Bits in a run that a stuff bit follows: 1 to 7 with SMODE 1, 2 to 7 with SMODE 2. 0 is reserved: it acts as 1 with SMODE 1, and with SMODE 2 both 0 and 1 act as 2 |
| REOP | 15 | 15 | Timed samples: 1 ends a reception at a sample where RPIN and RCLK both read 0, such as a USB SE0, and that sample gives no bit. If 1 to 6 bits follow the last byte boundary, all are ones and no stuff violation came, the reception ends at that boundary instead (5.9.7, the byte end). RKIND 3: 1 makes a window that closes empty end the reception at the last byte boundary when 1 to 7 bits follow it and no stuff violation came, as an IEEE 802.3 MAC truncates a frame to whole octets (5.9.7). RKIND 1 and 2 and the send half ignore it |
<!-- shift-fields-end -->

A **reference edge** is a clock edge for the Manchester decoder, of either kind, any edge of RPIN for timed samples, and an active edge of RCLK for an outside clock.

**Output codes.** Each symbol of each bit *b* gives each shift output a value:

| Code | Value |
|---|---|
| 0 | *b* |
| 1 | 1 - *b* |
| 2 | 0 |
| 3 | 1 |

So reset leaves a unit sending plain NRZ data on both outputs, at one symbol per cycle.

**Line codes.** With SYMS, the codes in symbol order, and the toggle bit:

| Line code | SYMS | Output A codes | TGLA | Output B, for a pair |
|---|---|---|---|---|
| NRZ | 0 | *b* | 0 | 1 - *b* |
| NRZI, a 0 changes the line (USB, HDLC) | 0 | 1 - *b* | 1 | The same codes, TGLB 1, IDLEB = 1 - IDLEA |
| Manchester, IEEE 802.3: a 1 is low then high | 1 | 1 - *b*, *b* | 0 | *b*, 1 - *b* |
| Manchester, G. E. Thomas: a 0 is low then high | 1 | *b*, 1 - *b* | 0 | 1 - *b*, *b* |
| Biphase mark (S/PDIF, FM1): a change at every bit start, and mid-bit for a 1 | 1 | 1, *b* | 1 | The same codes, TGLB 1, IDLEB = 1 - IDLEA |
| Differential Manchester (IEEE 802.5): a change mid-bit always, and at the bit start for a 0 | 1 | 1 - *b*, 1 | 1 | |
| Return to zero | 1 | *b*, 0 | 0 | |
| Three thirds, high, data, low (WS2812 style) | 2 | 1, *b*, 0 | 0 | |
| Data with its clock, SPI mode 0 | 1 | *b*, *b* | 0 | Clock: 0, 1 |
| Data with its clock, SPI mode 1 | 1 | *b*, *b* | 0 | Clock: 1, 0 |

For modes 2 and 3 set IDLEB to 1 and swap the clock codes. As TXMAP words: 802.3 Manchester with its complement is 0x0401, SPI mode 0 is 0x0E00, and a USB NRZI pair is 0x0101. 10BASE-T at 50 MHz is TPER 2 with TFRAC 128 and THALF: 2.5 cycles a half bit (5.9.5).

Each symbol's value comes from the current bit alone, or toggles the output on it. So a code that depends on earlier bits can't be written: Miller (delay modulation), AMI and other bipolar codes, whose marks alternate in polarity, and two-wire codes whose lines swap clock and data roles, such as the Maple bus. Firmware sends those. A one-line code among them can also be encoded into memory first, one bit per half bit, and sent as NRZ.

#### 5.9.4 Instructions

The eight instructions are rows of the table in section 6.3, between `tick` and `edge`, and section 6.2 gives their operand types.

- **`txw Rs, n [, k] [, last]`** puts an *n*-bit word in the send buffer. It blocks while the buffer is full. `last` marks the word that ends the send. The kind *k* says what section 5.9.7 does to the word's bits: with no kind, nothing; `stuff`, stuffing only; `crc`, the CRC and stuffing; `fcs`, the CRC and stuffing, and the CRC is appended after the word's last bit. Stuffing needs the stuffer on, and the CRC needs the CRC register on for the send half. While the send half is idle, a word moves from the buffer into the empty shift register in the cycle after its `txw`, so two words can wait before a start. While it runs, the buffer is read at each word boundary.
- **`rxw Rd`** takes the received word, and T says whether it is the last word of its reception. It blocks while the buffer is empty. `sstat` then gives the word's bit count.
- **`sstat Rd`** reads the status word. T is 1 if TXUR or RXOVR was set, and the sticky bits read as 1 are cleared. On a thread that both sends and receives, bits 7 and 10 tell the two apart.
- **`srd Rd, s`** reads the CRC register, its bits 15 to 0 with `lo` and 31 to 16 with `hi`, or with `end` the end flags of the word `rxw` took last (section 5.9.7). It changes nothing.
- **`scfg Rs, k`** writes configuration slot *k*.
- **`sgo m`** arms a start of the send half, the receive half or both. The start happens at the thread's next commit.
- **`sstop m`** stops a half at once and empties its buffers. A stopped send half shows its idle levels on its outputs from the next cycle.
- **`smap p, l`** prepares the output switch of pin *p*: `-` for OUT, `A` or `B` for a shift output of the owner's unit. It takes effect at the next commit, like `put`.

**Status word:**

| Bits | Meaning |
|---|---|
| 4:0 | RXPN, the bit count of the word `rxw` took last, 0 to 16 |
| 5 | The send half is armed, sending or in its tail |
| 6 | TXFULL |
| 7 | TXUR, sticky |
| 8 | RXRUN: a reception is in progress. It is carrier sense only while the receive loop keeps up: while a reception's last word waits for the buffer (RXEP), the half doesn't hunt, so a frame that starts then isn't seen. That needs a full buffer, so bits 8 and 9 both 0 mean no reception and nothing waiting |
| 9 | RXFULL |
| 10 | RXOVR, sticky |
| 11 | RXSYN |
| 12 | RXSEEN, sticky |
| 13, 14 | LA, LB |
| 15 | The receive half is armed, hunting, or holding its last word |

```
tx_push(v, n, last, kind):
    TXBUF := v
    TXBN := 16 if n == 0 else n
    TXBL := last
    TXBK := kind
    TXFULL := 1

rx_pop():
    RXPN := RXBN
    RXPE := RXBE
    RXFULL := 0

srd(d, s):
    if s == 0:
        R[d] := CRC & 0xFFFF
    elif s == 1:
        R[d] := CRC >> 16
    else:
        R[d] := RXPE

sstat(d):
    tb := 1 if TXARM or TXRUN or TXTL else 0
    rw := 1 if RXARM or RXHUNT or RXEP else 0
    f := RXPN | (tb << 5) | (TXFULL << 6) | (TXUR << 7) | (RXRUN << 8) | (RXFULL << 9)
    f := f | (RXOVR << 10) | (RXSYN << 11) | (RXSEEN << 12) | (LA << 13) | (LB << 14) | (rw << 15)
    R[d] := f
    T := TXUR | RXOVR
    if TXUR:
        TXUR := 0
    if RXOVR:
        RXOVR := 0
    if RXSEEN:
        RXSEEN := 0

set_cfg(k, v):
    CFG[k] := v & KEEP[k]

sgo(m):
    if m & 1:
        TXARM := 1
    if m & 2:
        RXARM := 1

sstop(m):
    if m & 1:
        tx_stop()
    if m & 2:
        rx_stop()

shadow_sel(p, l):
    if OWN[p] == TID:
        SHSEL[p] := l
```

`KEEP[k]` is the mask of the bits that the field table's rows define in slot *k*. Every bit of every slot has a row, so each `KEEP[k]` is 0xFFFF.

#### 5.9.5 Send

Each cycle, after the events of step 2, every thread runs `tx_step()`.

```
tx_step():
    OLA := LA
    OLB := LB
    TXH := 0
    if TXRUN:
        tx_run()
    elif TXTL:
        tx_tail()
    else:
        if TXN == 0 and TXFULL:
            tx_load()
        if TXARM and CMT:
            TXARM := 0
            TXACC := 0
            if TXN == 0:
                TXUR := 1
                tx_end()
            else:
                TXRUN := 1
                TXSYM := 0
                tx_begin()
                tx_symbol()

tx_half():
    TXH := 1 if THALF and not TEXT and TPER != 0 and bit(TXACC, 7) else 0

tx_load():
    TXSR := TXBUF
    TXN := TXBN
    TXLW := TXBL
    TXK := TXBK
    TXFULL := 0

tx_symbol():
    if TXSTF:
        b := 1 - TXRV
    elif TXAN != 0:
        b := bit(CRC, 31) ^ CXOR if CTX else CXOR
    else:
        b := bit(TXSR, TXN - 1) if TMSB else bit(TXSR, 0)
        if CMODE == 2 and CTX and TXK >= 2:
            b := b ^ bit(CRC, 31)
    TXB := b
    va := [b, 1 - b, 0, 1][(MAPA >> (2 * TXSYM)) & 3]
    vb := [b, 1 - b, 0, 1][(MAPB >> (2 * TXSYM)) & 3]
    LA := LA ^ va if TGLA else va
    LB := LB ^ vb if TGLB else vb
    tx_half()
    if TPER == 0:
        TXCD := 1
    else:
        TXACC := TXACC + TFRAC
        TXCD := TPER + (TXACC >> 8)
        TXACC := TXACC & 255

tx_run():
    if TEXT:
        due := bit(SYNC2, TCLK) != bit(SYNC3, TCLK) and bit(SYNC2, TCLK) != TFALL
    else:
        due := TXCD <= 1
    if not due:
        if not TEXT:
            TXCD := TXCD - 1
    elif TXSYM < SYMS:
        TXSYM := TXSYM + 1
        tx_symbol()
    else:
        TXSYM := 0
        tx_done()
        if TXSTF or TXN != 0 or TXAN != 0:
            tx_symbol()
        elif TXLW:
            tx_end()
        elif TXFULL:
            tx_load()
            tx_symbol()
        else:
            TXUR := 1
            tx_end()

tx_end():
    TXRUN := 0
    tx_half()
    if TAIL == 0:
        LA := IDLEA
        LB := IDLEB
    else:
        TXTL := 1
        TXCD := TAIL
        LA := ENDA
        LB := ENDB

tx_tail():
    if TXCD <= 1:
        TXTL := 0
        tx_half()
        LA := IDLEA
        LB := IDLEB
    else:
        TXCD := TXCD - 1

tx_stop():
    TXARM := 0
    TXRUN := 0
    TXTL := 0
    TXN := 0
    TXFULL := 0
    TXSYM := 0
    TXSTF := 0
    TXAN := 0
    TXH := 0
    LA := IDLEA
    LB := IDLEB
```

What follows from the code:

- **The grid.** A symbol written in cycle *w* is on the pad from *w* + 1 (section 5.7), for exactly TPER cycles plus any fraction carry. TPER 0 gives one cycle and leaves TXACC alone, whatever TFRAC is. The commit at E writes the first symbol, so with no fraction, symbol *k* of a send is on the pad from E + 1 + *k* x TPER, and a word of *n* bits lasts *n* x (SYMS + 1) x TPER cycles, with TPER 0 counted as 1 in both. A stuff bit or an appended CRC bit (5.9.7) is a bit like any other: it lasts SYMS + 1 symbols on the same grid.
- **Half cycles.** TXACC holds the fraction carried into the next symbol. With THALF, a symbol, a tail or an idle level that starts while it is 128 or more sets TXH in `tx_half`, so the pads show the new levels from the falling edge of the cycle after the write, half a cycle later than without THALF (section 5.2). So symbol *k* of a send with TPER 1 or more starts on the pad at E + 1 + *k* x (TPER + TFRAC / 256) rounded down to a half cycle, where without THALF it is rounded down to a cycle. With TPER 2 and TFRAC 128 every symbol lasts exactly 2.5 cycles, 10 Mbit/s Manchester at 50 MHz, where without THALF they alternate 2 and 3. The first symbol still lands at E + 1, because a start sets TXACC to 0. The tail keeps TXACC, so it lasts exactly TAIL cycles on the pad and the idle level after it lands on the same half. `sstop tx` and ABORT move the outputs at a rising edge. The grid is exact to the half cycle only with a 50% clock (section 7.3).
- **Refill.** The buffer is read only at a word boundary. A `txw` in the same cycle as the boundary is too late, because it fills the buffer at the end of that cycle.
- **The end.** After the last symbol of the word marked `last`, and of its CRC and any stuff bit if it is of kind `fcs` (5.9.7), the outputs show ENDA and ENDB for TAIL cycles, then IDLEA and IDLEB. With TAIL = 0 they go straight to the idle levels. A word marked `last` always ends the send: a word already in the buffer then belongs to the next send, and moves into the shift register once the tail is over.
- **Underrun.** If the shift register empties, the buffer is empty and the word wasn't marked `last`, TXUR sets and the send ends exactly as a normal end does, tail included. A start with no word does the same. The bit grid isn't kept, because a receiver can't use half a frame anyway, while a defined end leaves the line in a state the protocol allows.
- **Outside clock.** With TEXT, the first symbol still lands at the commit, and each later symbol starts in the cycle an active edge of TCLK reaches SYNC2. A pad edge in cycle *p* moves the output at the pad in *p* + 3, the same 3 cycles as a pre-armed reaction. The tail still counts cycles.
- **`sgo tx` while busy** stays armed. The first commit after the send half is idle starts the next send.

#### 5.9.6 Receive

Each cycle, after `tx_step()`, every thread runs `rx_step()`.

```
rx_step():
    if RXEP:
        if not RXFULL:
            rx_deliver(RXSR, RXN, 1)
            RXEP := 0
            if HUNT:
                RXHUNT := 1
    elif RXRUN:
        rx_run()
    elif RXHUNT:
        if bit(SYNC2, RPIN) == HLVL and bit(SYNC3, RPIN) != HLVL:
            rx_begin()
    elif RXARM and CMT:
        RXARM := 0
        if HUNT:
            RXHUNT := 1
        else:
            rx_begin()

rx_begin():
    h := RXHUNT and RKIND == 3 and bit(SYNC2F, RPIN) != bit(SYNC2, RPIN)
    RXHUNT := 0
    RXRUN := 1
    RXSEEN := 1
    RXSR := 0
    RXN := 0
    RXTOT := 0
    RXSYN := 1 if SYNCLEN == 0 else 0
    RXIC := 0
    RXWIN := 0
    RXLAST := bit(SYNC3, RPIN)
    RXKIND := RKIND
    RXOPEN := 0
    RXCAND := 0
    RXG := 1
    RXP3 := 512 if RPER < 2 else ((0xF00 | RFRAC) if RPER > 15 else (RPER << 8) | RFRAC)
    RXRG := RGAIN
    RXRA := RACQ
    rx_cbegin()
    if RXKIND == 1 or RXKIND == 3:
        RXPH := 128 if h else 256
        if not RDIFF:
            rx_bit(bit(SYNC2, RPIN))
    else:
        RXPH := (FIRST - 1 if FIRST else 0) << 8

rx_run():
    e := bit(SYNC2, RPIN) != bit(SYNC3, RPIN)
    if RXIC < 65535:
        RXIC := RXIC + 1
    p := 256 if RPER == 0 else (RPER << 8) | RFRAC
    if RXKIND == 1:
        if RPER > 255:
            p := 0xFF00 | RFRAC
        rx_manch(e, bit(SYNC2, RPIN), p, 256)
    elif RXKIND == 3:
        rx_manch3(RXP3)
    elif RXKIND == 2:
        if bit(SYNC2, RCLK) != bit(SYNC3, RCLK) and bit(SYNC2, RCLK) != RFALL:
            RXIC := 0
            rx_sample()
    else:
        rx_timed(e, p)
    if RXRUN and REND != 0 and RXIC >= REND:
        rx_end()

rx_timed(e, p):
    if RXPH < 256:
        RXPH := RXPH - 256 + p
        if REOP and bit(SYNC2, RPIN) == 0 and bit(SYNC2, RCLK) == 0:
            rx_eop()
        else:
            rx_sample()
    else:
        RXPH := RXPH - 256
    if e:
        RXIC := 0
        if RESYNC and (not RACQ or bit(SYNC2, RPIN) == HLVL):
            rx_pull((FIRST - 1 if FIRST else 0) << 8)
            rx_gear()

rx_manch(e, v, p, s):
    w := (p >> 1) if RACQ and RGAIN != 0 and RXG < 16 else (p >> 2)
    if RXPH >= p - w:
        if RXOPEN:
            rx_end()
        RXOPEN := 1
        RXCAND := 0
        RXPH := RXPH - p
    if RXRUN and RXOPEN:
        if e and RXPH < 0:
            if RXCAND:
                RXWIN := 1
            RXCAND := 1
            rx_pull(0)
        elif e or (RXCAND and RXPH >= 0):
            RXOPEN := 0
            RXCAND := 0
            RXIC := 0
            rx_bit(RXWIN if RDIFF else v)
            RXWIN := 0
            if e:
                rx_pull(0)
            rx_gear()
    elif RXRUN and e:
        RXWIN := 1
        rx_pull(p >> 1)
    RXPH := RXPH + s

rx_manch3(p):
    w := (p >> 1) if RXRA and RXRG != 0 and RXG < 16 else (p >> 2)
    k := 0 if RXRG == 0 else (1 if RXG <= 3 else (2 if RXRG == 1 else (4 if RXRG == 2 else 5)))
    pl := 0
    q := 0
    nb := 0
    two := bit(SYNC3, RPIN) != bit(SYNC2F, RPIN) and bit(SYNC2F, RPIN) != bit(SYNC2, RPIN)
    for h in range(2):
        if RXRUN:
            u := bit(SYNC2F, RPIN) if h else bit(SYNC3, RPIN)
            v := bit(SYNC2, RPIN) if h else bit(SYNC2F, RPIN)
            e := u != v
            if RXPH >= p - w:
                if RXOPEN:
                    rx_mend()
                RXOPEN := 1
                RXCAND := 0
                RXPH := RXPH - p
            if RXRUN and e and RXWIN and not RXCAND and not nb and (RXOPEN or RXPH >= p - w - (p >> 3)):
                if not RXOPEN:
                    RXPH := RXPH - p
                b := 1 if RDIFF else v
                if not pl:
                    pl := 1
                    q := RXPH >> k
                RXOPEN := 0
                RXWIN := 0
                RXIC := 0
                nb := 1
                rx_gear()
                rx_bit(b)
            elif RXRUN and RXOPEN:
                if e and RXPH < 0:
                    if RXCAND:
                        RXWIN := 1
                    RXCAND := 1
                    RXCE := RXPH
                elif e or (RXCAND and RXPH >= (p >> 3)):
                    kp := RXCAND and RXPH >= (p >> 3) and RXCE >= -(128 if p < 768 else (p >> 3))
                    b := (RXWIN or (RXCAND and e and not kp)) if RDIFF else (u if kp else v)
                    r := RXPH if e and not kp else RXCE
                    if not pl:
                        pl := 1
                        q := r >> k
                    RXOPEN := 0
                    RXCAND := 0
                    RXWIN := e and kp
                    RXIC := 0
                    nb := 1
                    rx_gear()
                    rx_bit(b)
            elif RXRUN and e:
                RXWIN := 1
                if not pl:
                    pl := 1
                    q := (RXPH - (p >> 1)) >> k
            RXPH := RXPH + 128
    RXPH := RXPH - (0 if two else q)

rx_pull(r):
    d := RXPH - r
    if RGAIN == 0:
        RXPH := r
    elif RXG <= 3:
        RXPH := RXPH - (d >> 1)
    elif RGAIN == 1:
        RXPH := RXPH - (d >> 2)
    elif RGAIN == 2:
        RXPH := RXPH - (d >> 4)
    else:
        RXPH := RXPH - (d >> 5)

rx_gear():
    if RXG < 16:
        RXG := RXG + 1

rx_sample():
    v := bit(SYNC2, RPIN)
    d := 1 if v == RXLAST else 0
    RXLAST := v
    rx_bit(d if RDIFF else v)

rx_bit(b):
    b := b ^ RINV
    if RXVIO:
        if b == RXRV:
            RXEF := RXEF | 4
        rx_end()
    elif RXSYN and (SMODE == 1 or SMODE == 2) and RXRL != 0 and RXRL >= SN and (SMODE == 1 or RXRL >= 2):
        if b == RXRV:
            RXEF := RXEF | 2
            if SMODE == 2:
                rx_end()
            else:
                RXVIO := 1
        else:
            rx_run_len(b)
    elif not RXSYN:
        if SMODE == 1 or SMODE == 2:
            rx_run_len(b)
        if RMSB:
            RXSR := ((RXSR << 1) | b) & 0xFFFF
            ok := ((RXSR ^ SYNC) & ((1 << SYNCLEN) - 1)) == 0
        else:
            RXSR := (RXSR >> 1) | (b << 15)
            ok := ((RXSR ^ SYNC) & (0xFFFF ^ (0xFFFF >> SYNCLEN))) == 0
        if ok:
            RXSYN := 1
            RXSR := 0
    else:
        if SMODE == 1 or SMODE == 2:
            rx_run_len(b)
        if RXTOT == CSKIP and (CMODE == 1 or CMODE == 2):
            RXCOV := 1
        if RXCOV and not CTX and (CMODE == 1 or CMODE == 2):
            f := bit(CRC, 31)
            crc_bit(b)
            if CMODE == 2:
                b := b ^ f
        if RMSB:
            RXSR := ((RXSR << 1) | b) & 0xFFFF
        else:
            RXSR := RXSR | (b << RXN)
        RXN := RXN + 1
        RXTOT := (RXTOT + 1) & 2047
        if RCNT != 0 and RXTOT == RCNT:
            rx_end()
        elif RXN == 16:
            rx_deliver(RXSR, 16, 0)
            RXSR := 0
            RXN := 0
        rx_byte(b)

rx_deliver(v, n, last):
    if RXFULL:
        RXOVR := 1
    else:
        RXBUF := v
        RXBN := n
        RXBL := last
        RXBE := RXEF if last else 0
        RXFULL := 1

rx_end():
    RXRUN := 0
    if CMODE == 1 and not CTX and CRC == ((CHKH << 16) | CHKL):
        RXEF := RXEF | 1
    if RXSYN:
        RXEP := 1
    elif HUNT:
        RXHUNT := 1

rx_stop():
    RXARM := 0
    RXHUNT := 0
    RXRUN := 0
    RXEP := 0
    RXFULL := 0
    RXVIO := 0
```

What follows from the code:

- **Start.** Without HUNT, a reception begins at the commit. With HUNT, the commit starts the hunt, and a reception begins in the first later cycle where RPIN moves to HLVL in SYNC2. Either way, the start cycle *s* counts as a reference edge, and the reception keeps the decoder that RKIND named in cycle *s*, and with RKIND 3 the *p*, RGAIN and RACQ of that cycle too (RXP3, RXRG and RXRA). For the Manchester decoder, *s* is the first clock edge: it gives the first bit at once (no bit with RDIFF) and puts the next predicted clock edge one bit later. Timed samples and an outside clock take no sample in *s*, even if an active edge of RCLK reaches SYNC2 in that cycle: the first sample is FIRST cycles later, or at the next active edge. So with HUNT, a start bit whose edge reaches SYNC2 in the cycle of a clock edge loses that edge's bit (section 11.5).
- **Period.** *p* is RPER x 256 + RFRAC, the period in 256ths of a cycle. RPER 0 gives *p* = 256, one cycle, and ignores RFRAC, so a timed half then samples every cycle and RXPH can't run away. The Manchester decoder takes RPER above 255 as 255, so its *p* stays below 65536 and RXPH stays between -32768 and 49407 there. RKIND 3 takes RPER above 15 as 15, so its *p* stays below 4096, and RXPH stays at or above 128 - *p* with the window closed and 256 - *p* with it open, and below *p* + 128 (below 7*p*/4 - 128 with RGAIN 0): from -3967 to 7037. With a candidate, RXCE is from 256 - *p* to -1.
- **Timed samples.** RXPH counts down by 256 a cycle, and a sample is taken from SYNC2 in the cycle where it is below 256. Sample *j* is in cycle *s* + FIRST + *j* x RPER, plus the carries of RFRAC. The input and the start edge pass the same synchroniser, so, as in section 5.7, FIRST counts from the edge at the pad. With RESYNC, every edge of RPIN pulls the countdown toward FIRST, by the gain below. With RGAIN 0 that restarts it, so the next sample is FIRST cycles after the edge. If an edge and a sample fall in one cycle, the sample is taken first. With RACQ as well, only an edge to HLVL pulls, and the other edges still count for REND. That is CAN's rule: a CAN node resynchronizes only on recessive-to-dominant edges (Bosch CAN 2.0, part A, Bit Timing Requirements, the synchronization rules; ISO 11898-1 likewise), so with HLVL dominant a dominant bit that a transceiver, or two other transmitters out of step in arbitration, stretch at RPIN can't move the sample point. Timed samples read RESYNC, RGAIN, RACQ, FIRST, RPER and RFRAC as they are in each cycle.
- **Manchester decoder: phase.** RXPH is the time since the predicted clock edge and grows by 256 a cycle. Let *w* be *p*/4, or *p*/2 with RACQ and RGAIN 1 to 3 until RXG reaches 16. When RXPH reaches *p* - *w*, the window for the next clock edge opens and RXPH drops by *p*, so it counts from -*w* to the predicted clock edge at 0. The next window opens when RXPH reaches *p* - *w* again: *p* - *w* after the predicted clock edge, as the edges since have moved it.
- **Manchester decoder: the bit.** In an open window, an edge before the predicted clock edge is a candidate, and a later one replaces it. At the predicted clock edge, the first cycle with RXPH at 0 or more, a candidate gives the bit: the level of RPIN in that cycle. With no candidate, the first edge from there on gives the level after it. Either way the window closes. So a clock edge is taken however late it comes before the next window, and a late bit-boundary edge misleads the decoder only if the clock edge after it is late too. For IEEE 802.3 a rising clock edge is a 1, and RINV gives the other convention. With RDIFF the bit is RXWIN: 1 if an edge other than a clock edge came between the last two clock edges. That decodes biphase mark, whose clock edges are the bit starts, one bit late, and, with RINV, differential Manchester, whose clock edges are mid-bit.
- **Manchester decoder: correction.** Every edge pulls the phase toward the place it should have: an edge in the window toward the predicted clock edge, any other edge toward half a bit after it, where a bit-boundary edge falls. The start edge sets the phase alone.
- **Gain.** A pull moves RXPH by 2^-k of its error, rounded down. With RGAIN 0, k is 0 and every pull is a full correction. With RGAIN 1 to 3, k is 1 while RXG is 3 or less, that is up to and including the third reference edge after the start, and then 2, 4 or 5. So the first edges set the phase quickly, and later ones are averaged over about 2^k edges. Random Manchester data has about 1.5 edges per bit, so k = 4 averages over about 11 bits. A larger k filters more jitter and follows less offset between the sender's clock and ours.
- **Outside clock.** Each active edge of RCLK samples RPIN in the same cycle. Both pins pass the same synchroniser, so they stay aligned to within the one cycle of phase uncertainty in section 10, item 2. RCLK can be one of the unit's own shift outputs read back from its `uio` pad, as a controller that receives on its own clock does: each sample is then the data as it was at the clock's pad edge.
- **Manchester decoder on half cycles (RKIND 3).** `rx_manch3` takes the place of `rx_manch` and runs two half steps a cycle, half a cycle each: first on the edge between SYNC3 and SYNC2F, with SYNC2F's level after it, then on the edge between SYNC2F and SYNC2. So every window opens, every predicted clock edge falls and every edge is timed to the half cycle, and a pulse shorter than a cycle that SYNC2F caught is two edges, one in each half: a line receiver's skew between its rising and falling edges, up to 4 ns for the ISL3176E (Renesas FN6307), turns 802.3's shortest pulses at ±13.5 ns of jitter, 23 ns, into 19 ns ones, which can fall between two rising-edge samples. These rules differ from RKIND 1's, for 802.3's jitter (14.3.1.3.1, section 7.3). A candidate keeps its phase in RXCE and doesn't pull: a bit-boundary edge in the window that a clock edge then replaces moves nothing, so a pattern of early clock edges and late boundary edges can't drag the prediction onto the boundary edges. A candidate gives the bit only in the first half step at *p*/8 or more after the predicted clock edge, unless an edge comes first: then that edge gives the bit, and the candidate was not the clock edge. An edge in that half step itself, which may have come up to 10 ns after *p*/8, overrides only a candidate that came more than *p*/8 before the predicted clock edge (RXCE below -*p*/8, a bit-boundary edge of a pattern with boundary edges late and clock edges early); a later candidate keeps the bit, and the edge after it counts as an edge other than the clock edge. At RPER 2 that bound is 128, one half step, not *p*/8 (64 to 95): a clean clock edge can be seen up to a half step before a prediction that a full pull put on the sample after the last one, and the bit-boundary edge after it can then come in the deadline half step. Manchester never has two edges other than clock edges in a row, so after one (RXWIN set, and no candidate) the next edge is the clock edge: it gives the bit at once, in the window or at most *p*/8 before it opens, and before the window it uses up that prediction (RXPH less *p*), unless the cycle has already given a bit. So the clock edges of a run of equal bits can come up to 27 ns early against those of alternating bits, ±13.5 ns each way, which is how a cable's low-pass shape moves them. An edge earlier than that counts as an edge other than the clock edge. A cycle with an edge in each half pulls nothing: the two edges of a pulse shorter than a cycle are a clock edge and a bit-boundary edge, and with only the first one's pull a run of such pulses would drag the prediction onto one kind of edge. And the half steps decide on RXPH as it was at the start of the cycle, plus 128 in the second, less *p* for a window opened: the cycle's one pull, the first half step's if both would pull, is applied at the end of the cycle, by the gain of RXG as it was before that half step's bit, and the window width is the one RXG gives at the start of the cycle. A candidate that gives the bit pulls then, by the error it had when it came. With RDIFF a candidate that a later edge replaces counts as an edge other than the clock edge, before or after the predicted clock edge. With HUNT, a start edge in the second half starts RXPH at 128, not 256. RPER 0 and 1 act as 2 and RPER above 15 as 15, so in any cycle there is at most one window opening and one bit, and the arithmetic stays narrow (the Period bullet). A test that gives one input level per cycle puts every edge in the first half (section 3.1).
- **Sync.** Until the pattern is found, bits shift through RXSR as a history and no word builds. The pattern is laid out as the received word it would form. With RMSB = 0 it sits in the top SYNCLEN bits: the 802.3 start frame delimiter alone is 0xD500 with SYNCLEN 8, and the last preamble byte with the delimiter is 0xD555 with SYNCLEN 16. With RMSB = 1 it sits in the bottom SYNCLEN bits. The bit that completes the pattern isn't kept. The next bit starts the first word.
- **Words.** A word of *n* bits holds them in bits *n* - 1..0, with higher bits 0. Every 16 bits make a word. If the buffer is still full, the new word is lost and RXOVR sets. A word that completes in the same cycle as an `rxw` counts as lost, because the buffer was full at the start of the cycle.
- **End.** A reception ends after its RCNT-th bit, after REND cycles with no reference edge (for the Manchester decoder, with no bit), when a Manchester window closes with no edge in it, one bit after a stuff violation with SMODE 1, at the violating bit with SMODE 2, at a timed sample that sees the end test with REOP (both in 5.9.7), or at `sstop rx`. A Manchester window closes empty in the cycle the next window would open: 7/4 bits after the last predicted clock edge, or 3/2 bits while RACQ's wide window is in use. With RKIND 3 and REOP that end drops the 1 to 7 bits after the last byte boundary (5.9.7, the byte end). A reception that found its pattern and ended any way but `sstop rx` then gives exactly one word marked last, holding the bits since the last full word, less any that the byte end drops (5.9.7): 0 to 15, or up to 16 when RCNT ends it. That word is never lost: it waits in RXSR, with RXEP set, until the buffer is empty, and only then does a HUNT receive half start hunting again. A reception that never found its pattern gives no word, and only RXSEEN shows it. `sstop rx` drops the reception, its last word and the buffer.
- **`sgo rx` while busy** stays armed, and status bit 15 shows it. `rx_step` takes the arm only when the half is idle: no reception, no hunt and no last word waiting. Without HUNT, the first commit after the reception ends starts the next one. With HUNT the half hunts again after every reception, so the arm waits until `sstop rx` clears it. If HUNT is cleared first, the first commit after the half goes idle starts a reception, whatever that commit was for.
- **Scope of the Manchester decoder.** It receives Manchester-family codes of up to 255 cycles per bit, down to about 195 kbit/s at 50 MHz, or with RKIND 3, where timing to the half cycle matters, up to 15 cycles and 255/256 per bit, down to about 3.1 Mbit/s, that mark a frame only by its start and its end. A frame ends at the first bit with no clock edge, and no status bit reports a code violation. So codes that frame by violations can't be received with it: AES3 and S/PDIF preambles, the MIL-STD-1553 sync, Token Ring's J and K. Those, and slower Manchester codes, are left to timed samples and firmware.
- **Exact.** Every decision falls on a cycle that RXPH, the synchroniser and whole 256ths fix, and with RKIND 3 on a half cycle that SYNC2F fixes too, so two simulators agree on every cycle.

**Implementation note.** This is not normative. The code fixes what happens in each cycle, not how. Written out literally, `rx_manch` chains about five carry chains in one cycle, too many for the slow corner at 50 MHz. The structure below gives the same result in every cycle with at most two:

- **Flops for values that depend only on configuration.** An implementation may hold such a value in flops of its own. Here they are *p* - *p*/4 and *p* - *p*/2, the two places where a Manchester window opens, 16 bits each. They are loaded in the cycle of the `scfg` that writes RXPER or RXOPT, from the value being written and the other slot's stored value, and reset to their values for the reset configuration, where *p* is 256: 192 and 128. The unit reads configuration as it was at the start of a cycle (section 3.2), and `scfg` writes at the end of its cycle, so in every cycle these flops match the configuration the unit reads, and no program can tell them apart from the code. In particular, an `scfg` in the same cycle as a receive start or a sample changes neither: in that cycle the start and the sample use the old configuration, in the code and in the flops alike, and from the next cycle both use the new one.
- **Manchester decoder.** Form RXPH - *p* and RXPH - *p*/2 beside the window compare. When the compare and the sign are known, select the in-window or the boundary error, then the gain's fixed wiring, and fold the + 256 into the one add that follows. *p* itself needs no flop: it is RPER and RFRAC with the rules for 0 and for 256 and up. The arithmetic fits 17 bits (the Period bullet), sign-extended into RXPH, because RXKIND keeps a timed RXPH out of it.
- **Timed samples.** Form the error, RXPH - 256 + *p* (when sampling) minus FIRST's target, beside the countdown, then select the gain's wiring and subtract: two carry chains on 25 bits.
- **Cost.** 32 flops per unit for the two window points, 128 for four units. RXKIND's 2 flops are in the state table.
- **RKIND 3.** The half steps read RXPH as it was at the start of the cycle and pull once at its end, so every compare is RXPH against a value from flops, side by side: the window points and RXPH + 128 against them, 0 and *p*/8 and RXPH + 128 against them, RXPH against *p* and RXPH + 128 - *p* against 0 and *p*/8. So is every error a pull can take: RXPH, RXPH - *p*, RXPH + 128 - *p*, RXPH - *p*/2, RXPH + 128, RXPH + 128 - *p*/2, and RXCE, a flop. Then the choice of one error, written flat as a sum of products of those compares and the flops (the half steps' decisions in sequence are too deep), the gain's fixed wiring, and one subtract from RXPH + 256 or RXPH + 256 - *p*, the base chosen by the one compare RXPH + 128 against the window point: two carry chains. With RPER above 15 as 15, *p* is below 4096 and RXPH stays from -3967 to 7037 (the Period bullet), so the arithmetic is 15 bits and RXCE 13. *p* is RXP3, which `rx_begin` sets (512 for RPER 0 and 1), and RKIND 3's own window points *p* - *p*/4 and *p* - *p*/2 (384 and 256 for RPER below 2; PWQ and PWH stay RKIND 1's) are held in flops of 12 bits loaded with it in the cycle a reception starts, from the configuration that cycle reads. RXPH's next value is one AND-OR of every decoder's result with selects from flops, so RKIND 3's enters two gates from the flop. Cost: RXCE's 13 flops per unit, and 39 for *p*, its window points, RXRG and RXRA. The structural rule's bound (RXPH + *p*/8 less the window point, one sum of three flop values), the keep test (RXCE plus *p*/8, or plus 128 when *p* is below 768: one sum of two) and the cycle with two edges add decision terms only: every error a pull can take is still one of the seven, written flat by whether the window opens in the first half, was open, or is closed, so the arithmetic stays two carry chains.
- **If the real flow misses 60 MHz: decide the window opening a cycle early.** Two more flops per unit, one per window point, may hold whether RXPH at the start of the next cycle will be at or above *p* - *p*/4, and at or above *p* - *p*/2. Each is formed in the cycle before, beside RXPH's own update and from the same operands (RXPH or RXPH - *p*, the selected correction and the + 256), by a carry-save compare against its window point. In the cycle of an `scfg` that writes RXPER or RXOPT the compare uses the window points being loaded, and in the cycle of `rx_begin` it compares 256. `rx_manch` then takes `RXPH >= p - w` from one of the two flops, picked by RACQ, RGAIN and RXG < 16, which are known at the start of the cycle. That takes the window compare off the front of the longest path, for 2 flops and two 17-bit compare trees per unit. It gives the same result in every cycle. It is written down so that it is ready; it was not synthesized or simulated.

#### 5.9.7 CRC and stuffing

Each unit has one CRC register of 32 bits, and a bit stuffer with its destuffer. Both work on bits, between a half's words and its line code:

- **Send:** a data bit goes to the CRC, or is scrambled by the LFSR; then the stuffer counts it and may add a stuff bit after it; then the line coder of 5.9.5 sends each bit, stuff bits and appended CRC bits included.
- **Receive:** the decoder of 5.9.6 gives a bit; RINV inverts it if set; the destuffer drops stuff bits and finds violations; then the CRC takes the bit, or the LFSR descrambles it; then the bit goes into the word.

So the CRC is over unstuffed bits, in wire order, and the stuffer is over the bits as they go on the line. The register serves one half at a time: the send half with CTX 1, the receive half with CTX 0.

**The register.** A CRC of width *w* = CW + 1 sits in the top *w* bits of CRC, the coefficient of x^(*w*-1) in bit 31. POLY, which the code writes `(POLYH << 16) | POLYL`, is left-aligned the same way, without its x^*w* term, so its bits below 32 - *w* are 0. `crc_bit(b)` takes one bit: the feedback is bit 31 XOR *b* (bit 31 alone for the LFSR), the register shifts left by one, and POLY is XORed in if the feedback is 1. The bits below 32 - *w* start at 0 and stay 0, so every width uses the same feedback bit and the same one-bit shift, and nothing shifts by the width.

**Wire order.** The register takes bits in the order they are on the wire, so the unit has no reflection setting. A catalogue CRC with reflected input and output, such as Ethernet's, USB's, HDLC's and 1-Wire's, is defined on bytes sent LSB first, and the register fed the same bits in wire order holds the same remainder, unreflected. An appended CRC goes out bit 31 first. That is the order each standard below puts its CRC on the wire: 802.3 sends the x^31 term first, USB sends its CRC MSb first, HDLC the highest term first, CAN its CRC MSB first, and 1-Wire's CRC byte, reflected and sent LSB first, comes out in the same order.

**Coverage on send** is set per word, by the kind that `txw` gives it:

| Kind | The word's bits |
|---|---|
| None, 0 | Sent as they are: not in the CRC, not stuffed. Each sets the stuffer's run to 0 |
| `stuff`, 1 | Stuffed. Not in the CRC |
| `crc`, 2 | In the CRC, or scrambled by the LFSR, and stuffed |
| `fcs`, 3 | As `crc`, and the CRC is appended after the word's last bit |

Stuffing needs SMODE 1 or 2, and the CRC needs CMODE 1 or 2 with CTX 1. Otherwise a kind has no effect, so every version 2 program sends as it did. The register starts from its initial value at each send's start, and again when an append ends, so coverage starts at the first word of kind 2 or 3 after either, and ends at a word of kind 3. One send can therefore hold several frames, each ending in a word of kind 3. A word of 1 to 16 bits can start or end anywhere, so any field boundary can be a word boundary. Examples:

- **Ethernet:** the preamble and SFD words have no kind, the frame words from the destination address on are `crc`, and its last word is `fcs` and `last`. The tail is TP_IDL.
- **USB:** the SYNC and PID words are `stuff`, because USB stuffs from the SYNC on; the token or data fields are `crc`, and the last is `fcs` and `last`. The EOP is the tail.
- **HDLC:** the opening flag has no kind; address, control and information are `crc`, and the last is `fcs`; the closing flag has no kind and is `last`.
- **CAN:** SOF to the end of the data field are `crc`, the last `fcs`; the CRC delimiter, ACK slot and EOF, 10 recessive bits that CAN doesn't stuff, are the tail (ENDA recessive) only up to 25 cycles per bit, since TAIL holds at most 255 cycles. At slower rates they are the idle level (IDLEA recessive) with TAIL 0, or a word of no kind.

**Coverage on receive** is set by CSKIP. The register starts from its initial value at the reception's start, and takes the bits from bit CSKIP after the sync pattern, counting from 0, to the end of the reception. The sync pattern is never covered, so the CRC leaves out Ethernet's preamble and SFD, USB's SYNC and an HDLC opening flag, and CSKIP leaves out the bits after it that the CRC doesn't cover, such as USB's 8-bit PID. CAN has no sync pattern (SYNCLEN 0), so its CRC starts at SOF with CSKIP 0. The CRC ends where the reception ends, which 5.9.6 defines: the Manchester end for Ethernet, the end test below for USB, the closing flag for HDLC, and RCNT for CAN, which firmware writes once the DLC has arrived. For CAN, RCNT can end the reception at the CRC's last bit, with CHK 0, or one bit later at the CRC delimiter, with CHK 0x8B320000, the good residue fed the delimiter's 1. The second takes a stuff bit after the CRC into the reception, where with HUNT it would start a short one, puts the ACK slot always in the next bit, and fails a dominant delimiter.

**Exact.** A bit enters the register in the cycle its last symbol ends on send, and in the cycle it is decoded on receive. Stuff bits and appended bits take a bit time each on the same grid, so the line carries every bit on a cycle the configuration and the data fix.

**The append.** After the last bit of a word of kind `fcs`, and any stuff bit it needs, the send half sends *w* more bits, each bit 31 of the register XOR CXOR, with the word's line code. After each one the register shifts left by one, with no feedback. When the last one ends, the register takes its initial value again, so the next frame of the same send starts its CRC from it. The stuffer counts the appended bits and adds any stuff bit they need. Then the send goes on as at any word end: the next word, the tail, or an underrun. The append uses the register only while CTX is 1: if CTX moves to 0 during it, the remaining appended bits are CXOR and the register is left to the receive half (section 10, item 17).

**The check.** At the end of a reception, with CMODE 1 and CTX 0, end flag 0 sets if CRC equals CHK, which the code writes `(CHKH << 16) | CHKL`. A good reception leaves a residue that doesn't depend on the data or the initial value: 0 when the sender's CXOR is 0, and with CXOR 1 the register that *w* ones give from 0. The table below has CHK for each standard. HDLC's check includes the first six bits of the closing flag, 011111, which reach the CRC before the destuffer knows they belong to a flag, so its CHK is its residue 0x1D0F fed those six bits: 0xD0F9. The check covers every covered bit up to the end, so a bit the protocol lets a sender add before its end marker makes a good packet fail, unless a byte end below drops it, as they drop USB's dribble and the excess bits of an 802.3 frame.

**Parity.** A width of 1 with POLY *x* + 1, CW 0 and POLYH 0x8000, and CINIT 0, makes the register the running XOR of the covered bits. Then the append is a parity bit, even with CXOR 0 and odd with CXOR 1, and the check is a parity check, with CHK 0 for even parity and 0x80000000 for odd. CINIT 1 starts the register at 1, which swaps even and odd. A receiver that covers the stop bit as well, to check parity and framing with one residue, moves CHK by the stop bit's 1: 0x80000000 for even parity and 0 for odd.

**Six standards.** CHK is used on receive only. The check value is the CRC of the nine ASCII bytes `123456789`, sent as the unit appends it and read in the standard's own bit order; each is the catalogue's value.

| Standard | *w* | POLYH, POLYL | CINIT, CXOR | CHKH, CHKL | Bits | CODING, send and receive | Check value |
|---|---|---|---|---|---|---|---|
| Ethernet FCS, CRC-32 | 32 | 0x04C1, 0x1DB7 | 1, 1 | 0xC704, 0xDD7B | LSB first | 0x02FF, 0x00FF | 0xCBF43926 |
| USB data, CRC-16 | 16 | 0x8005, 0 | 1, 1 | 0x800D, 0 | LSB first | 0xE6EF, 0xE4EF | 0xB4C8 |
| USB token, CRC-5 | 5 | 0x2800, 0 | 1, 1 | 0x6000, 0 | LSB first | 0xE6E4, 0xE4E4 | 0x19 |
| HDLC FCS-16 (X.25, IBM-SDLC) | 16 | 0x1021, 0 | 1, 1 | 0xD0F9, 0 | LSB first | 0x56EF, 0x54EF | 0x906E |
| CAN, CRC-15 | 15 | 0x8B32, 0 | 0, 0 | 0, 0 | MSB first | 0x5A8E, 0x588E | 0x059E |
| 1-Wire, CRC-8 (MAXIM-DOW) | 8 | 0x3100, 0 | 0, 0 | 0, 0 | LSB first | 0x0287, 0x0087 | 0xA1 |

The CODING values set CMODE 1 and CTX, and the stuffer: ones with N = 6 for USB, ones with N = 5 for HDLC, equal bits with N = 5 for CAN. USB's values also set REOP. Only the receive half reads it, so the send value keeps it set: a receive half that hunts on the same pins during the send then ends its reception of that packet at the EOP (section 11.5). One configuration checks one CRC, so a USB device rewrites CODING, POLYHI and CHKHI between a token and a data packet.

For 1-Wire, the master starts every time slot, read slots included. If its send half makes the slots and its receive half samples them, the register serves the receive half, with the receive value. The send value is for a unit that sends the CRC, as a device does at the end of its ROM code, and for an idle send half used as a CRC engine (below).

**End flags.** Three flags travel with a reception's last word. End flag *k* is bit *k* of the value `srd Rd, end` reads:

| Bit | Set when |
|---|---|
| 0 | The CRC check passed: CMODE 1, CTX 0 and CRC equal to CHK at the end |
| 1 | A stuff violation ended the reception |
| 2 | The bit after the violation continued the run. Only SMODE 1 sets it: with SMODE 2 the violating bit ends the reception |

They form in RXEF during the reception, move to RXBE with the last word, and to RXPE when `rxw` takes it, so `srd Rd, end` gives the end flags of the word `rxw` took last: a last word's flags, or 0 for a full word. Read them after the `rxw` that sets T and before the next `rxw`.

**Stuffing.** SMODE picks the run and SN its length N. With SMODE 2, an SN of 0 or 1 acts as 2: after a stuff bit the run is 1, so with N = 1 the receiver would take the next bit for a stuff position where the sender puts a data bit. The run is the number of bits in a row, up to 7, that equal the last bit; with SMODE 1 only ones count, and a 0 makes it 0.

- **Send.** After a bit that is stuffed (a word of kind 1 to 3, or an appended CRC bit) and makes the run N, a stuff bit follows: the complement of the run's value, so a 0 after ones. A stuff bit is not in the CRC, but it counts in the run: with SMODE 2 it starts a run of 1, as CAN counts it, and with SMODE 1 it ends the run of ones. A stuff bit never calls for another. A stuff bit due after a word's last bit is sent before the CRC, the next word, the tail or the end, as USB, HDLC and CAN require.
- **Receive.** The destuffer counts the run from the reception's start, the sync search included, because USB counts its SYNC's last 1 in the first run. It acts only once the sync pattern is found, so an HDLC opening flag, which is the sync pattern, is no violation. Then the bit after a run of N is the stuff position. The complement there is a stuff bit, and is dropped. The run's own value there is a stuff violation: that bit is dropped too, and end flag 1 sets. With SMODE 1 the next bit ends the reception, setting end flag 2 if it continues the run, and neither of those two bits goes to the CRC or into a word. With SMODE 2 the violating bit itself ends the reception. The bit count of words and RCNT count only the bits that remain.

What a violation means, per standard:

- **USB** (SMODE 1, N = 6): 7 ones is a bit-stuff error. The reception ends one bit later with end flag 1 set; drop the packet. A good packet ends at the end test.
- **CAN** (SMODE 2, N = 5): 6 equal bits is a stuff error. The reception ends at the sixth bit with end flag 1 set, so firmware can start its error flag in the next bit, where a CAN node starts it after a stuff error (Bosch CAN 2.0, part A, Error Signalling; ISO 11898-1 likewise). An error frame, if the node sends one, is firmware's.
- **HDLC** (SMODE 1, N = 5): 6 ones is a flag or an abort, and the next bit tells which. A 0 makes it a flag: the frame ended normally, with flag 1 set and flag 2 clear, and its last word holds the flag's first six bits, 011111, after the FCS, which firmware drops. A 1 makes it an abort, with flags 1 and 2 set: drop the frame.
- **HDLC frames one after another.** The closing flag ends the reception at its last bit. With HUNT the half then hunts for an edge, and a new reception needs a whole flag, its sync pattern, after that edge. So a run of flags gives one reception for every few flags, not one per flag. With a falling-edge hunt and the sync pattern 0x7E00 with SYNCLEN 8, the next falling edge is the next flag's last bit, and each reception uses three flags: the one that closes it, the one the hunt starts in, and the one that opens the next. A frame is then received only if its opening flag falls on that cycle, as when 3, 6 or 9 flags separate it from the frame before; with 1, 2, 4 or 5 flags it is lost. A sender that idles on flags gives a six-bit reception (011111, end flag 1) for every three flags, and its next frame arrives only if the idle flags end on the cycle. With SYNCLEN 7 and a rising-edge hunt the cycle is two flags. Frames that share one flag never both arrive, and a frame after a gap of ones (mark idle) always starts a reception.

**The end test.** With REOP, a timed sample at which RPIN and RCLK both read 0 in SYNC2 ends the reception, and that sample gives no bit. For a USB receiver RPIN is D+ and RCLK is D-, so the test sees the SE0 that starts an end of packet, and the CRC ends at the packet's last bit. Only timed samples (RKIND 0) test it.

**The byte end.** USB lets the last bit before an end of packet stretch by up to six bit times with no transition, which a receiver must accept (dribble, USB 2.0 7.1.9.1). The decoder reads it as ones after the packet's last whole byte. So the receive half keeps copies at every byte boundary: `rx_byte` sets RXSR8, RXN8 and RXC8 to RXSR, RXN and the CRC register at the start, which is the state the sync pattern leaves, and after every 8th bit after the sync pattern, and RXONES says whether every bit since is a 1. When the end test ends a reception with 1 to 6 bits after the last byte boundary, all of them ones, and no stuff violation, `rx_eop` puts RXSR, RXN and the register's top half back to the copies before `rx_end` checks the CRC. So the last word holds whole bytes, and end flag 0 judges the CRC of the whole bytes. A stuff violation keeps every bit and end flag 1: it takes seven 1s in a row, a bit-stuffing error (USB 2.0 7.1.9), one more than the six bit times with no transition that 7.1.9.1 asks a receiver to accept. Otherwise the end test ends the reception as before: with 7 bits after the boundary, or a 0 among them, every bit stays, and a packet that isn't whole bytes fails its check. The bits counted are the ones the destuffer keeps, so a stuff bit is never one of them. The end test puts back only the register's top 16 bits, which hold every CRC of up to 16 bits, USB's included; for a wider CRC it leaves the low half as it was at the end (section 10). The copies are taken whether or not REOP is set, and only the two byte ends use them.

**The byte end of a Manchester frame.** IEEE 802.3's MAC truncates a received frame to whole octets before it checks the FCS: 4.2.9's ReceiveLinkMgmt sets excessBits := frameSize mod 8 and frameSize := frameSize - excessBits, and ReceiveDataDecap then judges the FCS of what is left, reporting an alignmentError if it fails and excessBits isn't 0 (4.2.4.2.1 b). Excess bits come at a frame's end: a sender's extra bits, or TP_IDL's first transition, after a last bit whose edges came late, landing in the next window as one more clock edge (section 7.3). So with RKIND 3 and REOP, when a window closes empty and ends a reception with 1 to 7 bits after the last byte boundary and no stuff violation, `rx_mend` puts RXSR, RXN and the whole CRC register back to the copies before `rx_end` checks the CRC: the last word holds whole octets, and end flag 0 judges the FCS of those octets, as 802.3 does. With no bits after the boundary every bit stays, and so it does with REOP 0, for codes whose frames need not be whole octets. Only that end drops bits: RCNT, REND, a stuff violation and `sstop rx` end a reception of RKIND 3 as before. RKIND 1 has no byte end. For 802.3 the receive value of CODING is then 0x80FF.

**LFSR mode** (CMODE 2). The register runs as a Galois LFSR of POLY: each covered bit advances it with bit 31 alone as the feedback. On send, a covered data bit goes out XOR bit 31; on receive, a covered bit is XORed with bit 31 before it goes into the word. A sender and a receiver that start from the same initial value, at a send's first covered bit and a reception's first covered bit, undo each other: an additive scrambler. Words of zeros send the LFSR's own sequence, a PRBS, whose period is 2^*w* - 1 bits for a primitive POLY of width *w*. The LFSR needs CINIT 1: from 0 it stays at 0. Kind `fcs` acts as `crc`, and nothing is appended or checked. Self-synchronizing scramblers, which feed back the line bits, aren't offered.

**POLY for a standard's PRBS or scrambler.** POLY, with its *x*^*w* term, is the characteristic polynomial of the sequence the line carries: bit *n* + *w* is the XOR of the bits *n* + *i* for every lower term *x*^*i* of POLY. Standards define their PRBS generators and additive scramblers as a Fibonacci register named by the polynomial *x*^*w* + *x*^*m* + 1, whose stages *m* and *w* feed back, so bit *n* is the XOR of bits *n* - *m* and *n* - *w*. That sequence's characteristic polynomial is the reciprocal, *x*^*w* + *x*^(*w* - *m*) + 1, and that is the POLY to set; the named polynomial set as POLY sends the same sequence in reverse time, which a standard's checker or descrambler can't lock to. So PRBS7 (*x*^7 + *x*^6 + 1) is POLYHI 0x0600, PRBS9 (*x*^9 + *x*^5 + 1) 0x0880, PRBS15 (*x*^15 + *x*^14 + 1) 0x0006, and the 802.11 scrambler (*x*^7 + *x*^4 + 1) 0x1200, each with POLYLO 0.

**An idle send half as a CRC engine.** With no pin switched to its outputs, a send at TPER 0 takes one bit per cycle: 16 cycles for a word of kind `crc`, against 8 slots, 32 cycles, per bit for `crc32.s`. Firmware reads the register with `srd` once status bit 5 is clear. A word lasts 16 cycles at TPER 0, so a feed loop of 4 slots keeps up only from the second word: queue two words before the start, or the first refill misses its word boundary, the send underruns and the next start restarts the register. For 1-Wire's ROM code, it pushes the 64 bits and checks that both halves read 0. Use words of kind `crc` only: a word of kind `fcs` appends the CRC and then loads the initial value, so the register no longer holds the CRC.

```
crc_init():
    if CINIT:
        CRC := 0xFFFFFFFF ^ (0xFFFFFFFF >> (CW + 1))
    else:
        CRC := 0

crc_bit(b):
    f := bit(CRC, 31) ^ b if CMODE == 1 else bit(CRC, 31)
    CRC := (CRC << 1) & 0xFFFFFFFF
    if f:
        CRC := CRC ^ ((POLYH << 16) | POLYL)

tx_begin():
    TXSTF := 0
    TXAN := 0
    TXRL := 0
    if CTX and (CMODE == 1 or CMODE == 2):
        crc_init()

tx_done():
    if TXSTF:
        TXSTF := 0
        tx_run_len(TXB)
    elif TXAN != 0:
        if CTX:
            CRC := (CRC << 1) & 0xFFFFFFFF
        TXAN := TXAN - 1
        if TXAN == 0 and CTX and (CMODE == 1 or CMODE == 2):
            crc_init()
        tx_stuff(TXB)
    else:
        if TXK >= 2 and CTX and (CMODE == 1 or CMODE == 2):
            crc_bit(TXB)
        if TXK >= 1:
            tx_stuff(TXB)
        else:
            TXRL := 0
        TXN := TXN - 1
        if not TMSB:
            TXSR := TXSR >> 1
        if TXN == 0 and TXK == 3 and CTX and CMODE == 1:
            TXAN := CW + 1

tx_stuff(v):
    if SMODE == 1 or SMODE == 2:
        tx_run_len(v)
        if TXRL != 0 and TXRL >= SN and (SMODE == 1 or TXRL >= 2):
            TXSTF := 1

tx_run_len(v):
    if SMODE == 1 and v == 0:
        TXRL := 0
    elif TXRL == 0 or v != TXRV:
        TXRL := 1
    elif TXRL < 7:
        TXRL := TXRL + 1
    TXRV := v

rx_cbegin():
    RXEF := 0
    RXVIO := 0
    RXCOV := 0
    RXRL := 0
    if not CTX and (CMODE == 1 or CMODE == 2):
        crc_init()
    rx_byte(1)

rx_byte(b):
    if (RXTOT & 7) == 0:
        RXSR8 := RXSR
        RXN8 := RXN
        if not CTX and (CMODE == 1 or CMODE == 2):
            RXC8 := CRC
        RXONES := 1
    elif not b:
        RXONES := 0

rx_eop():
    nb := RXTOT & 7
    if RXONES and not RXVIO and nb != 0 and nb != 7:
        RXSR := RXSR8
        RXN := RXN8
        if not CTX and (CMODE == 1 or CMODE == 2):
            CRC := (RXC8 & 0xFFFF0000) | (CRC & 0xFFFF)
    rx_end()

rx_mend():
    if REOP and not RXVIO and (RXTOT & 7) != 0:
        RXSR := RXSR8
        RXN := RXN8
        if not CTX and (CMODE == 1 or CMODE == 2):
            CRC := RXC8
    rx_end()

rx_run_len(v):
    if SMODE == 1 and v == 0:
        RXRL := 0
    elif RXRL == 0 or v != RXRV:
        RXRL := 1
    elif RXRL < 7:
        RXRL := RXRL + 1
    RXRV := v
```

`tx_done` runs at the end of every bit of a send (5.9.5), and `rx_bit` (5.9.6) holds the destuffer's steps and the receive half's calls of `crc_bit` and `rx_byte`. `rx_byte` takes the bit as it goes into the word.

**Implementation note.** This is not normative. On receive the destuffer's decision adds about two gates between a decoded bit and the word insert, because whether a bit is at the stuff position depends only on flops (RXRL, RXSYN, SMODE, SN). The CRC's next value runs beside the word path and doesn't lengthen it, and in LFSR mode the descrambling adds one XOR. On send the choice of a bit's source, data, stuff or CRC, adds a 3-to-1 multiplexer in front of the output tables. Only the half that CTX names uses the register in a cycle (section 3.2), so the register's next value is the send half's or the receive half's, chosen by CTX. With SMODE 2 a stuff violation ends the reception in the cycle of its bit, as RCNT does, from the same decision. The byte end of a Manchester frame adds RXC8's low 16 bits, 16 flops a unit, and one more choice into RXSR, RXN and the register, selected by RKIND 3's empty window with REOP, RXVIO and RXTOT's low three bits. In that cycle RKIND 3 gives no bit (the window had no edge, and the reception ends in the half step that finds it empty), so the choice never meets a bit's insert, and end flag 0 compares CHK with RXC8 or with the register, both flops.

## 6. Instruction set

### 6.1 Notation

Every instruction is one 16-bit word.

- **Encoding** has 16 characters, bit 15 first, grouped by spaces for reading. `0` and `1` are fixed. A letter is an operand field. There are no don't-care bits: a word that matches no row, or has an illegal value in a field, is illegal (section 10).
- **Fields** gives each letter's operand type.
- **Slots** is the time taken with nothing to wait for. `1+` means it blocks until it can proceed.
- **W** is `y` for a world instruction, `n` for not, and `pin` for one that is a world instruction only if a pin field names a pin (section 3.6).
- **Semantics** is statements separated by `;`, run in order, each seeing the results of the ones before.

Semantics use Python expression syntax on unbounded integers. `~x` and a subtraction can go negative, and `&` and `>>` then treat the value as two's complement. `:=` assigns. An assignment to a register or D truncates to 16 bits, an assignment to PC is taken modulo DEPTH, and T takes 0 or 1. `if c: s` governs exactly one statement. `Rx` is the register named by field `x`. `f` is a temporary. `PC` reads as this instruction's address.

| Helper | Meaning |
|---|---|
| `bit(x, n)` | `(x >> n) & 1` |
| `need(c)` | If `c` is false, this instruction does nothing, and it runs again next slot |
| `TB`, `TID`, `DEPTH` | The timebase in this cycle, this thread's number, the memory depth |
| `load(d, A)`, `store(V, A)` | Section 4.2. Both hold PC for the second slot |
| `arm`, `now`, `wait` | Section 3.4 |
| `shadow`, `shadow_oe`, `put8`, `oe8` | Section 5.4 |
| `pcfg_in`, `pcfg_pp`, `pcfg_od`, `pcfg_byp` | Section 5.3 |
| `tx_push`, `rx_pop`, `sstat`, `srd`, `set_cfg`, `sgo`, `sstop`, `shadow_sel` | Section 5.9.4 |
| `halt` | The code block below |

```
halt():
    PC := next
    stop()
```

### 6.2 Operand types

| Type | Written as | Legal field values |
|---|---|---|
| `reg` | `R0`..`R7` | 0..7 |
| `reg4` | `R0`..`R3` | 0..3 |
| `u1`, `u3`, `u4`, `u5`, `u8`, `u12` | Number or expression | All. The assembler checks the range |
| `addr` | Label or expression | All. The assembler rejects a value of DEPTH or more |
| `opin` | `uo0`..`uo7` except `uo3`, `io0`..`io7` | All but 3 |
| `opin-` | As `opin`, or `-` | All. 3 means no pin |
| `ipin` | `ui0`..`ui7`, `io0`..`io7` | All |
| `ipin-` | As `ipin` except `ui0`, or `-` | All. 0 means no pin |
| `iopin` | `io0`..`io7` | 8..15 |
| `lane` | `uo0`, `uo4`..`uo7` | 0, 4, 5, 6, 7 |
| `obank` | `uo`, `io` | 0, 1 |
| `ibank` | `ui`, `io` | 0, 1 |
| `wmode` | `eq`, `ne`, `chg` | 0, 1, 2 |
| `eact` | `0`, `1`, `T`, `!T` | 0..3, in that order |
| `flag` | The word `inbox`, or nothing | 0, 1 |
| `nbits` | `1`..`16` | All. The field holds *n* mod 16, so 0 means 16 |
| `lflag` | The word `last`, or nothing | 0, 1 |
| `sslot` | `TXMODE`, `TXMAP`, `TXPER`, `TXTAIL`, `RXMODE`, `RXPER`, `RXFIRST`, `RXEND`, `RXCNT`, `RXSYNC`, `RXOPT`, `POLYHI`, `POLYLO`, `CHKHI`, `CHKLO`, `CODING` | 0..15, in that order |
| `sdir` | `tx`, `rx`, `txrx` | 1, 2, 3 |
| `sout` | `-`, `A`, `B` | 0, 1, 2 |
| `tkind` | The word `stuff`, `crc` or `fcs`, or nothing | 0..3: nothing is 0, then in that order |
| `sreg` | `lo`, `hi`, `end` | 0, 1, 2 |

Where the syntax shows a literal `0` or `1` as a pin value, in `put p, 0` or an `eact`, a constant expression with that value also works. So `edge SCK, CPOL, R2` is legal.

### 6.3 Instruction table

This table is the single source. Every row has the same six columns, so it converts directly to YAML or JSON. The assemblers and all three simulators read it from this file.

<!-- isa-table-begin -->
| Syntax | Encoding | Fields | Slots | W | Semantics |
|---|---|---|---|---|---|
| `idle` | `0000 00000 000 0000` | | 1 | n | `pass` |
| `halt` | `0000 00001 000 0000` | | 1 | y | `halt()` |
| `now` | `0000 00010 000 0000` | | 1 | y | `now()` |
| `clrt` | `0000 00011 000 0000` | | 1 | n | `T := 0` |
| `sett` | `0000 00100 000 0000` | | 1 | n | `T := 1` |
| `nott` | `0000 00101 000 0000` | | 1 | n | `T := 1 - T` |
| `tlate` | `0000 00110 000 0000` | | 1 | y | `T := LATE; LATE := 0` |
| `jr Rn` | `0000 00111 nnn 0000` | `n:reg` | 1 | n | `PC := Rn` |
| `swap Rn` | `0000 01000 nnn 0000` | `n:reg` | 1 | n | `Rn := (Rn >> 8) \| (Rn << 8)` |
| `not Rn` | `0000 01001 nnn 0000` | `n:reg` | 1 | n | `Rn := ~Rn` |
| `rds Rn` | `0000 01010 nnn 0000` | `n:reg` | 1 | y | `Rn := S` |
| `rdd Rn` | `0000 01011 nnn 0000` | `n:reg` | 1 | y | `Rn := D` |
| `wrd Rn` | `0000 01100 nnn 0000` | `n:reg` | 1 | y | `D := Rn` |
| `rdtb Rn` | `0000 01101 nnn 0000` | `n:reg` | 1 | n | `Rn := TB` |
| `put8 Rn, b` | `0000 01110 nnn 000b` | `n:reg b:obank` | 1 | y | `put8(b, Rn)` |
| `oe8 Rn` | `0000 01111 nnn 0001` | `n:reg` | 1 | y | `oe8(Rn)` |
| `tid Rn` | `0000 10000 nnn 0000` | `n:reg` | 1 | n | `Rn := TID` |
| `goto a` | `0001 000 aaaaaaaaa` | `a:addr` | 1 | n | `PC := a` |
| `jt a` | `0001 001 aaaaaaaaa` | `a:addr` | 1 | n | `if T: PC := a` |
| `jf a` | `0001 010 aaaaaaaaa` | `a:addr` | 1 | n | `if not T: PC := a` |
| `call a` | `0001 011 aaaaaaaaa` | `a:addr` | 1 | n | `R7 := (PC + 1) % DEPTH; PC := a` |
| `djnz Rn, a` | `0010 nnn aaaaaaaaa` | `n:reg a:addr` | 1 | n | `Rn := Rn - 1; if Rn != 0: PC := a` |
| `ldi Rn, i` | `0011 nnn 0 iiiiiiii` | `n:reg i:u8` | 1 | n | `Rn := i` |
| `lui Rn, i` | `0011 nnn 1 iiiiiiii` | `n:reg i:u8` | 1 | n | `Rn := (Rn & 0xFF) \| (i << 8)` |
| `addi Rn, i` | `0100 nnn 0 iiiiiiii` | `n:reg i:u8` | 1 | n | `Rn := Rn + i` |
| `subi Rn, i` | `0100 nnn 1 iiiiiiii` | `n:reg i:u8` | 1 | n | `Rn := Rn - i` |
| `cmpi Rn, i` | `0101 nnn 0 iiiiiiii` | `n:reg i:u8` | 1 | n | `T := Rn == i` |
| `tsti Rn, i` | `0101 nnn 1 iiiiiiii` | `n:reg i:u8` | 1 | n | `T := (Rn & i) != 0` |
| `andi Rn, i` | `0110 nnn 0 iiiiiiii` | `n:reg i:u8` | 1 | n | `Rn := Rn & i` |
| `ori Rn, i` | `0110 nnn 1 iiiiiiii` | `n:reg i:u8` | 1 | n | `Rn := Rn \| i` |
| `cp Rd, Rs` | `0111 ddd sss 0000 00` | `d:reg s:reg` | 1 | n | `Rd := Rs` |
| `add Rd, Rs` | `0111 ddd sss 0001 00` | `d:reg s:reg` | 1 | n | `Rd := Rd + Rs` |
| `sub Rd, Rs` | `0111 ddd sss 0010 00` | `d:reg s:reg` | 1 | n | `Rd := Rd - Rs` |
| `and Rd, Rs` | `0111 ddd sss 0011 00` | `d:reg s:reg` | 1 | n | `Rd := Rd & Rs` |
| `or Rd, Rs` | `0111 ddd sss 0100 00` | `d:reg s:reg` | 1 | n | `Rd := Rd \| Rs` |
| `xor Rd, Rs` | `0111 ddd sss 0101 00` | `d:reg s:reg` | 1 | n | `Rd := Rd ^ Rs` |
| `xort Rd, Rs` | `0111 ddd sss 0110 00` | `d:reg s:reg` | 1 | n | `if T: Rd := Rd ^ Rs` |
| `crcr Rd, Rs` | `0111 ddd sss 0111 00` | `d:reg s:reg` | 1 | n | `f := T ^ bit(Rd, 0); Rd := Rd >> 1; if f: Rd := Rd ^ Rs` |
| `crcl Rd, Rs` | `0111 ddd sss 1000 00` | `d:reg s:reg` | 1 | n | `f := T ^ bit(Rd, 15); Rd := Rd << 1; if f: Rd := Rd ^ Rs` |
| `cmp Rd, Rs` | `0111 ddd sss 1001 00` | `d:reg s:reg` | 1 | n | `T := Rd == Rs` |
| `lt Rd, Rs` | `0111 ddd sss 1010 00` | `d:reg s:reg` | 1 | n | `T := Rd < Rs` |
| `bt Rn, b` | `1000 nnn bbbb 000 00` | `n:reg b:u4` | 1 | n | `T := bit(Rn, b)` |
| `bn Rn, b` | `1000 nnn bbbb 001 00` | `n:reg b:u4` | 1 | n | `T := 1 - bit(Rn, b)` |
| `bx Rn, b` | `1000 nnn bbbb 010 00` | `n:reg b:u4` | 1 | n | `T := T ^ bit(Rn, b)` |
| `ba Rn, b` | `1000 nnn bbbb 011 00` | `n:reg b:u4` | 1 | n | `T := T & bit(Rn, b)` |
| `bo Rn, b` | `1000 nnn bbbb 100 00` | `n:reg b:u4` | 1 | n | `T := T \| bit(Rn, b)` |
| `bs Rn, b` | `1000 nnn bbbb 101 00` | `n:reg b:u4` | 1 | n | `Rn := (Rn & ~(1 << b)) \| (T << b)` |
| `put p, 0` | `1001 pppp 000 00000` | `p:opin` | 1 | y | `shadow(p, 0)` |
| `put p, 1` | `1001 pppp 001 00000` | `p:opin` | 1 | y | `shadow(p, 1)` |
| `put p, T` | `1001 pppp 010 00000` | `p:opin` | 1 | y | `shadow(p, T)` |
| `put p, !T` | `1001 pppp 011 00000` | `p:opin` | 1 | y | `shadow(p, 1 - T)` |
| `put p, ^T` | `1001 pppp 100 00000` | `p:opin` | 1 | y | `shadow(p, bit(SHD, p) ^ T)` |
| `put p, ^!T` | `1001 pppp 101 00000` | `p:opin` | 1 | y | `shadow(p, bit(SHD, p) ^ T ^ 1)` |
| `oe p, v` | `1001 pppp 110 0v 000` | `p:iopin v:u1` | 1 | y | `shadow_oe(p, v)` |
| `pcfg p, IN` | `1001 pppp 111 00 000` | `p:iopin` | 1 | y | `pcfg_in(p)` |
| `pcfg p, PP` | `1001 pppp 111 01 000` | `p:opin` | 1 | y | `pcfg_pp(p)` |
| `pcfg p, OD` | `1001 pppp 111 10 000` | `p:iopin` | 1 | y | `pcfg_od(p)` |
| `pcfg p, BYP` | `1001 pppp 111 11 000` | `p:lane` | 1 | y | `pcfg_byp(p)` |
| `get p` | `1010 00 pppp 00 0000` | `p:ipin` | 1 | y | `T := bit(S, p)` |
| `getn p` | `1010 00 pppp 01 0000` | `p:ipin` | 1 | y | `T := 1 - bit(S, p)` |
| `xget p` | `1010 00 pppp 10 0000` | `p:ipin` | 1 | y | `T := T ^ bit(S, p)` |
| `aget p` | `1010 00 pppp 11 0000` | `p:ipin` | 1 | y | `T := T & bit(S, p)` |
| `recv Rn, m` | `1010 01 0 1 mmm nnn 00` | `n:reg m:u3` | 1+ | y | `need(FULL[m]); Rn := M[m]; FULL[m] := 0` |
| `send Rn, m` | `1010 01 1 1 mmm nnn 00` | `n:reg m:u3` | 1+ | y | `need(not FULL[m]); M[m] := Rn; FULL[m] := 1` |
| `tryrecv Rn, m` | `1010 01 0 0 mmm nnn 00` | `n:reg m:u3` | 1 | y | `T := FULL[m]; if T: Rn := M[m]; if T: FULL[m] := 0` |
| `trysend Rn, m` | `1010 01 1 0 mmm nnn 00` | `n:reg m:u3` | 1 | y | `T := 1 - FULL[m]; if T: M[m] := Rn; if T: FULL[m] := 1` |
| `ld Rd, Ra` | `1010 10 0 ddd aaa 000` | `d:reg a:reg` | 2 | n | `load(d, Ra)` |
| `ldp Rd, Ra` | `1010 10 1 ddd aaa 000` | `d:reg a:reg` | 2 | n | `load(d, Ra); Ra := Ra + 1` |
| `st Rd, Ra` | `1010 11 0 ddd aaa 000` | `d:reg a:reg` | 2 | n | `store(Rd, Ra)` |
| `stp Rd, Ra` | `1010 11 1 ddd aaa 000` | `d:reg a:reg` | 2 | n | `store(Rd, Ra); Ra := Ra + 1` |
| `until p, l [, inbox]` | `1011 0 pppp l 0 000 0 m` | `p:ipin l:u1 m:flag` | 1+ | y | `wait(1 << (p & 7), l << (p & 7), p >> 3, 0, None, m)` |
| `until p, l, Rt [, inbox]` | `1011 0 pppp l 1 ttt 0 m` | `p:ipin l:u1 t:reg m:flag` | 1+ | y | `wait(1 << (p & 7), l << (p & 7), p >> 3, 0, Rt, m)` |
| `untilm Rr, b, c [, inbox]` | `1011 1 b cc rrr 0 000 m` | `b:ibank c:wmode r:reg m:flag` | 1+ | y | `wait(Rr >> 8, (S >> (8 * b)) & 255 if c == 2 else Rr & 255, b, c != 0, None, m)` |
| `untilm Rr, b, c, Rt [, inbox]` | `1011 1 b cc rrr 1 ttt m` | `b:ibank c:wmode r:reg t:reg m:flag` | 1+ | y | `wait(Rr >> 8, (S >> (8 * b)) & 255 if c == 2 else Rr & 255, b, c != 0, Rt, m)` |
| `tick i` | `1100 iiiiiiiiiiii` | `i:u12` | 1 | y | `arm(i)` |
| `txw Rs, n [, k] [, last]` | `1101 0 l nnnn sss kk 0` | `s:reg n:nbits l:lflag k:tkind` | 1+ | n | `need(not TXFULL); tx_push(Rs, n, l, k)` |
| `scfg Rs, k` | `1101 10 kkkk sss 000` | `s:reg k:sslot` | 1 | y | `set_cfg(k, Rs)` |
| `smap p, l` | `1101 110 pppp ll 000` | `p:opin l:sout` | 1 | y | `shadow_sel(p, l)` |
| `rxw Rd` | `1101 1110 00 ddd 000` | `d:reg` | 1+ | n | `need(RXFULL); Rd := RXBUF; T := RXBL; rx_pop()` |
| `sstat Rd` | `1101 1110 01 ddd 000` | `d:reg` | 1 | n | `sstat(d)` |
| `srd Rd, s` | `1101 1110 10 ss ddd 0` | `d:reg s:sreg` | 1 | n | `srd(d, s)` |
| `sgo m` | `1101 1111 00 mm 0000` | `m:sdir` | 1 | y | `sgo(m)` |
| `sstop m` | `1101 1111 01 mm 0000` | `m:sdir` | 1 | y | `sstop(m)` |
| `edge p, v, i` | `1110 pppp vv 0 iiiii` | `p:opin- v:eact i:u5` | 1 | y | `shadow(p, [0, 1, T, 1 - T][v]); arm(i)` |
| `edge p, v, Rr` | `1110 pppp vv 1 00 rrr` | `p:opin- v:eact r:reg` | 1 | y | `shadow(p, [0, 1, T, 1 - T][v]); arm(Rr)` |
| `shl8 Rr, i, o` | `1111 0 0 rr iiii oooo` | `r:reg4 i:ipin- o:opin-` | 1 | pin | `if i != 0: T := bit(S, i); f := bit(Rr, 7); Rr := (Rr & 0xFF00) \| ((Rr << 1) & 0xFE) \| T; T := f; shadow(o, T)` |
| `shl16 Rr, i, o` | `1111 0 1 rr iiii oooo` | `r:reg4 i:ipin- o:opin-` | 1 | pin | `if i != 0: T := bit(S, i); f := bit(Rr, 15); Rr := (Rr << 1) \| T; T := f; shadow(o, T)` |
| `shr8 Rr, i, o` | `1111 1 0 rr iiii oooo` | `r:reg4 i:ipin- o:opin-` | 1 | pin | `if i != 0: T := bit(S, i); f := bit(Rr, 0); Rr := (Rr & 0xFF00) \| ((Rr & 0xFF) >> 1) \| (T << 7); T := f; shadow(o, T)` |
| `shr16 Rr, i, o` | `1111 1 1 rr iiii oooo` | `r:reg4 i:ipin- o:opin-` | 1 | pin | `if i != 0: T := bit(S, i); f := bit(Rr, 0); Rr := (Rr >> 1) \| (T << 15); T := f; shadow(o, T)` |
<!-- isa-table-end -->

89 rows. Of the 65,536 words, 38,248 are legal and 27,288 (41.6%) are illegal. In group `1101`, the words that start `1101 1110 11` and `1101 1111 1` are free for future assists.

### 6.4 Pseudo-instructions

| Written | Assembles to | Words |
|---|---|---|
| `li Rd, imm16` | `ldi Rd, low byte`, then `lui Rd, high byte` if the high byte isn't 0 | 1 or 2 |
| `ret` | `jr R7` | 1 |
| `tick Rn` | `edge -, 0, Rn` | 1 |

A `li` whose value depends on a label defined later in the source always takes 2 words, because its size can't depend on an address that isn't known yet.

### 6.5 The shift rule

The `shl`/`shr` family is the only shift, and T is both the bit going in and the bit coming out:

1. If the `i` field names a pin, T := S[pin].
2. The outgoing bit is saved, T enters at the vacated end, and T := the outgoing bit.
3. If the `o` field names a pin, shadow[pin] := T.

With `-` in both fields it's a plain rotate through T. With both set, one instruction reads a sampled bit in and prepares the next bit out. The byte forms leave `Rr[15:8]` alone, so bit 7 is the top, and bytes assemble in place in either bit order.

### 6.6 Assembler source

One statement per line, and `;` starts a comment. Everything is case-sensitive. A label is a name followed by `:`. Numbers are decimal or `0x` hex. An expression can use `+ - * // % & | ^ << >>`, unary `-` and parentheses, with Python's precedence. A value that doesn't fit its field is an error.

A name can be defined only once. It can't be a mnemonic, a register, a pin name, or one of `T`, `inbox`, `eq`, `ne`, `chg`, `uo`, `ui`, `io`, `IN`, `PP`, `OD` and `BYP`. The words of the operand types `lflag`, `sslot`, `sdir`, `sout`, `tkind` and `sreg` are read as those words only in an operand of that type, and such an operand takes nothing else, so a label or `.equ` name spelled the same is never read there. Everywhere else they are ordinary names. In `txw` the kind comes before `last`, and either can be left out.

| Directive | Effect |
|---|---|
| `.equ NAME expr` | Defines a constant from numbers and earlier constants. A pin name or pin alias makes NAME a pin alias |
| `.org n` | Sets the address of what follows. Placing a second word at an address is an error |
| `.data expr` | Places one word, 0 to 65535 |
| `.if expr`, `.else`, `.endif` | Conditional assembly on numbers and earlier constants. These can nest |

The assembler warns when a line writes a pin that no earlier line names in a `pcfg`, because on every thread but 0 that write is dropped (section 11.5). The warning also covers `smap`.

Macros, `.include` and per-thread sections aren't defined yet.

## 7. Timing rules

### 7.1 Slot counts

Almost everything takes one slot, 4 cycles, whatever the other threads do. The Slots column in section 6.3 is the rule, and a run matched it for every row except `halt`, which ends the run. Branches take one slot, taken or not. There are no caches, no forwarding and no interlocks.

A pipeline that gives this: with four stages and four slots, each thread has at most one instruction in flight.

| Stage | Cycle | Work |
|---|---|---|
| 1 | *c - 3* | The memory cycle: present the address |
| 2 | *c - 2* | Capture the word. This absorbs the SRAM's clock-to-out |
| 3 | *c - 1* | Decode, read registers |
| 4 | *c* | The execute cycle: execute and write |

The latch array and SRAM use the same stages, which is why depth doesn't change timing. T, S, D and STALE are read in stage 4, because an event in cycle *c - 1* is visible to the instruction in cycle *c*. A match or inbox exit there clears STALE, so a read in stage 3 would set a false LATE. Only the architectural timing in sections 3 and 4 is normative.

### 7.2 The on-time rule

An event instruction executing in cycle *a* with new deadline D is **on time** if `1 <= (D - a) mod 65536 <= 32767`.

**Simple rule.** Let *k* be the number of slots from the instruction after the previous event instruction up to and including this one. While the previous event was on time, a step of at least `4k + 1` is on time. On a stream of `edge` instructions with nothing between them, *k* = 1, so the smallest spacing that is safe on any phase is 5 cycles.

**After a wait.** A wait that ends in cycle E retires at its first slot after E, and the next instruction executes in that slot, E + 1 to E + 4 depending on alignment. So the *k*-th instruction after a wait is on time with a step from E of at least `4k + 1`. The exception is a wait whose timeout had already passed when it armed. It leaves D behind E (section 3.3), so run `now` before the next step.

**Tight loops.** Let the previous event instruction be a `tick`, `edge` or `now`, executing at *a0* with deadline *D0*, and let *r* be the number of slots from the first world instruction after it up to but not including this one. This instruction executes no later than

```
max(a0 + 4k, D0 + 4(r + 1))
```

Writing the margin `m = D - a`, the worst case is `m1 = min(m0 + step - 4k, step - 4(r + 1))`, and the instruction is on time when `m1 >= 1`. The first term is the thread falling behind the deadlines. The second is a stall on the previous event, at the worst phase of D. Iterate it around a loop. A wait blocks every instruction behind it, not just world instructions, so after a wait use the rule above.

Two limits come out of it:

- **Throughput.** Around a loop, the steps have to add up to at least 4 cycles per slot, or the margin falls every time round. This holds on any phase of D.
- **Phase.** If D lands on the thread's own execute cycle, `D mod 4 = TID`, the next world instruction still sees P = 1 and loses a slot. Any other phase keeps that slot, so a step one shorter is on time. A step that is a multiple of 4 keeps D on one phase. `now` puts D one cycle after the thread's slot, which is a good phase, and RUN's D := *h* can be any phase. After `now`, any even step keeps D on TID + 1 and TID + 3, so it never meets the bad phase, while an odd step walks D through all four. So the second term is exact only when D can reach the bad phase, and the clean rate needn't rise steadily near the limit: unrolled groups of a shift naming a pin, `edge`, `edge`, with a first step of 8 after `now`, are clean at half period 6, late at 7 and clean from 8. A `goto` in that loop adds a slot, so half period 8 is then the first clean one.

| Loop | Which limit binds | D on any phase | D phased by `now`, even steps |
|---|---|---|---|
| `edge`, `edge`, a burst | Phase | Step 5 | Step 4. Started from RUN's D, step 4 is late on the 4 of 16 alignments with `D mod 4 = TID` |
| `edge`, `edge`, `goto`, sustained | Throughput, 3 slots for 2 steps, then phase | Step 7 | Step 6 |
| SPI: `shl8`, `edge`, `edge`, `djnz` | Phase, then throughput, 4 slots for 2 steps | Half period 9 | Half period 8 |
| UART TX: `shr8`, `tick`, `djnz` | Throughput, 3 slots for 1 step | Bit 12 | Bit 12 |

Count memory instructions as 2 slots. LATE is the ground truth, and firmware tests should check it.

### 7.3 What is exact

| Behaviour | Exact to | Holds when |
|---|---|---|
| Commit time of `tick`/`edge` | The cycle | LATE stays clear |
| Spacing between commits | The cycle | Same. D is arithmetic, so there is no accumulated drift |
| All of a thread's pins moving together | Same cycle | Always, inside the chip |
| Two threads hitting one cycle | Same cycle | They share a deadline by `wrd`, or by being started together by RUN |
| Snapshot time | The cycle | At every event |
| Pre-armed reaction | 3 cycles pad to pad | Every thread and phase run |
| Time from an input edge to a later `tick` | The cycle, relative to the first sample that saw the edge | Real input edges are asynchronous, so silicon adds up to 1 cycle of phase uncertainty |
| One thread's timing against the others | Identical | Always: fixed slots, and memory use stays in the thread's own cycle |
| Symbol edges of a send | The cycle | Symbol *k* reaches the pad at E + 1 + *k* x TPER after the start commit at E, with TPER 0 counted as 1, with no fraction and no outside clock. Stuff bits and appended CRC bits count as bits. Holds until the send ends |
| Symbol edges of a send with THALF | The half cycle | Symbol *k* reaches the pad at E + 1 + *k* x (TPER + TFRAC / 256), rounded down to a half cycle, with TPER 1 or more and no outside clock. With a 50% clock: below |
| Timed receive samples | The cycle, relative to the start | No RESYNC. With RESYNC and RGAIN 0, relative to the last edge, or with RACQ the last edge to HLVL |
| Manchester decoder bits and ends | The cycle | Given the values in SYNC2. Which cycle first samples a pad edge is section 10, item 2 |
| Manchester decoder bits and ends with RKIND 3 | The half cycle | Given the values in SYNC3, SYNC2F and SYNC2. With a 50% clock: below |
| An outside clock edge to a change of a shift output | 3 cycles pad to pad | Every thread and phase |
| One thread's unit against the others | Identical | Always. Units share only the synchroniser, which they only read, and the pads, of which a unit drives only pins its own thread owns and has switched to it |

What isn't exact:

| Behaviour | Spread |
|---|---|
| When a blocked thread resumes after an event | The next instruction runs 1 to 4 cycles after it, by slot alignment |
| Untimed reaction, `until` then `put` then `now` | 9 to 12 cycles, input pad to output pad |
| Anything crossing the mailbox | Depends on the other side |
| Pad and board delays | Outside the ISA. Multi-corner STA has to bound them |
| A wait that arms after a change has already passed | Fires on the cycle after it arms, so D is late by up to the arming time |

**Half cycles and the duty cycle.** THALF and RKIND 3 take their second edge in each cycle from the falling clock edge, so they assume a 50% duty cycle. A falling edge *d* ns off the middle of the cycle moves every falling-edge move and sample by *d* ns:

- **Send.** At 10 Mbit/s with THALF, half bits alternate 50 + *d* and 50 - *d* ns, so each edge is ±*d*/2 from the ideal grid: 0, ±0.5, ±1 and ±1.65 ns at *d* = 0, 1, 2 and 3.3. 802.3 lets a DTE's encoder put ±0.5 ns on its edges (7.5.2.1), and the MAU with the twisted-pair model add ±3.5 ns more (14.3.1.2.3), which B.4.3.3 widens to ±5.5 ns an edge for an integrated MAU, ±11 ns in its test from a trigger edge. On the way to a receiver the link segment adds up to ±5 ns (14.4.2.3), and B.4.1's budget comes to ±13.5 ns at the receiver's input on a link of the greatest length, where the medium gives ±1.5 ns and noise ±8, and to ±12.5 ns on a short one, where the medium with the transmitter's equalization gives ±6 and noise ±2.5. Without THALF, 50 MHz forces half bits of 40 and 60 ns, ±5 ns.
- **Receive.** With RKIND 3 the half cycles become 10 + *d* and 10 - *d* ns long. No receiver can take every pattern within 802.3's ±13.5 ns of zero-crossing jitter (14.3.1.3.1): two frames whose grids are 25 ns apart can share every transition, the preamble and SFD 12.5 ns late on the first grid and 12.5 ns early on the second, then a step of 25 ns to a run of equal bits, a 10 MHz square wave that the two grids read with clock and boundary edges swapped, as complementary bits. Below ±12.5 ns no two frames share their transitions. A receiver that sees only samples 10 ns apart knows each edge only to its sampling interval, which brings that step down: two frames that each step their edges by at most *s* ns after some bit can share their samples at every sampling phase from *s* = 20 ns, and at the worst phase a run of two edges moved 11 ns against the rest already has such a twin. Shapes made against a sampler have twins from about ±10 ns. Frames whose jitter follows the families below had none in 115,968 frames at 8 sampling phases, except steps of 15 ns or more in a frame's last bits. A pattern that moves no edge against the others in steps has no such twin at any size: random jitter, every bit-boundary edge *a* ns late with every mid-bit edge *a* ns early, and the reverse. RKIND 3 decodes those within ±13.5 ns at every sampling phase, transceiver corner and receiver skew sign, with *d* up to 3.3 ns of either sign and the partner's clock 0.01% off on 1518-octet frames, and also: rising edges late and falling edges early, and the reverse; a cable's data-dependent jitter, every edge after a whole bit with no edge late and every other edge early, 6 to 13.5 ns alone and 2 to 12 ns of it with random jitter making ±13.5 ns in all; and steps of up to 20 ns with a 50% clock (section 7.4). Its prediction follows the clock edges, a boundary edge in the window gives way to the clock edge after it without pulling, an edge after a bit-boundary edge is the clock edge however early it comes up to *p*/8 before the window, and a 19 ns pulse is two edges that pull nothing. It does not take every frame that a receiver could: data-dependent jitter of the opposite sign, edges after a whole bit early, which a transmitter equalized for a long cable gives on a short one, it decodes with a 50% clock alone up to 10 ns, and with random jitter making 13.5 ns up to 4 ns of it, or making 12.5 ns up to 6 ns, B.4.1's split for a short link (at 7 ns one frame of 2,400 tried was lost, and it had no twin); beyond that a run's bit-boundary edge comes in the window as a candidate and its clock edge after the deadline, and frames are lost that a receiver waiting about 0.6 bit before it decides could take. Steps of 21 to 27 ns lose frames too, some of which have twins in samples 10 ns apart, and so does a spike, one clock edge 21 to 27 ns earlier than the edges around it, all within ±13.5 ns (section 7.4); and under the 13.5 ns pattern of late bit-boundary edges, a frame whose 7 bits after its last whole octet end in a 0 can take the rise that starts TP_IDL, itself a bit-boundary edge 13.5 ns late, for the clock edge of an eighth bit: the byte end then has 8 bits to keep, not 7 to drop, and the FCS fails. That lost 20 of 36 such frames, each with a single reading of its samples; with the last bit a 1, under the reverse pattern or with random jitter, none of 180. 802.3 allows the MAU receiver ±1.5 ns more before DI (14.3.1.3.1), which in an integrated node the decoder sees; the tests apply the ISL3176E's own skew instead, up to ±2 ns at its slowest corner and less at the others. A step in a frame's last bits can also bring TP_IDL's first transition into the next window as one more bit; with REOP the byte end drops it, as an 802.3 MAC drops the excess bits of a frame (4.2.9, section 5.9.7). With RKIND 1's 20 ns steps the gap between the latest boundary edge and the earliest clock edge at ±13.5 ns is 3 ns.
- **The clock.** A 50% clock needs an even divider, or duty-cycle correction, at the board's clock source. The RP2350's clock generators divide on the input clock's rising edge, so an odd divider gives 33.3% at 3 and 40% at 5 unless their DC50 correction is on (RP2350 datasheet 8.1.3.4). The chip's clock pad and clock tree add their own duty error, which only STA bounds.

**10BASE-T transmit network.** IEEE 802.3 14.3.1.2.1 asks the transmitter to equalize. Its output through the twisted-pair model of Figure 14-7 into 100 Ω must fit the template of Figure 14-10 (Table 14-1, at a scale of 0.9 to 1.1) for all data. The peak into 100 Ω must be 2.2 to 2.8 V, and every harmonic of all-ones data at least 27 dB below the fundamental. 14.3.1.2.2 asks for 15 dB of return loss at all times, idle included.

The units need no other help. Shift output A drives TD+ and shift output B drives TD− through two equal legs (a 3.3 V CMOS buffer and a resistor each), and a passive network on the board equalizes and filters. With TXMODE 0x8081 and TXMAP 0x0401 (section 5.9.3: Manchester on A and its complement on B; idle A 0 and B 0; tail A 1 and B 0), the pair is +1, −1 or 0 and both legs are always driven. So the source impedance holds at idle, TP_IDL is the tail, and a link test pulse is a start with no word (section 5.9.5).

One such network, in a 100 Ω system on the line side:
- a 1:1.41 transformer and a resistive pad for a 100 Ω source;
- a constant-resistance bridged-T shelf with DC gain 0.5 and its zero at 2 MHz: 100 Ω arms, a bridge of 100 Ω in parallel with 796 pF, and a shunt of 100 Ω in series with 7.96 µH;
- a 5-pole Butterworth low-pass at 17 MHz.

On a circuit model of Figure 14-7 (insertion loss 6.81 dB at 5 MHz and 10.06 dB at 10 MHz, inside the ranges of 14.3.1.2), every trace of a 511-bit pseudo-random sequence fits the template. Each stays at least 0.09 V inside it from 10 ns after its trigger crossing, for any peak from 1.97 to 3.06 V. This holds at every tolerance of the model and of the network, with rise times of 2 to 15 ns, a clock duty error of ±2 ns and a source of 77 to 115 Ω.

Zero crossings move at most ±1.02 ns through the model (14.3.1.2.3 allows ±3.5 ns). Harmonics are at least 33 dB down, and the return loss is 18.4 dB or more with nominal parts. TP_IDL (Figure 14-11) and the link test pulse (Figure 14-13) fit into both loads of Figure 14-12, directly and through the model, from a peak of 2.36 V. So the board holds the peak between 2.36 and 2.8 V.

Without equalization no scale of the template fits. `eth_arp.s`'s header draws a simpler board: one RS-485 transceiver whose enable makes the silence, 1:1 magnetics, and no filter. That board misses the template, the harmonic limit (it reaches 11 dB down) and the return loss (a 23 Ω source, open at idle). It serves a bench, not a conforming link.

Timing is in cycles, not nanoseconds, so 40 to 45 MHz silicon works by changing constants, and programs that take their bit time from the host need no change at all. UART, SPI, I2C write, WS2812 and 1-Wire ran at 40 and 45 MHz with only constants changed. Three protocols depend on the clock: USB low speed needs 33 cycles per bit, so 50 MHz, CAN at 500 kbit/s needs at least 46 MHz to receive and 45.5 MHz to transmit, and a 10BASE-T half bit is 2.5 cycles at 50 MHz, exact on send only with THALF.

### 7.4 Measured limits

At 50 MHz, on the simulators, with LATE clear wherever an output is timed. A thread that only samples times no output, so the data it delivers judges it, and the row says where its LATE sets. ILL and bus conflicts always count. These come from the programs in section 11, so faster code may exist.

| Limit | Value | Conditions |
|---|---|---|
| One pin, burst of `edge` | Every 5 cycles on any phase of D, every 4 after `now` | 16 alignments |
| One pin, sustained | Half period 6, a 4.17 MHz square wave, after `now`. Half period 7, 3.57 MHz, on any phase of D | `edge`, `edge`, `goto`. Half period 5 is clean on none. On all 4 phases, half period 6 is clean on 48 of 64 runs and 7 on 64 of 64 |
| UART TX | 12 cycles per bit, 4.17 Mbaud | 4 threads x 4 alignments, back to back, decoded at the pad |
| UART RX, abutting frames | 43 cycles per bit, 1.16 Mbaud. 22 cycles, 2.27 Mbaud, with the 23-word program | 8 phases, 8 bytes. 23-word program: 4 threads x 8 phases, 64 bytes |
| UART RX, 1-bit gap | 15 cycles per bit, 3.33 Mbaud. 18 cycles with the 23-word program | 8 phases. On the 22-word program LATE sets below 17, when a sampling tick lands late |
| SPI controller | Half period 9, 2.78 MHz, on any phase of D. Half period 8, 3.125 MHz, after `now` | All 4 modes, 4 threads, 4 alignments. Half period 8 is clean on 3 phases of D and on none where `D mod 4 = TID`. Target clock-to-out up to half period minus 3 |
| SPI target | Half period 10, 2.5 MHz | Mode 0, 12 start phases |
| JTAG | Half period 8, 3.125 MHz | 37-word scan engine, 4 threads x 4 alignments. One `shr16` carries TDO in and TDI out. The 51-word program stops at half period 16 |
| SWD | Half period 24, 1.04 MHz | 128-word program, thread 0. 1 MHz ran on 4 threads x 4 alignments |
| MDIO | Station half period 20, 1.25 MHz. PHY half period 12, 2.08 MHz | Neither reaches the 2.5 MHz the standard allows, which is half period 10. PHY MDIO moves 3 cycles after MDC rises |
| Manchester RX with clock recovery | 30 cycles per bit, 1.67 Mb/s | 28-word program. LATE sets below 39, when a sampling tick lands late |
| Line-coded firmware (HDLC with NRZI, stuffing and CRC) | 46 cycles per bit | Thread 0 |
| USB low speed | 33 cycles per bit, so 50 MHz only | TX clean at 33 and at no rate from 32 down. The device link is measured at 33. RX alone, with its constants moved, is clean at the exact bit time down to 26 |
| CAN RX | 92 cycles per bit | Sample point at 60% of the bit or later, on every start offset across a bit. 90 on a single offset. Frames decode down to 68, but the ACK is late there, 32 cycles at 68 |
| CAN RX at any sample point | 68 cycles per bit, 61 decoded | Every start offset across a bit. 60 decodes on 1 of the first 8 offsets, which is phase luck |
| CAN TX | 91 cycles per bit | 500 kbit/s ran, with stuffing and CRC in firmware. It needs at least 45.5 MHz for TX and 46 MHz for RX |
| I2C controller | 100 and 400 kHz | UM10204 intervals in range, including tBUF, with rise modelled at the spec maximum. Past the spec, `i2c_write` is clean down to a 32-cycle clock at the pad, 1.56 MHz, against a target model with no interval checks |
| I2C target ACK | 3 cycles pad to pad at 100 and 400 kHz | Firmware decides before the 8th clock falls. At 1 MHz with minimum tHIGH, reaction is 3 to 5 cycles |
| Shift unit: UART with parity | 3 cycles a bit, 16.7 Mbaud, both directions | `uart_par_unit`, every thread pair and alignment, the sender 4% fast and slow. At 2 cycles a bit the send underruns |
| Shift unit: SPI target | Half period 3, 8.33 MHz, with CS held low, where MISO has no setup left and half period 4 gives 20 ns. Half period 7, 3.57 MHz, with CS pulsed high for one SCK period between words | `spi_unit`, modes 0 and 3. Each framed word's restart must fit in CS's high time: 11 cycles at half period 8, 21 at 4 |
| Shift unit: I2S and TDM controller | SCK at TPER 3, 8.33 MHz, sending and receiving, with TFRAC below 86. TPER 2, 12.5 MHz, sending only | `i2s_ctl`, 8 thread pairs and alignments. The receive half's start limits the receive side |
| Shift unit: I2S and TDM target | SCK half period 5, 5 MHz, realigned every frame. Half period 3 for a single frame | `i2s_tgt`, 4 threads, alignments and sampling phases |
| Shift unit: SD host, 1-bit bus | Clock 8.33 MHz | `sd_unit`. The thread that feeds the clock and takes the response sets it: at 12.5 MHz the clock stops |
| Shift unit: CAN | 1 Mbit/s, 50 cycles a bit, both directions. Receive to 1.25 Mbit/s. Two other transmitters out of step by 72 cycles at 500 kbit/s and 36 at 1 Mbit/s, and dominant bits stretched by 72 cycles at RXD, with only falling edges resynchronizing; the sender's clock 3% slow or 2% fast: from 2.5% fast a frame with ten bits between falling edges, the most stuffing allows, is lost. A stuff error's flag starts in the next bit at each of the 115 positions that can hold one, stuffed bits 5 to 119 counting SOF as 0: 32 to 35 cycles after the violating bit's sample, which with 100 ns each way through the transceiver is 0.15 to 0.18 bit times after the violating bit ends at 500 kbit/s and 0.54 to 0.60 at 1 Mbit/s | `can_unit`, every thread and alignment, at the shortest interframe space. RESYNC with RACQ, RGAIN 0, FIRST three quarters of the bit |
| Shift unit: 10BASE-T send with THALF | Every edge on the 50 ns grid: peak deviation 0.5, 1.5 and 0 ns at the transceiver's typical, slowest and fastest corners, its own skew, and the bit rate exact. A falling edge 1, 2 and 3.3 ns late adds about 0.5, 1 and 1.65 ns. Without THALF, 6.5, 7.5 and 6.0 ns, each from a least-squares grid per frame, or 5.5, 6.5 and 5.0 ns from the grid that makes the peak smallest | `eth_arp`, 3 transceiver corners (section 7.3) |
| Shift unit: 10BASE-T receive with RKIND 3 | No frame lost, 960 frames a case (half of them broadcast frames from a universally administered source), at 8 sampling phases, 3 transceiver corners and both receiver skew signs: random ±12 and ±13.5 ns; bit-boundary edges 10 to 13.5 ns late with mid-bit edges as early, and the reverse; rising edges 10 to 13.5 ns late with falling edges as early, and the reverse; every edge after a whole bit with no edge 6 to 13.5 ns late and the others as early, 2 to 12 ns of it with random jitter making 13.5 ns, and 3.5 ns of it with 9 and 10 ns of random jitter; the opposite sign up to 10 ns alone, 2 and 4 ns of it with random jitter making 13.5 ns, and 4 and 6 ns making 12.5 ns; steps of up to 20 ns either way; the falling clock edge 1, 2 or 3.3 ns early or late on the 13.5 ns patterns; the partner 0.01% off either way on 1518-octet frames, 192 frames a case. The opposite sign of data-dependent jitter loses 92, 177, 383 and 693 of 960 at 10.5, 11, 12 and 13.5 ns alone, and 2, 1, 15, 66, 236, 490 and 572 at 6 to 12 ns of it with random jitter making 13.5 ns, 6 and 18 at 8 and 8.5 ns making 12.5 ns (none at 7 ns, but 1 of 480 on the bench), and 30 at 8.5 ns making 13.5 ns; steps of +21, +22, +23, +25 and +27 ns lose 67, 143, 215, 358 and 544, and -21 to -27 ns 55, 147, 198, 326 and 455; a spike, every edge *g*/2 late but one clock edge after a bit change *g*/2 early, 69, 327 and 558 at *g* = 21, 24 and 27 ns, counted without the search for twins. Those are with a 50% clock: with the falling edge 1, 2 and 3.3 ns off either way, the opposite sign alone at 10 ns lost 25 to 31, 53 to 55 and 50 to 66 of 960 (none at 9.5 ns with 1 ns off, or at 9 ns with 2 ns off), steps of 20 ns 1 to 6, 1 to 12 and 0 to 19, and B.4.1's split 0, 1 and 3 of 3,840, also without that search. Clean lines at RPER 2, 3, 5, 8 and 15, with and without fractions, every RGAIN, RACQ and RDIFF: every bit exact, 5,440 lines. On the bench, frames with 1 to 7 bits after their FCS are kept and answered, and dropped with a bad FCS; under the 13.5 ns pattern of late bit-boundary edges, 20 of 36 frames whose 7 excess bits end in a 0 take TP_IDL's first transition as an eighth bit and are lost (none of 180 with a last 1, the reverse pattern or random jitter) | `eth_arp`; phases none a multiple of 2.5 ns. A frame counts as lost only if no other frame within ±13.5 ns fits its samples, which an exhaustive search of the frames that could decides; 24 step frames with such twins are left out. A second draw of every case, with new frames and phases, loses no frame where these lose none. With REOP the byte end drops the bits after the last whole octet, as 802.3 4.2.9 truncates a frame (5.9.7) |
| Shift unit: USB low speed | Every answer 4.1 to 5.5 bit times after the host's EOP, against 6.5, with host edge jitter to ±70 ns | `usb_enum`, 33 cycles a bit |
| Shift unit: LFSR | 1 bit a cycle on send, 2 cycles a bit on receive | `lfsr_unit` |

The rows above the shift units follow from one instruction per 4 cycles. The shift units of section 5.9 move bit timing into hardware, so they lift the limits that come from bit timing, and their rows come from the programs of section 11.2 that use them. There the thread that feeds a unit often binds before the unit does. THALF and RKIND 3 run in `eth_arp`, on a bench that gives every input its level at each falling edge and reads every output in both halves (section 3.1).

## 8. Reset state

Reset is asynchronous, active low, on every flop, and its release passes a synchroniser (section 9.1). Memory has no reset.

| State | Value after reset |
|---|---|
| TB | 0. Cycle 0 is the first cycle after reset ends |
| All threads | Stopped: RUN = 0, P = 0, WAITING = 0, MEM clear, IV = 0 |
| PC, R0..R7, T, D, S | 0 |
| LATE, STALE, ILL | 0 |
| All other sequencing state in section 2 | 0 |
| M0..M7, FULL0..7 | 0, so every word is empty |
| SYNC1, SYNC2, SYNC3, SYNC1F, SYNC2F | 0 |
| OUT, SHD | 0 |
| OE, SHOE, OD, BYP, SHBYP | 0 |
| SEL, SHSEL | 0, so every pin shows OUT |
| OWN | Every pin owned by thread 0 |
| BUSY | 0 |
| CFG[0..15] of every thread | 0, so the CRC register and the stuffer are off |
| Every shift unit | Idle: TXARM, TXRUN, TXTL, RXARM, RXHUNT, RXRUN, RXEP 0, both buffers and both shift registers empty, LA and LB 0, TXUR, RXOVR and RXSEEN 0. RXKIND, RXPH, RXG, RXOPEN and RXCAND 0, CRC and RXPE 0, and every other state of section 5.9.2 0 |
| Memory | Undefined |

So after reset every `uio` pin is undriven, and every `uo` pin is low, in both halves of the cycle, since TXH is 0. A UART TX line sits low until firmware sets it, which looks like a start bit, and an active-low chip select is asserted, so boards should allow for that. Nothing runs until the host starts it. Firmware should set its idle levels before it first blocks on the host. A line that must stay high or recessive through reset belongs on an `io` pin in `OD` mode with an outside pull-up.

## 9. Host interface

**This section is a specification only below the byte level.** The simulators run every command as bytes in one call, and serve all of it in cycle *h*. They don't model SCK timing, bit-level framing or the delays in sections 9.2 and 9.4.

### 9.1 Link

The host is the RP2350 on the demo board, using SPI0 on `ui0` (CS), `ui1` (SCK), `ui2` (MOSI) and `uo3` (MISO). Mode 0, MSB first, with SCK at most clk/6. Each SCK high and each SCK low lasts at least 2 cycles, and CS stays high for at least 3 cycles between transactions. The chip oversamples SCK through the input synchroniser, so there is no second clock domain, and a pad edge may reach it one cycle late (section 10), which shorter phases can't absorb.

Drive CS from a plain GPIO, not from the SPI block's own chip select. In mode 0 the RP2350's SPI block can pulse its chip select high between bytes, which would end every command after its first byte.

A transaction is CS low, one command byte, its arguments, then CS high. A rising CS discards any partial byte and ends the command. A command cut before its last argument bit does nothing, and the arguments it had are dropped. Bytes after a command's last argument, up to CS high, are ignored, a second command byte included. MISO reads 0 except during the reply bytes listed.

A rising SCK first seen at the pad in cycle *p* reaches the link in cycle *p + 2*, and MISO changes at the pad in *p + 3*. That leaves the host a stable bit at the next rising edge, provided its hold time is under 3 cycles. MISO returns to 0 three cycles after CS rises at the pad.

Cycle *h* is the cycle in which the last bit of a command's arguments reaches the link, so *h = p + 2* for that bit. The table in section 9.2 says which bit it is. A command takes effect after step 5 of cycle *h*, apart from a memory access (section 9.4) and the read of RDREG (section 9.2), which come later.

`rst_n` passes a synchroniser, so cycle 0 starts 2 clock edges after the pad rises. *h* is never before cycle 0, because the link is held in reset with everything else. Keep CS high or SCK low while `rst_n` is low and for 2 cycles after it rises: if CS is low and SCK high as reset ends, the link counts a rising SCK that never happened.

### 9.2 Commands

| Command | Byte 0 | MOSI after byte 0 | MISO after byte 0 | *h* is the last bit of | Effect |
|---|---|---|---|---|---|
| CAPS | `0x00` | 2 dummy bytes | ISA version, then thread count << 4 \| log2(DEPTH) | None | Version 0x01 is v0.1 |
| WRMEM | `0x01` | addr16, data16 ... | 0 | Each data word | Write each complete word, auto-increment |
| RDMEM | `0x02` | addr16, dummy16, dummy16 ... | 0 for 4 bytes, then data16 ... | addr16, then each word sent, as its low byte starts | Read, auto-increment |
| RUN | `0x10` | stop8, start8 | 0 | start8 | Stop and start threads. Bit *i* is thread *i* |
| SETPC | `0x11` | thread8, pc16 | 0 | pc16 | PC := pc mod DEPTH on a stopped thread |
| STATUS | `0x12` | 4 dummy bytes | 4 bytes, below | The fourth dummy byte | Reads status, clears BUSY |
| STEP | `0x13` | thread8 | 0 | thread8 | Run one instruction on a stopped thread |
| RDREG | `0x14` | thread8, reg8, dummy8, dummy16 | 0 for 3 bytes, then value16 | reg8 | Read a stopped thread's state |
| POST | `0x20 + n` | data16, optional dummy8 | 0 for 2 bytes, then the result byte | data16 | Fill M[n] if empty. Result bit 0 is 1 if refused |
| TAKE | `0x28 + n` | 3 dummy bytes | full flag, data[15:8], data[7:0] | The command byte | If M[n] is full, take it. Otherwise all three bytes read 0 |
| ABORT | `0x3F` | none | 0 | The command byte | Below |

Any other code is ignored for the rest of the transaction. In `thread8`, only the low 2 bits count, so 5 names thread 1.

**STATUS bytes**, first out first:

| Byte | Bits 7..4 | Bits 3..0 |
|---|---|---|
| 0 | RUN[3:0] | ILL[3:0] |
| 1 | LATE[3:0] | STALE[3:0] |
| 2 | FULL[7:4] | FULL[3:0] |
| 3 | 0 | Bit 0 is BUSY, others 0 |

STATUS is not one atomic read. Each reply byte shows the flops at the start of the cycle in which the last bit of the byte before it arrives. BUSY clears at the last bit of the fourth dummy byte, so a STATUS cut short leaves it set.

**RDREG `reg8`:** 0..7 are R0..R7, 8 is D, 9 is S, 10 is PC, 11 is flags with T in bit 0, LATE in bit 1, STALE in bit 2 and WAITING in bit 3. 12 is the shift status, as `sstat` reads it but with nothing cleared; 13 and 14 are the CRC register's bits 15 to 0 and 31 to 16, and 15 the end flags RXPE, as `srd` reads them; and 16 + *k* is configuration slot *k*, for *k* from 0 to 15. Other values read 0. RDREG returns the state when it is served, not at *h*: the flops at the start of a cycle from *h + 2* to *h + 5*, by thread and by *h* mod 4. A stopped thread can still move: STALE can set, or a stepped event can fire. A unit keeps running on a stopped thread, so 12, 13 and 14 can move between reads. 15 changes only at `rxw`, and 16 + *k* only at `scfg`, which a stopped thread doesn't run.

**BUSY** is sticky. It sets when a memory command finds no thread stopped, or SETPC, STEP or RDREG names a running thread, and the command then does nothing. RDREG and RDMEM read 0 in that case.

### 9.3 Running and stopping

**RUN.** All of it happens in cycle *h*. For each thread, with its own bit of each mask:

```
host_run(stop, start, h):
    if stop:
        if RUN:
            STOPREQ := 1
        else:
            P := 0
    elif start and not RUN:
        RUN := 1
        D := h
        P := 0
        WAITING := 0
        STOPREQ := 0
        LATE := 0
        ILL := 0
        STALE := 0
```

A start bit on a running thread does nothing, and a thread named in both masks is only stopped.

Every thread started by one RUN gets the same D, so threads can place edges on the same cycle with no firmware handshake. A started thread fetches at its next memory cycle, so its first instruction executes 4 to 7 cycles after *h*.

**Stopping.** A thread with STOPREQ runs `stop()` at its next execute cycle, 1 to 4 cycles after *h*. If that slot is the one that finishes a `ld` or `st`, the instruction finishes there and `stop()` runs in the slot after it. If a STEP was running that instruction, `retire()` ends the STEP and cancels any pending event itself, so the stop isn't lost. A pending event is cancelled, so its prepared pins don't commit. They stay prepared, though, and the thread's next event commits them, the same trap section 3.3 warns about for a timeout. A blocked wait is abandoned with PC on the next instruction, so restarting runs that instruction and doesn't re-arm the wait. T holds the wait's result if the event had fired, and otherwise keeps its old value. If the event hadn't fired, WAITING stays 1 and RDREG shows it, so the host can tell this thread from one whose wait ended. To repeat the wait, the host sets PC back by one with SETPC. A start, STEP and ABORT clear WAITING, and SETPC and a further stop leave it. Stopping leaves pins as they are. `halt` stops a thread with PC on the next instruction, after any pending event has fired, because it is a world instruction.

**Shift units.** A stop, `halt`, an illegal word, STEP, SETPC and RUN don't touch the shift units, their CRC registers included. A send already started finishes the words it holds, with any CRC they append, then ends with its tail: normally if its last word was marked, otherwise with an underrun. A reception keeps running and fills its buffer. An armed start stays armed, like a prepared pin, and the thread's next commit starts it.

**STEP.** In cycle *h*, for the named thread:

```
host_step():
    if RUN:
        BUSY := 1
    else:
        RUN := 1
        STEPPING := 1
        WAITING := 0
```

The thread stops as soon as one instruction retires. A blocked instruction keeps it running until it retires or the host stops it. A wait retires in its first slot after the event and a STEP ends there, with PC on the next instruction, which needs a new STEP. STEP doesn't touch D, so a stepped `tick` is usually late. An event armed by a stepped instruction stays pending when the thread stops, and fires at its deadline unless the host cancels it with a stop, a start or ABORT.

**SETPC.** In cycle *h*, for the named thread:

```
host_setpc(pc):
    if RUN:
        BUSY := 1
    else:
        PC := pc
```

SETPC leaves WAITING alone, so RDREG still shows an abandoned wait after PC has moved.

**ABORT.** In cycle *h*, `abort_thread()` for each thread, then `abort_pins()` once:

```
abort_thread():
    P := 0
    WAITING := 0
    if RUN:
        STOPREQ := 1
    tx_stop()
    rx_stop()

abort_pins():
    for i in range(8):
        OE[i] := 0
        SHOE[i] := 0
        SEL[8+i] := 0
        SHSEL[8+i] := 0
        if OD[i]:
            OUT[8+i] := 1
            SHD[8+i] := 1
```

So every `uio` pad is undriven, `uo` pins and bypass switches hold, and a `uo` pin on a shift output shows that output's idle level. Use ABORT, not a plain stop, when a target must see a line released. ABORT clears P and WAITING itself, so a thread it stops in a wait reads the same as one whose wait ended, which is deliberate: ABORT abandons the thread, and a host that needs the difference uses a plain stop.

**Load and start.** After reset all threads are stopped. The host writes the image with WRMEM, sets each thread's PC with SETPC, and starts them with RUN.

### 9.4 Memory and mailbox

Host memory accesses use a memory cycle of a stopped thread, so they never disturb a running thread, and a running thread keeps exact timing while another thread is reloaded. A stopped thread's memory cycle comes round every 4 cycles, so a host access lands in the first free memory cycle, *h + 1* to *h + 4*. RDMEM data reaches the reply register 2 cycles after that. Host addresses wrap at DEPTH, as thread addresses do.

If all four threads run, memory commands set BUSY and do nothing. That BUSY sets in *h + 1*, from the RUN flops of that cycle, and RDMEM runs this test as soon as `addr16` arrives. A memory command that fails is dead until CS rises: no later word of that transaction is written or read. Mailbox commands always work, because the mailbox isn't in memory.

The eight mailbox words each have a full flag, and the host and threads use them the same way, so they also serve as thread-to-thread channels and semaphores. There is no FIFO, so a POST to a full word is refused and the host must poll. A POST is 24 SCK cycles without its result byte, so at clk/6 it takes 144 clock cycles (2.9 µs at 50 MHz), about 350,000 words per second before any polling.

Mailbox operations are atomic and are never lost. POST decides on the flag when its last data bit arrives, and TAKE decides when its command byte arrives. If the host and a thread's successful `send`, `recv`, `trysend` or `tryrecv` touch the same word in one cycle, the thread goes first and the host's operation follows, on what the thread left.

## 10. Undefined, reserved and illegal

The rules above define every cycle, so two conforming simulators agree on every pin and register. This section lists every exception.

**Undefined.** A simulator may do anything consistent with these, and should fill unwritten memory with arbitrary values so firmware can't depend on it.

1. Memory contents before the first write to each address.
2. Exactly which cycle, or for the falling-edge sample which half cycle, an asynchronous pad change is first sampled, and any metastability in the synchroniser. A cycle-level simulator takes the values presented for that cycle, the falling-edge sample included (section 3.1).
3. Sub-cycle behaviour of a bypass path, and of its switch at a commit, and of the output switch: a commit that changes SEL[p], or a `pcfg` that changes OWN[p] or clears SEL[p], may glitch the pad, or the enable of a pin in `OD`, between clock edges. A cycle-level simulator may lag a bypass path by one cycle, and must say so.
4. Host SCK above clk/6, an SCK high or low shorter than 2 cycles, CS high for less than 3 cycles, CS moving while SCK is high, or CS low with SCK high as reset ends (section 9.1). Commands may be misread.
5. Everything before the first reset.
6. The cycle in which a STATUS byte, a host memory access, its BUSY or an RDREG read is served, within the limits in section 9. A byte-level simulator may do all of them in cycle *h*, and must say so.

**Reserved.** Defined today, but firmware must not rely on it, because it may change.

1. `ld`/`st` addresses of DEPTH or more, which wrap today.
2. Every illegal word. Illegal today, and may gain a meaning.
3. `edge -, v, n` with `v` other than 0.
4. Host command codes not listed, and STATUS byte 3 bits 7..1.
5. CMODE 3, SMODE 3, SN 0, SN 1 with SMODE 2, SYNCLEN 17 to 31, RPER 256 and up for the Manchester decoder, and RPER 0 and 1 and 16 and up with RKIND 3. Today CMODE and SMODE 3 act as 0, SN 0 acts as 1 with SMODE 1, SN 0 and 1 act as 2 with SMODE 2, SYNCLEN above 16 acts as 16, the Manchester decoder takes RPER above 255 as 255, and RKIND 3 takes RPER 0 and 1 as 2 and RPER above 15 as 15. Every configuration bit is a field, so none is left reserved.
6. RDREG `reg8` values 32 up.

**Illegal words.** A word is illegal if it matches no row in section 6.3, or has a value in a field that its type forbids. That includes any non-zero bit where a row has a fixed 0, `put` on `uo3`, `oe` on a `uo` pin, `pcfg` with `IN` or `OD` on a `uo` pin, `pcfg` with `BYP` on anything but a lane, `untilm` mode 3, `sgo` and `sstop` with a direction of 0, `smap` on `uo3` or with output value 3, and `srd` with a register of 3. Every slot of `scfg` and every kind of `txw` is legal.

Executing an illegal word sets ILL and runs `stop()`, whatever else is pending. PC stays on the word, so the host can read it. The check comes before the stall check, so a pending event is cancelled, not left to fire. No other state changes. The assembler can't produce an illegal word, but `.data` can place one.

**Defined, but probably a bug.** The hardware doesn't check these.

1. Two threads claiming one pin with no agreed hand-over. The last `pcfg` wins, and the old owner's writes are dropped.
2. A deadline 32768 or more cycles after the arming cycle, which is late by the rule in section 3.5. The test is on that distance, not on the step: `edge -, 0, R1` with R1 = 32768 straight after `now` lands 32765 cycles ahead, on time.
3. `tick 0` with D in the past, which is late. Use `now`. After `wrd` of a future cycle, `tick 0` or `edge p, v, 0` is on time and is the way to land on exactly that cycle, which section 7.3 relies on.
4. `untilm` with an empty mask. `eq` matches at once, and `ne` never matches.
5. Two parties filling one mailbox word without a convention. Each operation is atomic, but a refused `trysend` is silent unless T is tested.
6. Storing into code that a thread is running. It is defined by memory-cycle order, and a thread can even rewrite its own next instruction.
7. Running a `tick` from a D that a host STEP left behind. It is defined, and usually late.
8. Writing a slot of a half that is armed, running, hunting or in its tail. The half uses the new value from the next cycle, mid-word. RKIND is the exception: a reception keeps the decoder it started with, and a new RKIND acts from the next reception. So does a reception of RKIND 3 with its *p*, RGAIN and RACQ (RXP3, RXRG and RXRA): a new RXPER or RXOPT acts on them from the next reception. Timed samples, an outside clock and RKIND 1 read those fields as they are.
9. `smap` on a pin that no `pcfg` has claimed, and `pcfg` on a pin that is on a shift output. The second takes the pin off its output at once.
10. A third `txw` before `sgo tx` and its commit. It blocks until a commit starts the send, and with no event pending the thread never reaches one.
11. TCLK driven by one of the same unit's shift outputs. It is defined, and it loops: each symbol moves the clock that ends it. RCLK on the unit's own output doesn't loop, and is how a controller receives on its own clock (section 5.9.6).
12. A receive loop slower than one word per 16 bit times. Full words are lost and RXOVR sets. The last word of a reception still arrives.
13. `sgo rx` with HUNT on a pin that is already at HLVL. The hunt waits for the pin to leave HLVL and come back.
14. RACQ on a Manchester reception that does not begin with alternating bits. The wide window can take a bit-boundary edge for a clock edge, which loses the first bits or ends the reception.
15. RGAIN 2 or 3 against a sender whose clock is far from ours. Those gains follow only a small offset, so the bits slip and the reception ends early or decodes wrong.
16. `sgo rx` while the receive half is receiving, hunting or holding its last word. It stays armed (status bit 15) and starts a reception at the first commit after the half is idle, which may be a commit meant for something else. With HUNT that is only after HUNT is cleared.
17. Writing CODING, CSKIP, POLYHI, POLYLO, CHKHI or CHKLO while the half that uses the CRC register or the stuffer is running. As item 8, the half uses the new value from the next cycle, mid-frame. In particular CTX moved during a send or a reception takes the register from that half at once, in the middle of an append too, whose remaining bits are then CXOR. The half that gets the register goes on from the value the other half left: the CRC of the bits the other half took. That is exactly what a CAN node that loses arbitration needs, if CTX moves after the send half's last covered bit and before the receive half's next one (section 11.5). Otherwise the CRC is meaningless until that half's next start.
18. A POLY with a bit set below 32 - *w*. The bits below the width then don't stay 0, they move up into the CRC, and the CRC is not the standard's.
19. A word of kind `fcs` that is the last word of a send whose CRC is off, or a kind on a word while the stuffer is off. The kind has no effect.
20. REOP with RKIND 1 or 2. It has no effect. With RKIND 3 it asks for the byte end of a Manchester frame (section 5.9.7).
21. HDLC frames that follow one another between flags. The closing flag ends the first reception and the half hunts again, so the next frame is received only if the flags between them complete the hunt's cycle (three flags with a falling-edge hunt and SYNCLEN 8), or after a gap of ones. Frames that share one flag never both arrive (section 5.9.7).
22. Writing RXCNT with a count that RXTOT has already passed. `rx_bit` ends a reception only when RXTOT equals RCNT, so that reception ends some other way, or when RXTOT wraps at 2048 and reaches it.
23. REOP's end test on a protocol whose receptions need not be whole bytes. A reception that ends with 1 to 6 ones after a byte boundary and no stuff violation loses them, and its check is on the bits before them (section 5.9.7, the byte end). With RKIND 3, REOP drops the 1 to 7 bits after the last byte boundary of every reception that a window ends with no stuff violation.
24. REOP's end test with a CRC wider than 16 bits. Its byte end puts back only the register's bits 31 to 16, so after it the register mixes two times and the check is meaningless. The byte end of RKIND 3 puts back the whole register.

## 11. Firmware

### 11.1 Fit in 128 words

UART, SPI and the I2C controller each fit a 128-word memory on their own. Four threads can share one image too:

| Image | Words of 128 | Held |
|---|---|---|
| Full I2C controller, trimmed | 128 | UART TX 14, UART RX 20, SPI 22, I2C controller 72, no words spare. The only cost of the trim is that the host also posts the half bit to UART RX |
| Write-only I2C | 101 | UART TX 15, UART RX 23, SPI 21, I2C write 42 |
| Write-only I2C, WS2812 in place of UART RX | 97 | UART TX 15, SPI 21, I2C write 42, WS2812 19 |
| Write-only I2C, 1-Wire in place of UART RX | 112 | UART TX 15, SPI 21, I2C write 42, 1-Wire 34 |

Run as threads of one image, each protocol's pins and host traffic matched its solo run to the cycle, with no LATE on any thread. The standard 135-word set with the full I2C controller did the same on a 512-word memory, and so did a set with a thread hammering memory.

Three things need the 512-word memory. Device-side USB (194 words) and a full CAN node (166 words) don't fit 128, so on the fallback USB is one direction only and CAN is transmit or receive only. Target emulation with real storage, such as a 24C02, doesn't fit either, because its 256 bytes are the whole memory.

### 11.2 All programs run on v0.1

Every program below ran on the simulators. Programs with pins ran against device models that see only pad levels. The device models and decoders follow the protocol specs but were written alongside the firmware, so a shared misreading wouldn't show in them alone. `programs/sigrok/` also decodes the buses of 40 programs with sigrok's protocol decoders, which other people wrote from the protocol documents, on the simulator and on the RTL. It found two firmware bugs, in `i2c` and `cec`, fixed in the rows below. None has run on hardware yet.

Each program is the file named, under `programs/`. `programs/README.md` says what its bench checks. The published benches repeat the results below except the sweeps, which take too long for a routine run: the limit sweeps (12 cycles per bit for `trimmed/uart_tx.s`, half period 8 for `basic/spi.s` and `advanced/jtag.s`), the sender-error and jitter windows of `basic/uart_rx.s`, `advanced/manch_rx.s` and `advanced/usb_rx.s`, and the break-length and slow-host runs of `standard/uart_rx.s`. A name can appear in more than one folder, as different code.

| Program | Words | Result |
|---|---|---|
| `basic/uart_tx.s`, `standard/uart_tx.s` | 15 | The example in section 11.3. 115200 baud and 1 Mbaud on 16 of 16 threads and alignments, 9600 baud, and 12, 434, 5208 and 32767 cycles per bit |
| `trimmed/uart_tx.s` | 14 | Trimmed: the stop bit is a ninth shifted bit. 13 and 434 cycles per bit in the bench, 12 cycles per bit on 16 of 16 in the limit sweep |
| `standard/uart_rx.s` | 22 | Mid-bit sampling exact at 8 phases. Glitch rejected, framing error flagged in bit 8. 31 of 31 break lengths clean. No false bytes with a slow host |
| `basic/uart_rx.s` | 23 | Reports a framing error as a separate word, which keeps frames abutting. 115200 baud and 1 Mbaud at 8 phases. Sender error -5% to +4.5% at 115200 baud, -5% to +2.5% at 1 Mbaud |
| `trimmed/uart_rx.s` | 20 | Trimmed: the host posts the half bit too. 50 and 434 cycles per bit, and the same mailbox traffic in the shared image as solo |
| `standard/spi.s` | 22 | All 4 modes, half periods 9, 10 and 25, 16 alignments each |
| `basic/spi.s` | 20 | 21 with CPOL = 1. All 4 modes x 4 threads x 4 alignments at half periods 8, 9, 25 and 250. Multi-byte frames. MOSI and MISO decoded at the pads |
| `extra/spi_target.s` | 17 | SPI target, mode 0, half period 10 on 12 start phases |
| `standard/i2c.s` | 76 | Controller against an EEPROM model. At 100 kHz: write, random read, wrong address, data NACK, stretching on every clock, SDA stuck, SCL stuck at STOP, and a good transaction after a wrong address and after a bus fault: a write that ends early still takes its data word. At 400 kHz: write and random read |
| `extra/i2c.s` | 76 | Same source as `standard/i2c.s`. Same cases, through a second bench |
| `trimmed/i2c.s` | 72 | Trimmed, same host interface. The same cases as `i2c` at 100 and 400 kHz, and write and random read in the shared image |
| `basic/i2c_write.s` | 42 | 100 and 400 kHz: target ACK, stretching on every clock and after each ACK, NACK on data, wrong address, 3 transactions, 16 of 16 threads and alignments. Every UM10204 interval in range, including tBUF, with rise at the spec maximum. No SCL timeout |
| `standard/i2c_write.s` | 42 | Write-only controller. 100 and 400 kHz on all four threads, stretching, NACK on address and data, stretch timeout reported |
| `standard/i2c_target.s` | 62 | 24C02 style, plus 256 words of storage, 512-word memory only. Against `i2c` on another thread of the same chip at 100 and 400 kHz: write, random read, wrong address |
| `advanced/i2c_tgt.s` | 78 | Target with 32 bytes of storage, on 128 words. ACK depends on the address and lands 3 cycles pad to pad at 100 and 400 kHz, on all threads and alignments. Repeated START, pointer wrap, wrong addresses silent |
| `advanced/i2c_fp.s` | 87 | 1 MHz build of the target. Bytes correct. Reaction 3 cycles at minimum tLOW, 3 to 5 at minimum tHIGH. Intervals not checked at this rate |
| `basic/ws2812.s` | 19 | T0H 360 ns, T1H 700 ns, 1240 ns per bit, no jitter, 72 bits on 16 of 16 threads and alignments, latch gap checked |
| `standard/ws2812.s` | 17 | Same bit timing, on all four threads |
| `basic/onewire.s` | 34 | Reset 490 us, presence, 5 bytes written, no-device case reported, 12 of 12 device timing corners, 16 of 16 threads and alignments. No read slots or search |
| `standard/onewire.s` | 28 | Reset 480.0 us, presence, write byte, no-device case. No read slots or search |
| `advanced/jtag.s` | 37 | Host-driven scan engine on a 2-TAP chain: both IDCODEs, IR scan, 40 bits through BYPASS, Pause-DR mid-scan, with a pad decoder walking the TAP states. Half period 9 in the bench, 8 on 16 of 16 in the limit sweep |
| `standard/jtag.s` | 51 | Fixed sequence against a TAP model: IDCODE, IR load, BYPASS loop, half periods 25 and 16 on all four threads |
| `advanced/swd.s` | 128 | Host with turnaround: line reset, JTAG-to-SWD, DPIDR, writes, posted AP reads, WAIT retry and limit, FAULT, no response, read and write parity. Contention checked every cycle. No words spare |
| `extra/swd.s` | 64 | Read of DPIDR, ACK and parity errors reported, no contention |
| `advanced/mdio_sta.s` | 47 | Clause 22 station, read and write, 1.25 MHz, PHY output delay 0 to 300 ns, 16 of 16 threads and alignments |
| `advanced/mdio_phy.s` | 60 | Plus 32 registers. Follows an external MDC, and MDIO moves exactly 3 cycles after MDC rises |
| `advanced/manch_rx.s` | 28 | Clock recovery: preamble lock, start mark, end of frame, 4,800-bit frame, 4 threads x 8 phases. Sender clock error -20% to +24%, edge jitter 10% of a bit |
| `standard/manch_rx.s` | 13 | Fixed format. Sender clock +/-10% |
| `advanced/dmx_tx.s` | 29 | DMX512 transmit. Full 512-slot universe decoded at the pad, break 110 us, MAB 16.2 us, clean through a 1.4 ms host stall |
| `advanced/dmx_rx.s` | 52 | Slot windows 1, 250 and 509, shortest legal break and MAB, sender error 2%, 1 ms mark between slots, non-zero start code ignored. The universe streams through the mailbox |
| `advanced/ps2_dev.s` | 114 | PS/2 device, both directions with parity and device ACK. Commands FF, F2, ED, EE and FE, bad parity answered, host inhibit and request-to-send handled, byte resent |
| `advanced/usb_tx.s` | 43 | Low-speed transmit, NRZI and stuffing at 33 cycles per bit, every D+ edge on the grid |
| `advanced/usb_rx.s` | 69 | Low-speed receive in 2 threads: stuffing-heavy packets, sender error 1.5%, D+/D- skew to 160 ns, 2-bit gaps, jitter to 60 ns. 80 ns loses 4 runs of 12, and 100 ns fails |
| `advanced/usb_dev.s` | 194 | Device link in 3 threads, 512-word memory: SETUP with CRC-16 check, ACK, NAK, IN with DATA1, host ACK, address filter, bad CRC rejected. Worst turnaround 6.06 bit times against 6.5. No enumeration, data toggle or token CRC-5 check |
| `advanced/can.s` | 166 | 3 threads, 512-word memory. 500 kbit/s, stuffing and CRC-15 in firmware, sender error 2%, 440 ns transceiver loop, ACK in the right slot, both arbitration outcomes. RX alone 92 words and TX alone 74 ran on 128. No error frames or retransmit |
| `advanced/uart_bypass.s` | 65 | UART pass-through on the bypass lane of `uo4`, with one byte replaced in flight, chosen by its frame number or by the byte before it. 16 to 6553 cycles per bit, every thread, alignment and sender phase at 16 and 17: `uo4` equals `ui4` on every cycle but the new data bits, each on the incoming frame's bit boundary. The lane switches 3 cycles into the start bit and 9 into the stop bit, where `ui4` and OUT have been equal for 3 cycles and stay so for 3 more |
| `advanced/eth_arp.s` | 402 | 10BASE-T node at 50 MHz in 2 threads, 512-word memory. The units send and decode 802.3 Manchester with clock recovery, append and check the CRC-32 FCS, and make TP_IDL and the link test pulses. It answers ARP requests, and ICMP echo requests of up to 57 data octets, for the MAC and IP the host posts, waits 96 bit times of quiet line before sending, and drops and counts runts, FCS errors and frames for others. Against a partner line through a transceiver model, judged by a checker of 802.3 clauses 3, 4, 7 and 14 and the RFCs: 3 transmit corners; receive at 3 corners, 8 sampling phases, 9 thread pairs and 4 alignments; partner clock +/-0.01%; 12-bit preambles; frames 96 and 47 bit times apart; an FLP burst; +/-8 ns of random jitter on every edge at 8 phases. Both units run on half cycles: THALF puts every half bit at exactly 2.5 cycles, so every transmit edge is within the transceiver's own skew of the 50 ns grid (0.5 ns typical, 1.5 at its slowest corner) and the bit rate is exact, and RKIND 3 times every received edge to 10 ns. At 802.3's +/-13.5 ns no frame is lost at any phase or corner, with the receiver's skew either way: random, with the bit-boundary edges late and the mid-bit ones early, or the reverse, with transitions to HI late and to LO early, or the reverse, and with a cable's data-dependent jitter of 13.5 ns, or 3.5, 6 and 10 ns of it with random jitter; nor with the reverse sign of that jitter up to 10 ns, or 4 and 6 ns of it with random jitter (6 ns with +/-6.5 is B.4.1's short link), broadcast frames from a universally administered source, the clock's falling edge 2 ns late, the partner 0.01% off on 1518-octet frames, or a phase step of up to 17 ns; and with REOP's byte end, frames that end with 1 to 7 bits after their FCS are kept and answered, as 802.3 4.2.9 truncates them (section 7.4). No collision detection, IP options or reassembly |
| `advanced/usb_enum.s` | 504 | USB low-speed device on endpoint 0 at 50 MHz in 3 threads, 512-word memory. The units do NRZI, bit stuffing, the CRC5 check of tokens on a second unit, the CRC16 check of the host's data and the CRC16 of its own, the end of packet and the clock recovery. It answers GET_DESCRIPTOR for its 18-byte device descriptor and for its configuration (the configuration descriptor and one interface descriptor, 18 bytes) in packets of 8, and SET_ADDRESS, with every status stage, NAK when nothing is due and STALL for other requests, and keeps the data toggles, retries after lost ACKs, keep-alives and bus reset. Against a host model at the pins through a bus model, judged by a checker of USB 2.0 chapters 7 to 9 and by sigrok's USB decoders: the enumeration with two 10 ms resets; 8 thread placements, 4 alignments and 8 sampling phases; the host 0.05% fast and slow, edge jitter to +/-70 ns, 2-bit inter-packet delays, EOPs of 670 ns to 1.5 us; a SETUP and a reset in the middle of a transfer. Every answer comes 4.1 to 5.5 bit times after the host's EOP, against 6.5. A packet with 1 to 6 dribble bit times before its EOP, at most six bit times with no transition counting its own last 1s (USB 2.0 7.1.9.1), gets the answer it gets without them: the byte end drops them (section 5.9.7), so the firmware takes whole bytes with end flag 1 alone, and a handshake of 8 or 9 bits. Tested with one dribble bit on every kind of host packet, alone and on every packet of whole enumerations, with 2 to 6 on every packet of an enumeration, and against extra bits that are not a dribble, a 29-bit token whose CRC5 passes among them. Tokens to endpoints 1, 2, 4 and 8 go unanswered. 2 words free: SET_CONFIGURATION, 6 more, is STALLed |
| `advanced/spi_unit.s` | 50 | SPI target in one thread on the shift unit, modes 0 and 3, MSB first. The receive half samples MOSI at each rising edge of SCK (RKIND 2), the send half moves MISO 3 cycles after each falling edge (TEXT, TFALL), with a 1-bit dummy word first in mode 3, and the wait for CS low is the commit that starts both. RXCNT ends each frame, RXEND one the controller cuts short. Against a controller modelled on the RP2350's SPI block (PL022, Motorola frame format): mode 0 with CS pulsed high for one SCK period between bytes, mode 3 with CS held low, frames of 1 to 100 bits, every thread and alignment, at 2 to 10 sampling phases; MOSI words and MISO bits checked, and decoded by sigrok. Each frame's restart has to fit in CS's high time: 11 cycles at half period 8, 21 at 4, so with a one-period pulse the fastest clock is half period 7, 3.57 MHz. One endless reception (RXCNT 0) streams through the pulses from half period 3, 8.33 MHz, where MISO has no setup left; 4 gives 20 ns. Built with MISO moved 3 cycles after the capture edge instead, it runs from half period 1.5, and at 2 with 20 ns of setup and 40 ns of hold. A reception ends by count or by REND, not when CS rises |
| `advanced/can_unit.s` | 251 | CAN 2.0A node in one thread on the shift unit, 512-word memory, at 500 kbit/s. The unit stuffs and destuffs (SMODE 2, N 5), appends and checks the CRC-15 (CINIT 0, CXOR 0), MSB first both ways, and times every bit from a hunted SOF; the firmware writes RXCNT once the DLC is in, so that the reception ends at the CRC delimiter, with CHK 0x8B320000, the good residue fed the delimiter's 1: before the second word for a remote frame or 8 bytes, otherwise from that word, and it waits for the next word again 41 to 44 cycles after the second word's last sample. It checks its own bits with `xget` 40 cycles into each, and on a lost arbitration stops the send and hands the CRC register to the receive half before that half samples the lost bit, so the winner's frame is checked and acknowledged; then it sends again. The ACK is a dominant bit armed in the send half as a 1-bit word of no kind, not marked last, so the send ends by underrun after it, 1 to 4 cycles after the slot starts; after a stuff error the same bit starts the error flag and a 5-bit word follows it, and after CRC, bit and ACK errors the flag is a 6-bit word. TXD is an `io` pin in OD. Against frames built from the Bosch specification, another node's ACK and a rival that arbitrates by the wire, and decoded by sigrok: 0 to 8 bytes, all-zero and all-one stuffing, a stuff bit after the CRC, remote frames, back to back at the shortest interframe space at 500 kbit/s and 1 Mbit/s (receive to 1.25), every thread and alignment, the sender's clock 3% fast and slow (from 2.5% fast a frame with ten bits between falling edges, the most stuffing allows, is lost), transceiver loops of 80 to 440 ns, a corrupted CRC, six equal bits, a stuff error at each of the 115 positions that can hold one at both rates, the next frame also at the earliest start the error frame allows (and with loops of 440 ns at 500 kbit/s and 380 ns at 1 Mbit/s), spikes and a lone dominant bit on an idle bus, a stuff error that only the node sees with another node that answers its flag, no ACK, arbitration lost and won, and its own frames back to back with the whole end of frame and intermission between them. Only falling edges resynchronize, as CAN's synchronization rules ask (RESYNC with RACQ, RGAIN 0, the sample 75 cycles after each): a dominant bit stretched at RXD by the transceiver, or by two other transmitters out of step in arbitration, moves no sample, so stretches of up to 72 cycles are clean (76 is not), and the node receives and acknowledges the winner's frame with the two other transmitters 72 cycles apart at 500 kbit/s (75 fails) and 36 at 1 Mbit/s (38 fails). With SMODE 2 a stuff error ends the reception at the violating bit, and 7 slots after the `rxw` that takes the last word, whichever word it is, the flag starts: in the next bit, 32 to 35 cycles after the violating bit's sample, so 0.15 to 0.18 bit times after it ends at 500 kbit/s and 0.54 to 0.60 at 1 Mbit/s with 100 ns each way through the transceiver, for 40 to 140 cycles a bit. After its ACK the node restarts its hunt one bit time later, and after a flag eight bit times later, past the flags that other nodes start by its seventh bit on seeing it, so no reception starts on its own bits or on theirs, and a reception under 16 bits, a stuff error in SOF to DLC bit 3, gets its flag too; only a spike that never sampled dominant is dropped. No extended frames, error counters, overload frames or form check of the end of frame |
| `advanced/sd_unit.s` | 194 | SD card host in SD mode, 1-bit bus, in 2 threads on their shift units, 512-word memory. One unit sends each command most significant bit first with its CRC7 appended (CW 6, CINIT 0, CXOR 0, kinds `crc` and `fcs`) and the clock on its second output (data with its clock); a committed `oe` releases CMD as the end bit ends, and the same unit receives the response at the rising edges of its own clock, read back from the pad (RKIND 2). That reception starts at the turnaround commit, with no hunt and the start bit as its sync pattern (SYNCLEN 1, SYNC 0), so every rising edge samples and the start bit is taken at any phase of the card's output, the cycle of a rising edge included; it checks the CRC7 with the end bit (CHK is POLY), and for R2 with CSKIP 7, and keeps the 47 bits after the start bit, R2's 135. The other unit receives the data block on DAT0 the same way, from a reception that `until DAT0, 1` starts before each block, with the CRC16-CCITT, and its thread rewrites RXCNT after word 130, so that the reception ends at bit 4113, past RCNT's 2047. Against a card model written from the SD Physical Layer Simplified Specification, whose CRCs reproduce the examples of its section 4.5, with commands and responses decoded by sigrok: CMD0, CMD8, CMD55 and ACMD41, CMD2 and CMD3 at 397 kHz, then CMD7 and CMD17 with a 512-byte block at 6.25 MHz; reads on all 12 thread pairs; every response delay from 2 to 64 clocks against every whole-cycle card output delay from 0 to two half clocks at 8.33, 6.25 and 3.13 MHz, each start bit measured where that delay puts it; blocks from 2 clocks after the command, during the response, to 1000 clocks at every output delay, and up to the 100 ms read time-out of its section 4.6.2.1; corrupted data, end bits and responses all fail the checks. The fastest clock is 8.33 MHz, set by the thread that keeps the clock fed and takes the response; at 12.5 MHz it reports the stopped clock. No 4-bit bus, writes or busy |
| `advanced/ow_unit.s` | 108 | 1-Wire master at standard speed in one thread on the shift unit, with output A on `io4` in OD: a shift output on an open-drain pin, which releases on 1 and can't drive high. One 2-bit word at TPER 24600 makes the 492 us reset and its high time, and the receive half samples the presence 70 us after the release. Each time slot is a 13-bit word at TPER 300: 6 us low for a write 1 or a read, 66 us low with 12 us of recovery for a write 0, 78 us a slot. The receive half samples every slot 13 us after its fall, on the grid of the same commit, and its CRC-8 (CODING 0x0087, CSKIP 8 past the command's own slots) leaves the residue 0 after a Read ROM's 64 bits. Against a device model written from the DS18B20 data sheet's signalling, decoded by sigrok: Maxim's worked ROM code (CRC 0xA2) on every thread and alignment and random ones back to back, a flipped ROM bit (the check fails), no device, write samples at 15 and 60 us, a read 0 held 15.5 to 60 us, presence 15 to 60 us after the release and 60 to 240 us long, line rise times to 6.8 us. A rise of 7.04 us, or a read 0 held under 13 us, fails. A new transaction waits for the last slot to end, or `sgo` stays armed. No search ROM or overdrive |
| `advanced/uart_par_unit.s` | 77 | UART 8E1 or 8O1 on two shift units. The sending unit appends the parity bit from its CRC register (width 1, POLY x + 1, CXOR 1 for odd), and one send holds a whole burst, the stop bit and the next start bit going out as a 2-bit word of no kind. The receiving unit hunts each start bit and covers the data, parity and stop bits with its own register, so one residue, 0x80000000 for even and 0 for odd, checks the parity and the stop bit together, and bit 9 of the word tells a framing error. In a loopback on `io0` and against a sender model with wrong parity bits and stop bits of 0, decoded by sigrok with parity: 115200 baud to 16.7 Mbaud (3 cycles a bit) on every thread pair and alignment, the sender 4% fast and slow. At 2 cycles a bit the send underruns |
| `advanced/ws2812_unit.s` | 44 | WS2812B driver in one thread on the shift unit: three symbols a bit (SYMS 2) with the output codes 1, the bit and 0 (MAPA 0x23), MSB first, and TPER 20 with TFRAC 213, 1.25 us a bit with no drift. T0H and T1L are 400 or 420 ns, T1H and T0L 820 or 840 ns, inside the data sheet's +-150 ns, on every thread and alignment, decoded by sigrok. TPER 18 to 23 with no fraction, and 17 with TFRAC 128, also meet every window; 17 and 24 with no fraction don't. A host that falls behind cuts the frame |
| `advanced/lfsr_unit.s` | 110 | PRBS generator and additive scrambler on the LFSR (CMODE 2) of one unit, with another unit receiving. PRBS7 (x^7 + x^6 + 1) and ITU-T O.150's 2^9 - 1 and 2^15 - 1, with POLY the reciprocal of each (5.9.7), pass a checker built from the named Fibonacci recurrences with no error: over a whole period each at 50 Mbit/s, and after a sync byte at 2 to 10 cycles a bit on 4 thread pairs; inverted with output code 1. Frames scrambled behind a sync byte of no kind are descrambled by the other unit, whose LFSR starts at the first bit after its sync pattern, and the line XOR the data is the PRBS, for each width. The named polynomial set as POLY fails the checker. The receiver needs 2 cycles a bit |
| `advanced/crc_unit.s` | 85 | The idle send half as a CRC engine in one thread, one bit a cycle at TPER 0 with no pin on its outputs: a buffer in memory goes through as words of kind `crc`, 4 slots a word, two words waiting before the start, and `srd` reads the register once status bit 5 clears; the thread then reflects and complements the result as the CRC asks. CRC-32 gives 0xCBF43926 for `123456789` and equals zlib.crc32 on buffers of 1 to 256 bytes on every thread, 2,520 cycles for 256 bytes. CRC-8/MAXIM gives 0xA1, and 0 over a 1-Wire ROM code with its CRC byte, on 128 words |
| `advanced/xy2_unit.s` | 108 | XY2-100 controller in 3 threads on their shift units, started at one absolute cycle: each thread arms its send, writes the shared start cycle into D with `wrd` and commits there with `tick 0`, so all three run on one grid of 25-cycle bits, 2 Mbit/s, TPER 12 with TFRAC 128. One unit makes CLK (output codes 1, 0) and SYNC (the bit) on its two outputs; the others send the control bits 0 0 1 and the 16-bit position MSB first, as words of kind `crc` and `fcs`, so each appends the even parity bit. Read at CLK's falling edges by a frame reader written from the XY2-100 frame: frames back to back on 8 thread placements and alignments, extreme positions, SYNC low on the parity bit only, and SYNC, X and Y moving only in cycles where CLK rises |
| `advanced/i2s_ctl.s` | 117 | I2S or TDM controller in 2 threads on their shift units, started at one absolute cycle by `wrd` and `tick 0`, 512-word memory. One unit makes SCK (output codes 2 then 3, low then high) and WS or FS (the bit) from a pattern table; the other sends SD from a table, and receives the target's SD at SCK's rising edges, read back from its `io` pad (RKIND 2), from a second commit 4 x TPER after the start. Both lines begin with 2 bits before the first word, so a receiver that latches WS only at SCK's edges sees it change. Against a slave transmitter model written from the I2S bus specification (Philips, 1996) and the TLV320ADC3140 data sheet's TDM mode (TX_OFFSET 1, BCLK_POL 1), read by readers written from them and decoded by sigrok's `i2s` and `tdm_audio`: I2S at 48 kHz with 2 x 32 bits (TPER 8 with TFRAC 35, 48,008 Hz) and 2 x 16, and at 8 kHz; TDM at 48 kHz with 8 x 16 and 4 x 32 bits (6.144 MHz) and at 8 kHz with one 8-bit slot; 8 thread pairs and alignments; the specification's Table 1 timing on every edge. Transmit and receive are clean from TPER 3 (8.33 MHz) with TFRAC below 86, from 4 up to 170 and from 5 at any fraction; at TPER 2 (12.5 MHz) the bus is right, but the receive half's start comes too late |
| `advanced/i2s_tgt.s` | 55 | I2S or TDM target in one thread on the shift unit, 512-word memory. The receive half samples SD at SCK's rising edges (RKIND 2, RXCNT the frame's bits), the send half moves SD out 3 cycles after each rising edge (TEXT, TFALL 0), and a wait `untilm` on SCK high with WS low (FS high), armed while WS is high (FS low), is the commit that starts both on the edge that first latches the frame's WS (FS) change, so the first bit is the next edge's, as I2S puts the MSB one SCK after WS changes. Each later frame is armed by `sgo` during the frame and started by `now` 2 to 5 cycles after its first edge. Against a controller model written from the same documents, with WS (FS) and SD 0 to 2 cycles after its falling edges, on 4 threads, alignments and sampling phases, decoded by sigrok: I2S at 48 kHz (2 x 32 and 2 x 16 bits) and at 8 kHz, TDM at 48 kHz (4 x 16 and 2 x 32 bits) and at 8 kHz with one 8-bit slot; every frame received, SD out latched right with setup of at least 0.2T. The restart sets the limit: SCK half period 5 (5 MHz) is clean, 4.5 is not, because a later frame's first bit goes out up to 8 cycles after the frame's first edge; a single frame runs at half period 3. TDM at 128 fs (6.144 MHz) needs one endless send from the first frame instead. WS (FS) must change while SCK is low, as a controller that changes it on the falling edge does |
| `standard/cec.s` | 50 | HDMI CEC initiator. All bit windows met. A start bit falls only after 7 bit periods of free line, so another initiator's frame is waited out. Each block's ACK is judged by the frame's destination, inverted for a broadcast; a block that fails ends the frame, and its later blocks are taken unsent. No arbitration or retransmission |
| `extra/lin.s` | 52 | LIN responder header with auto-baud at +/-14%. Header only |
| `extra/nec_tx.s` | 41 | NEC IR transmit. Carrier exact, 1316 cycles with 439 high |
| `extra/parallel.s` | 7 | 8-bit parallel capture. Data hold 0 cycles after the strobe, strobe period 28 |
| `standard/hdlc_tx.s` | 48 | NRZI, stuffing and CRC-16/X-25 in firmware, decoded from the pad trace |
| `standard/crc16.s` | 14 | CRC-16/CCITT, published check value |
| `standard/crc8.s` | 12 | CRC-8/MAXIM, published check value |
| `standard/crc32.s` | 26 | CRC-32, published check value |
| `standard/reflex.s` | 15 | The reaction test behind section 11.4. Pre-armed 3 cycles, firmware 9 to 12 |
| `standard/memstress.s` | 17 | Memory load for the timing-isolation check: a thread hammering `ld` and `st` changes nothing on another thread |
| `basic/memhog.s` | 8 | The same job in the concurrency runs |

### 11.3 Worked example: a UART transmitter

This is the exact source that ran. `.equ` sets constants.

```
; UART TX 8N1. Runs on any thread; bit time is settable at run time.
; mailbox MCFG: cycles per bit (12..32767). mailbox MIN: bytes to send.
.equ TX uo0
.equ MCFG 7
.equ MIN 0
start:  pcfg  TX, PP        ; this thread owns the pin
        put   TX, 1         ; line idles high
        now                 ; commit it
        recv  R2, MCFG      ; R2 = cycles per bit, sent by the host once
byte:   recv  R0, MIN       ; block here with the line idle
        put   TX, 0         ; start bit
        now                 ; falls one cycle later; D = that cycle
        ldi   R1, 8
bit:    shr8  R0, -, TX     ; T = next data bit, LSB first; prepared on TX
        tick  R2            ; lands exactly one bit time after the previous edge
        djnz  R1, bit
        put   TX, 1         ; stop bit
        tick  R2            ; stop bit starts
        tick  R2            ; and lasts one full bit time
        goto   byte
```

The shift prepares the next bit, and `tick` names the cycle it lands. Neither depends on when the instruction issues, as long as it arrives in time. `tick R2` is D + bit time, so the edges sit on an exact grid with no drift.

### 11.4 Worked example: a pre-armed reaction

From the reaction test, also as run:

```
loop:   put   A, 1          ; prepared, nothing moves yet
        until TRIG, 1       ; the match commits it, 3 cycles pad to pad
```

The wait's match is the event, so the commit needs no instruction after the match. The thread is blocked, and the hardware does the rest.

### 11.5 Traps

Most of these cost a real bug while writing programs for this ISA.

**Pins**

- A write to a pin the thread doesn't own is dropped, with no error. Thread 0 owns everything at reset, so `put` before `pcfg` works on thread 0 and silently does nothing on the others. The assembler warns.
- `uo` pins reset low, so a program that blocks on the host before setting idle levels leaves a TX line looking like a break and a chip select asserted. Take configuration from the mailbox after the first `now`, and run `now` again after it if a `tick` follows, because D goes stale while the thread waits.
- A line that must stay recessive through reset can't be on a `uo` pin: a low `uo` holds a CAN bus dominant until firmware runs. Use `io` in `OD` with an outside pull-up.

**Waits and events**

- Prepared pins commit on any later match, including the wrong one, and after a timeout they stay prepared (section 3.3).
- A wait armed after its edge fires on the next cycle, so D is that cycle, not the edge time. On alternating data the error compounds.
- A wait armed straight after your own edge compares the old level until E + 3. After an open-drain release, `until p, 0` can match your own low and report an event nobody sent. It happens only on some phases of D, so one alignment can pass. Wait for the line to rise first. A wait straight after a wait whose match committed the edge can hit this too: it arms 1 to 4 cycles after E, and sees the old level if it arms at E + 1.
- Every slot between `now` and the first `edge` or `tick` counts. In `now`, `li R1, 0x1234`, `edge p, 0, 10`, the `li` is 2 words, so the `edge` runs 12 cycles after `now` and misses a deadline 11 cycles after it. Put set-up code before `now`.
- A level wait can't see a pulse that ended while the thread was busy. USB's end of packet needed an idle thread watching, because the last CRC step took longer than the pulse.
- A snapshot taken after an edge moves the `chg` reference, so `chg` never fires for that edge.
- `inbox` watches M[TID], so a program using it has to name that thread's word, and be assembled per thread.

**Registers, shifts and the mailbox**

- Only R0..R3 shift.
- A wait writes T: 1 on a match, 0 on a timeout or inbox exit. A shift with no input pin right after a wait feeds that T in at the vacated end. Run `clrt` first if that matters.
- `shl8` and `shr8` keep the high byte. A stale bit 8 stuck in Manchester, and turned every USB byte into a false end of packet.
- The immediate form of `edge` has a 5-bit step. A bit time above 31 cycles needs a register, or `put` plus `tick`.
- Eight mailbox words go quickly. The USB device link used all of them.

**Shift units**

- `pcfg` takes a pin off its shift output at once, even the owner's own `pcfg`, and the pin shows OUT. OUT may hold an old level, since a pin driven through the unit never needs a `put`. Claim pins first, `put` each line's idle level and commit it, then `smap` them. ABORT does the same to `uio` pins on a shift output.
- On a pin in `OD`, change the output switch only where a short low pulse does no harm: the switch may glitch the pin's enable between clock edges (section 10, item 3).
- A unit isn't reset when its thread starts, stops or halts. Run `sstop txrx` before configuring it.
- Idle levels take effect at `sstop tx` and at the end of a tail, not when written.
- Any commit of the thread starts an armed half, including a commit meant for another pin.
- An underrun ends a send early and looks like a short frame on the line. Test TXUR with `sstat` after each send.
- With HUNT, the receive half hunts again after every reception, forever. Stop it with `sstop rx`.
- `sgo rx` while the receive half is busy stays armed. With HUNT it waits until `sstop rx` clears it, or fires at a commit long after, once HUNT is cleared.
- The receive buffer holds one word. Take each word within 16 bit times.
- Status bit 8 (RXRUN) misses a frame that starts while the last word of the one before waits for the buffer. A sender that waits for a quiet line should wait for bits 8 and 9 both 0.
- `sstat` sets T for TXUR or RXOVR. A thread that both sends and receives tests bit 7 or bit 10 to know which.
- An LSB-first sync pattern sits in the top bits of SYNC.
- For the Manchester decoder, RXPER is cycles per bit, not per half bit, with RFRAC for the fraction. It takes RXPER above 255 as 255, so slower Manchester codes stay in firmware; RKIND 3 takes RXPER above 15 as 15, so slower codes take RKIND 1.
- Clear RACQ unless the reception begins with alternating bits, as 802.3's preamble does.
- A Manchester reception ends at the first bit with no clock edge. REND is only a backstop.
- RGAIN 2 and 3 filter hard and follow only a small clock offset. Use them where both clocks come from crystals, and RGAIN 0 or 1 where the sender's clock may be a percent or more off.
- A bit error in a preamble can complete the sync pattern early. That reception gives a runt of a few bits, and the next one, still in the preamble, gives the frame. Drop receptions shorter than the protocol's minimum.
- A new RKIND acts from the next reception, not the one in progress.
- A thread stop doesn't stop a send. Use ABORT to cut one.
- THALF and RKIND 3 are only as exact as the clock's duty cycle (section 7.3). A test that gives one input level per cycle puts every edge in the first half, so it times edges to the cycle even with RKIND 3, and it reads THALF's moves half a cycle late.
- With timed samples or an outside clock, the cycle that starts a reception takes no sample. With HUNT on a start bit whose edge reaches SYNC2 in the same cycle as a clock edge, that bit is lost, as an SD card's start bit would be when its output delay, with pads and board, is H - 1 to H cycles at half clock H. With an outside clock, where the line rests at a known level before the start bit, start the reception before it instead, at a commit, with no hunt and the start bit as a 1-bit sync pattern: every active edge of RCLK then samples, and the start bit completes the pattern whatever its phase. `sd_unit` starts its CMD reception at the turnaround and its DAT0 reception with `until DAT0, 1`.
- A send's first symbol lands at the commit that starts it. An SPI target with CPHA 1 moves MISO on the edge before each capture, so that first edge would move MISO on too soon: send a 1-bit dummy word first, which that edge ends.
- REND 1 ends an outside-clock reception in the first cycle with no active edge, so RXEND written as 1 when CS rises ends the frame there, with its last word partial and marked last. A thread that waits for CS can't take words meanwhile, so that end holds only for frames of up to 31 bits.
- RCNT counts to 2047. For a longer reception, such as an SD block's 4113 bits, write RXCNT with the length modulo 2048 during the reception, after RXTOT has passed that value for the last time before the end (section 10, item 22). That is the opposite of CAN's rule below.
- `sgo txrx` while the send half is busy starts the idle receive half at that commit and the send half at the first commit after it goes idle, so the two halves leave one grid with no sign. Wait for status bit 5 to clear first.

**CRC and stuffing**

- The CRC register serves one half at a time. Set CTX to 1 before a send that appends, and back to 0 before the next reception begins: a reception that begins with CTX 1 starts its CRC from the register's old value, not its initial value, so its check is meaningless. A CAN node that loses arbitration hands the register over on purpose: CTX moves to 0 after the send half's last covered bit and before the receive half's next sample, and the receive half goes on with the CRC of the bus bits so far.
- One configuration checks one CRC: CW, CINIT and POLY at a reception's start and bits, CHK at its end. A USB device writes CODING, POLYHI and CHKHI for CRC-16 after a SETUP or OUT token and before the data packet's SYNC, and back for CRC-5 after the data packet.
- On a shared line, such as USB's pair, CAN or a loopback, the receive half receives its own unit's send, because a `uio` pin reads its own drive. Take or drop those words, or `sstop rx` before the send and arm the half again after it. Otherwise the self-reception's last word waits in RXSR, the half doesn't hunt, and the reply is lost. For USB keep REOP set in the send value of CODING (0xE6EF), or the self-reception doesn't end at the EOP and runs into the reply.
- USB lets the last bit before an EOP stretch by up to six bit times (dribble). With REOP the end test drops 1 to 6 ones after the last whole byte if no seven 1s in a row result (a bit stuffing error, USB 2.0 7.1.9.1), so such a packet arrives as whole bytes with its check, as a clean one does (section 5.9.7, the byte end). Use REOP only where receptions are whole bytes: elsewhere a reception that ends in ones after a byte boundary loses them.
- An 802.3 MAC truncates a frame to whole octets before its FCS check (4.2.9). With RKIND 3, set REOP in the receive value of CODING, 0x80FF, not the table's 0x00FF: then a frame that ends with 1 to 7 bits after its last whole octet arrives as whole octets with its check (section 5.9.7). Without REOP its FCS fails.
- A word with no kind is not stuffed and breaks the run. Give every word the stuffer must see its kind, USB's SYNC and PID included (`stuff`).
- The receive CRC covers every bit from CSKIP after the sync pattern to the end, the received CRC included, so a good reception leaves the residue, not 0. With HDLC it also covers the closing flag's first six bits, so CHK is 0xD0F9, not 0x1D0F.
- An HDLC reception's last word ends with the closing flag's first six bits, 011111. Drop them. The closing flag ends the reception and the half hunts again, so it receives one frame per hunt: with a falling-edge hunt and SYNCLEN 8, a run of flags gives one six-bit reception for every three flags, and a frame arrives only if its opening flag falls on that cycle, as after 3, 6 or 9 flags or after a gap of ones. Drop the six-bit receptions. A sender that puts 1, 2, 4 or 5 flags between frames, or idles on flags, loses frames.
- The end flags belong to the last word. Read them with `srd Rd, end` after the `rxw` that sets T and before the next `rxw`.
- `srd` reads the register as it is: the remainder in wire order, left-aligned, neither reflected nor complemented.
- An idle send half used as a CRC engine needs words of kind `crc`: after a word of kind `fcs` the register holds its initial value again. Queue two words before its start, or a feed loop of 4 slots misses the first word boundary and the send underruns.
- POLY is the reciprocal of the polynomial a standard names for a Fibonacci PRBS or scrambler, and the LFSR needs CINIT 1 (section 5.9.7).
- The end test needs RCLK set to the second line, and works only with timed samples.
- A stuff violation on USB or CAN ends a reception early, and the line goes on: with HUNT, the half hunts again in the middle of the frame. Drop receptions with end flag 1 set, and on CAN expect the error frame next.
- On CAN, with HUNT, the half hunts again as soon as RCNT ends a frame, so a stuff bit after the CRC, or another node's ACK, can start a short reception that ends on a stuff violation in the recessive tail. Drop it: it has end flag 1 set.
- On CAN, write RXCNT before RXTOT reaches the new count. `rx_bit` ends a reception only when RXTOT equals RCNT, so a late write never ends it: it runs into the recessive tail, ends on a stuff violation and fails its check. With MSB-first words the DLC is complete only with the second word, at bit 32, 2 bit times before the end of a frame with no data, or 3 when RCNT ends at the CRC delimiter (section 5.9.7).
- With RESYNC alone every edge of RPIN pulls the sample countdown, rising and falling alike. CAN resynchronizes only on recessive-to-dominant edges, so for CAN set RACQ too, with HLVL 0: then only those edges pull (5.9.6). Without RACQ, a transceiver whose rising and falling delays differ moves the sample point by that difference after each dominant-to-recessive edge, and two other transmitters out of step in arbitration stretch every dominant bit by their offset, which moves the sample the same way: with RGAIN 0, a stretch longer than the bit minus FIRST moves the next sample into the next bit, 25 cycles at 500 kbit/s with FIRST 75.
- A data packet with no data, such as a USB zero-length packet, can't use `fcs`, which covers its own word: send the CRC of no data as a word of kind `stuff`, 0x0000 for USB's CRC-16.
