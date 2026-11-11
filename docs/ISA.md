# ISA v0.1

Draft, 2026-11-11. This instruction set can change until the hardware freezes.

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

All state is flops that update on the rising edge ending a cycle. A value written in cycle *c* is visible from cycle *c + 1*. Only the rising clock edge is used.

### 3.2 Order inside one cycle

A simulator does these steps for cycle *c*, in this order. Every read of a flop sees its value from the start of the cycle, with one exception: `stop()` reads P as step 2 left it. Inside one step, a block also sees what it wrote itself a line earlier.

1. **Pads.** Compute the pad outputs from flop state (section 5.2), then take the pad inputs for cycle *c*. A lane in bypass has no flop in its path, so it depends on `ui_in` too. A simulator either takes inputs first or shows `ui_in` of cycle *c - 1*, as section 10 allows.
2. **Events.** For every thread, set HIT to whether TB = D xor 0x8000. Then, for every thread with P = 1, run `events()` (section 3.4).

   2b. **Shift units.** Every thread runs `tx_step()`, then `rx_step()` (section 5.9).

3. **Execute.** The thread owning cycle *c* runs `slot(P_start)` (section 3.6), where P_start is its P flag from before step 2.
4. **Memory.** The thread whose memory cycle is *c* runs `memory()` (section 4.2).
5. **Update.** Every thread runs `update_stale()` (section 3.5), and CMT := 0. Then `SYNC3 := SYNC2`, `SYNC2 := SYNC1`, `SYNC1 := {uio_in, ui_in}`, and *c* advances.

A host command takes effect after step 5 of its cycle. A memory access and the read of RDREG come later (section 9.1).

Step 2b reads CMT, which step 2 sets in the same cycle. Like HIT, CMT lasts one cycle, so it isn't really state. Otherwise step 2b reads only values from the start of the cycle. That includes configuration: the unit sees an `scfg` from the cycle after it, even one in the same cycle as a start, a symbol or a sample.

**The two halves of a unit share no flop in step 2b.** Each half writes only its own state, and reads only that, CMT, the configuration and the synchroniser. The CRC register belongs to the half that CTX names in that cycle: the send half reads and writes it only with CTX 1, and the receive half only with CTX 0 (section 5.9.7). So the order of `tx_step` and `rx_step` can't change a result, and an implementation may run them at once. Inside one half the code runs in order, so a line reads what an earlier line of the same step wrote.

**Step 3 reads the unit's flops as they were at the start of the cycle, before step 2b.** That is the first rule of section 3.2, and it matters here because step 2b, unlike the steps before it, writes flops that step 3 reads in the same cycle: TXFULL (`tx_load`), RXFULL, RXBUF, RXBN, RXBL and RXBE (`rx_deliver`), CRC (`crc_init`, `crc_bit` and the append), and every flop of the status word, TXUR, RXOVR, RXSEEN, RXRUN, RXEP, LA and LB among them. So a `txw` in the cycle the unit takes the buffer still blocks, an `rxw` in the cycle a word arrives still blocks, an `sstat` in the cycle a flag sets reads it as 0 and leaves it set, and an `srd` in the cycle a bit enters the CRC register reads the register as it was before that bit. A simulator that runs the steps one after another on live state must keep these flops' values from before step 2b for step 3, or run step 3 on a copy of the start-of-cycle state and apply its writes after step 2b's.

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

Pads have no pulls and no hysteresis. `uo` pads can't tri-state. Only `uio` pads can do open drain, turnaround or read-back. A `uio` input reads the pad even while the chip drives it, so a thread can read another thread's output with no outside wire, which gives a loopback self-test.

`LA[t]` and `LB[t]` are the shift outputs of thread *t*'s unit. SEL[p] is pin *p*'s output switch. A shift output replaces OUT and nothing else:

- **Open drain** works unchanged. A shift output on a pin in `OD` releases the pin on 1 and pulls it low on 0, and the pin still can't drive high.
- **OE** stays firmware's. A shift output on a `uio` pin in `PP` drives only while OE is 1, so a turnaround is still a committed `oe`.
- **Bypass wins.** On a pin in `BYP`, the output switch has no effect until the bypass switch is committed off.
- **Idle.** A pin switched to a shift output shows that output's level: the idle level when the send half is idle (after `sstop tx` or a tail), or the end level during a tail. After reset every shift output is 0.
- **Inputs** need no switch. Any unit can read any input pin, the pins of its own outputs included: a `uio` pin reads its own drive (section 5.2), so a shift output looped back to its own unit is a self-test with no outside wire. A change of a shift output written in cycle *w* is on the pad at *w* + 1 and in SYNC2 at *w* + 3.

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

Inputs pass a two-flop synchroniser with no glitch filter. A pad change in cycle *p* is in SYNC2 at *p + 2*. A commit in cycle E reaches the pad at *E + 1*.

- **Pre-armed reaction.** Prepare pins, then wait: the match commits them. An input edge at the pad moves an output 3 cycles later, exactly, on every thread and phase run.
- **Timed reaction.** After a match in cycle E, `edge p, v, n` fires at E + n. The smallest step on time on every alignment is n = 5, which is 8 cycles pad to pad with no jitter.
- **Sampling.** The start edge and later samples pass through the same synchroniser, so the 2 cycles cancel. After a wait matches, `tick n` snapshots what the pad showed exactly *n* cycles after the edge.
- **Sampling on your own clock edge.** The event that commits a clock edge at E also snapshots, but the pad moves at E + 1 and S holds the pad at E - 2, so the sample looks 3 cycles ahead of the edge at the pad. An SPI target's clock-to-out therefore has to fit in the half period minus 3 cycles, which is 6 cycles at half period 9.

### 5.8 Bypass

Mode `BYP` gives a pad-to-pad path with no flop in it. The switch is committed state like any other pin, so it changes at an event, on an exact cycle. There is no crossbar: pins are used where they are.

### 5.9 Shift units

#### 5.9.1 Overview

Each thread has one shift unit with a send half and a receive half.

- The **send half** turns 16-bit words into a stream of symbols on two **shift outputs**, A and B. Each symbol lasts a set number of cycles, or ends at an edge of an outside clock. An output's value per symbol comes from a small table, so one mechanism gives NRZ, NRZI, Manchester, biphase mark, return to zero, a data line with its clock, and a complement for a differential pair.
- The **receive half** turns one input pin into 16-bit words. It samples at a set period, decodes a Manchester-family code against a predicted clock edge that every input edge corrects, or samples at the edges of an outside clock. It can wait for a sync pattern before it starts a word, and it ends a reception after a set number of bits, a set time without edges, or, for Manchester, the first bit with no clock edge.
- A **CRC register** of 32 bits serves one half at a time (section 5.9.7). As a CRC of any width from 1 to 32 bits and any polynomial, it takes one bit per data bit in wire order, appends the CRC after a send's covered words, and checks a reception against a set residue. As an LFSR it scrambles a send or descrambles a reception, which also gives a PRBS. A **bit stuffer** inserts a bit after a set number of equal bits on send, and the **destuffer** removes it on receive and ends a reception on a stuff violation.

A unit starts only at a commit of its own thread, so its first symbol lands on the exact cycle of an event (section 3.3). From then on it runs by itself on a cycle grid fixed by that commit. The thread only fills and empties its buffers, through instructions that block like `send` and `recv`. A pin follows a shift output only if its owner switches it to that output, and the switch is committed like any other pin state (section 5.4). In this section "lane" keeps the meaning of section 5.2, a bypass lane.

The unit counts cycles per symbol. It never changes when a thread issues instructions, so every thread still runs one instruction every 4 cycles. Units share nothing with each other, so one thread's unit can't change another thread's timing.

#### 5.9.2 State

**Per-thread state that firmware or the host can see:**

| State | Bits | Purpose |
|---|---|---|
| CFG[0..15] | 16 each | Configuration slots, written by `scfg` (section 5.9.3). Reserved bits aren't stored and read 0 |
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
| RXG | 5 | Reference edges since the start, counting the start as 1, up to 16. It sets the gain and ends RACQ's wide window |
| RXOPEN | 1 | The Manchester decoder's window is open: the predicted clock edge has had no edge yet |
| RXCAND | 1 | An edge came in the open window before the predicted clock edge |
| RXIC | 16 | Cycles since the last reference edge, or, for the Manchester decoder, since the last bit. It stops at 65535 |
| RXLAST | 1 | The last level sampled, for NRZI |
| RXWIN | 1 | An edge other than the clock edge came since the last clock edge, for RDIFF |
| RXBUF, RXBN, RXBL, RXFULL | 16, 5, 1, 1 | The received word waiting for `rxw`, its bit count, its last flag, and whether it is there |
| RXPN | 5 | Bit count of the word `rxw` took last |
| RXOVR | 1 | Sticky. A full word was lost because the buffer was full |
| RXSEEN | 1 | Sticky. A reception started |
| RXRL, RXRV | 3, 1 | The destuffer's run: equal bits in a row, 0 to 7, and their value |
| RXVIO | 1 | The destuffer has seen a stuff violation, and the next bit ends the reception |
| RXCOV | 1 | The CRC covers the bits of this reception from here on |
| RXEF, RXBE | 3 each | End flags (section 5.9.7) of the reception in progress, and of the word in the buffer |
| CMT | 1 | The thread's prepared pins commit in this cycle. Lasts one cycle, like HIT |

**Global state:**

| State | Bits | Purpose |
|---|---|---|
| SYNC3 | 16 | SYNC2 one cycle older, so that an edge on any pin is `SYNC2 ^ SYNC3` |
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
| MAPA | 1 `TXMAP` | 7:0 | Output A's code for symbols 0 to 3, two bits each, symbol 0 in bits 1:0 |
| MAPB | 1 | 15:8 | Output B's codes, the same way |
| TPER | 2 `TXPER` | 15:0 | Cycles per symbol. 0 acts as 1 and ignores TFRAC |
| TAIL | 3 `TXTAIL` | 7:0 | Cycles of the end levels after the last symbol. 0: no tail |
| TFRAC | 3 | 15:8 | Fraction of a cycle added to each symbol, in 256ths. TPER 0 ignores it |
| RKIND | 4 `RXMODE` | 1:0 | 0: timed samples. 1: Manchester decoder. 2: outside clock. 3: reserved, acts as 0. A reception keeps the decoder it started with |
| RMSB | 4 | 2 | 1: the first bit of a word ends up in bit n - 1. 0: in bit 0 |
| RINV | 4 | 3 | 1: invert every decoded bit |
| RDIFF | 4 | 4 | Timed and outside clock: 1 decodes NRZI, where a change is 0. Manchester decoder: 1 makes the bit "an edge other than a clock edge came between the last two clock edges" |
| HUNT | 4 | 5 | 1: after its commit, and after each reception, the receive half waits for an edge to HLVL on RPIN and starts there |
| HLVL | 4 | 6 | The level that edge goes to |
| RFALL | 4 | 7 | Outside clock: 1 samples at a falling edge of RCLK, 0 at a rising one |
| RPIN | 4 | 11:8 | Input pin of the data, numbered as `ipin` |
| RCLK | 4 | 15:12 | Input pin of the outside clock, numbered as `ipin`. With REOP, timed samples read it as the second line of the end test (section 5.9.7) |
| RPER | 5 `RXPER` | 15:0 | Timed: cycles per sample. Manchester decoder: cycles per bit, where 256 and up are reserved and act as 255. Both add RFRAC. 0 acts as 1 and ignores RFRAC |
| FIRST | 6 `RXFIRST` | 15:0 | Timed: cycles from the start to the first sample. With RESYNC, an edge pulls the next sample toward FIRST cycles after it, all the way with RGAIN 0. 0 acts as 1 |
| REND | 7 `RXEND` | 15:0 | Cycles since the last reference edge, or for the Manchester decoder since the last bit, that end a reception. 0: never |
| RCNT | 8 `RXCNT` | 10:0 | Bits after the sync pattern that end a reception. 0: no limit |
| SYNCLEN | 8 | 15:11 | Bits in the sync pattern, 0 to 16. 0: no sync, words start at once. 17 to 31 are reserved and act as 16 |
| SYNC | 9 `RXSYNC` | 15:0 | The sync pattern, laid out as a received word (section 5.9.6) |
| RFRAC | 10 `RXOPT` | 7:0 | Fraction of a cycle added to RPER, in 256ths |
| RESYNC | 10 | 8 | Timed: 1 makes every edge of RPIN pull the sample countdown toward FIRST |
| RGAIN | 10 | 10:9 | How far an edge corrects the receive phase: by 2^-k of its error. 0: k = 0, a full correction. 1, 2, 3: k = 1 up to and including the third reference edge after the start, then k = 2, 4 or 5 |
| RACQ | 10 | 11 | Manchester decoder with RGAIN 1 to 3: 1 opens the window half a bit before the predicted clock edge, not a quarter, until RXG reaches 16. For receptions that begin with alternating bits. RGAIN 0 ignores it |
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
| REOP | 15 | 15 | Timed samples: 1 ends a reception at a sample where RPIN and RCLK both read 0, such as a USB SE0, and that sample gives no bit. The send half ignores it |
<!-- shift-fields-end -->

TXMODE bit 15 is reserved. A **reference edge** is a clock edge for the Manchester decoder, any edge of RPIN for timed samples, and an active edge of RCLK for an outside clock.

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

For modes 2 and 3 set IDLEB to 1 and swap the clock codes. As TXMAP words: 802.3 Manchester with its complement is 0x0401, SPI mode 0 is 0x0E00, and a USB NRZI pair is 0x0101.

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

`KEEP[k]` is the mask of the bits that the field table's rows define in slot *k*, leaving out any row marked reserved.

#### 5.9.5 Send

Each cycle, after the events of step 2, every thread runs `tx_step()`.

```
tx_step():
    if TXRUN:
        tx_run()
    elif TXTL:
        tx_tail()
    else:
        if TXN == 0 and TXFULL:
            tx_load()
        if TXARM and CMT:
            TXARM := 0
            if TXN == 0:
                TXUR := 1
                tx_end()
            else:
                TXRUN := 1
                TXSYM := 0
                TXACC := 0
                tx_begin()
                tx_symbol()

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
    LA := IDLEA
    LB := IDLEB
```

What follows from the code:

- **The grid.** A symbol written in cycle *w* is on the pad from *w* + 1 (section 5.7), for exactly TPER cycles plus any fraction carry. TPER 0 gives one cycle and leaves TXACC alone, whatever TFRAC is. The commit at E writes the first symbol, so with no fraction, symbol *k* of a send is on the pad from E + 1 + *k* x TPER, and a word of *n* bits lasts *n* x (SYMS + 1) x TPER cycles, with TPER 0 counted as 1 in both. A stuff bit or an appended CRC bit (5.9.7) is a bit like any other: it lasts SYMS + 1 symbols on the same grid.
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
    rx_cbegin()
    if RXKIND == 1:
        RXPH := 256
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
        rx_manch(e, p)
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
            rx_end()
        else:
            rx_sample()
    else:
        RXPH := RXPH - 256
    if e:
        RXIC := 0
        if RESYNC:
            rx_pull((FIRST - 1 if FIRST else 0) << 8)
            rx_gear()

rx_manch(e, p):
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
            rx_bit(RXWIN if RDIFF else bit(SYNC2, RPIN))
            RXWIN := 0
            if e:
                rx_pull(0)
            rx_gear()
    elif RXRUN and e:
        RXWIN := 1
        rx_pull(p >> 1)
    RXPH := RXPH + 256

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
            RXVIO := 1
            RXEF := RXEF | 2
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

- **Start.** Without HUNT, a reception begins at the commit. With HUNT, the commit starts the hunt, and a reception begins in the first later cycle where RPIN moves to HLVL in SYNC2. Either way, the start cycle *s* counts as a reference edge, and the reception keeps the decoder that RKIND named in cycle *s*. For the Manchester decoder, *s* is the first clock edge: it gives the first bit at once (no bit with RDIFF) and puts the next predicted clock edge one bit later.
- **Period.** *p* is RPER x 256 + RFRAC, the period in 256ths of a cycle. RPER 0 gives *p* = 256, one cycle, and ignores RFRAC, so a timed half then samples every cycle and RXPH can't run away. The Manchester decoder takes RPER above 255 as 255, so its *p* stays below 65536 and RXPH stays between -32768 and 49407 there.
- **Timed samples.** RXPH counts down by 256 a cycle, and a sample is taken from SYNC2 in the cycle where it is below 256. Sample *j* is in cycle *s* + FIRST + *j* x RPER, plus the carries of RFRAC. The input and the start edge pass the same synchroniser, so, as in section 5.7, FIRST counts from the edge at the pad. With RESYNC, every edge of RPIN pulls the countdown toward FIRST, by the gain below. With RGAIN 0 that restarts it, so the next sample is FIRST cycles after the edge. If an edge and a sample fall in one cycle, the sample is taken first.
- **Manchester decoder: phase.** RXPH is the time since the predicted clock edge and grows by 256 a cycle. Let *w* be *p*/4, or *p*/2 with RACQ and RGAIN 1 to 3 until RXG reaches 16. When RXPH reaches *p* - *w*, the window for the next clock edge opens and RXPH drops by *p*, so it counts from -*w* to the predicted clock edge at 0. The next window opens when RXPH reaches *p* - *w* again: *p* - *w* after the predicted clock edge, as the edges since have moved it.
- **Manchester decoder: the bit.** In an open window, an edge before the predicted clock edge is a candidate, and a later one replaces it. At the predicted clock edge, the first cycle with RXPH at 0 or more, a candidate gives the bit: the level of RPIN in that cycle. With no candidate, the first edge from there on gives the level after it. Either way the window closes. So a clock edge is taken however late it comes before the next window, and a late bit-boundary edge misleads the decoder only if the clock edge after it is late too. For IEEE 802.3 a rising clock edge is a 1, and RINV gives the other convention. With RDIFF the bit is RXWIN: 1 if an edge other than a clock edge came between the last two clock edges. That decodes biphase mark, whose clock edges are the bit starts, one bit late, and, with RINV, differential Manchester, whose clock edges are mid-bit.
- **Manchester decoder: correction.** Every edge pulls the phase toward the place it should have: an edge in the window toward the predicted clock edge, any other edge toward half a bit after it, where a bit-boundary edge falls. The start edge sets the phase alone.
- **Gain.** A pull moves RXPH by 2^-k of its error, rounded down. With RGAIN 0, k is 0 and every pull is a full correction. With RGAIN 1 to 3, k is 1 while RXG is 3 or less, that is up to and including the third reference edge after the start, and then 2, 4 or 5. So the first edges set the phase quickly, and later ones are averaged over about 2^k edges. Random Manchester data has about 1.5 edges per bit, so k = 4 averages over about 11 bits. A larger k filters more jitter and follows less offset between the sender's clock and ours.
- **Outside clock.** Each active edge of RCLK samples RPIN in the same cycle. Both pins pass the same synchroniser, so they stay aligned to within the one cycle of phase uncertainty in section 10, item 2.
- **Sync.** Until the pattern is found, bits shift through RXSR as a history and no word builds. The pattern is laid out as the received word it would form. With RMSB = 0 it sits in the top SYNCLEN bits: the 802.3 start frame delimiter alone is 0xD500 with SYNCLEN 8, and the last preamble byte with the delimiter is 0xD555 with SYNCLEN 16. With RMSB = 1 it sits in the bottom SYNCLEN bits. The bit that completes the pattern isn't kept. The next bit starts the first word.
- **Words.** A word of *n* bits holds them in bits *n* - 1..0, with higher bits 0. Every 16 bits make a word. If the buffer is still full, the new word is lost and RXOVR sets. A word that completes in the same cycle as an `rxw` counts as lost, because the buffer was full at the start of the cycle.
- **End.** A reception ends after its RCNT-th bit, after REND cycles with no reference edge (for the Manchester decoder, with no bit), when a Manchester window closes with no edge in it, one bit after a stuff violation, at a timed sample that sees the end test with REOP (both in 5.9.7), or at `sstop rx`. A Manchester window closes empty in the cycle the next window would open: 7/4 bits after the last predicted clock edge, or 3/2 bits while RACQ's wide window is in use. A reception that found its pattern and ended any way but `sstop rx` then gives exactly one word marked last, holding the bits since the last full word: 0 to 15, or up to 16 when RCNT ends it. That word is never lost: it waits in RXSR, with RXEP set, until the buffer is empty, and only then does a HUNT receive half start hunting again. A reception that never found its pattern gives no word, and only RXSEEN shows it. `sstop rx` drops the reception, its last word and the buffer.
- **`sgo rx` while busy** stays armed, and status bit 15 shows it. `rx_step` takes the arm only when the half is idle: no reception, no hunt and no last word waiting. Without HUNT, the first commit after the reception ends starts the next one. With HUNT the half hunts again after every reception, so the arm waits until `sstop rx` clears it. If HUNT is cleared first, the first commit after the half goes idle starts a reception, whatever that commit was for.
- **Scope of the Manchester decoder.** It receives Manchester-family codes of up to 255 cycles per bit, down to about 195 kbit/s at 50 MHz, that mark a frame only by its start and its end. A frame ends at the first bit with no clock edge, and no status bit reports a code violation. So codes that frame by violations can't be received with it: AES3 and S/PDIF preambles, the MIL-STD-1553 sync, Token Ring's J and K. Those, and slower Manchester codes, are left to timed samples and firmware.
- **Exact.** Every decision falls on a cycle that RXPH, the synchroniser and whole 256ths fix, so two simulators agree on every cycle.

**Implementation note.** This is not normative. The code fixes what happens in each cycle, not how. Written out literally, `rx_manch` chains about five carry chains in one cycle, too many for the slow corner at 50 MHz. The structure below gives the same result in every cycle with at most two:

- **Flops for values that depend only on configuration.** An implementation may hold such a value in flops of its own. Here they are *p* - *p*/4 and *p* - *p*/2, the two places where a Manchester window opens, 16 bits each. They are loaded in the cycle of the `scfg` that writes RXPER or RXOPT, from the value being written and the other slot's stored value, and reset to their values for the reset configuration, where *p* is 256: 192 and 128. The unit reads configuration as it was at the start of a cycle (section 3.2), and `scfg` writes at the end of its cycle, so in every cycle these flops match the configuration the unit reads, and no program can tell them apart from the code. In particular, an `scfg` in the same cycle as a receive start or a sample changes neither: in that cycle the start and the sample use the old configuration, in the code and in the flops alike, and from the next cycle both use the new one.
- **Manchester decoder.** Form RXPH - *p* and RXPH - *p*/2 beside the window compare. When the compare and the sign are known, select the in-window or the boundary error, then the gain's fixed wiring, and fold the + 256 into the one add that follows. *p* itself needs no flop: it is RPER and RFRAC with the rules for 0 and for 256 and up. The arithmetic fits 17 bits (the Period bullet), sign-extended into RXPH, because RXKIND keeps a timed RXPH out of it.
- **Timed samples.** Form the error, RXPH - 256 + *p* (when sampling) minus FIRST's target, beside the countdown, then select the gain's wiring and subtract: two carry chains on 25 bits.
- **Cost.** 32 flops per unit for the two window points, 128 for four units. RXKIND's 2 flops are in the state table.
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

**Coverage on receive** is set by CSKIP. The register starts from its initial value at the reception's start, and takes the bits from bit CSKIP after the sync pattern, counting from 0, to the end of the reception. The sync pattern is never covered, so the CRC leaves out Ethernet's preamble and SFD, USB's SYNC and an HDLC opening flag, and CSKIP leaves out the bits after it that the CRC doesn't cover, such as USB's 8-bit PID. CAN has no sync pattern (SYNCLEN 0), so its CRC starts at SOF with CSKIP 0. The CRC ends where the reception ends, which 5.9.6 defines: the Manchester end for Ethernet, the end test below for USB, the closing flag for HDLC, and RCNT for CAN, which firmware writes once the DLC has arrived.

**Exact.** A bit enters the register in the cycle its last symbol ends on send, and in the cycle it is decoded on receive. Stuff bits and appended bits take a bit time each on the same grid, so the line carries every bit on a cycle the configuration and the data fix.

**The append.** After the last bit of a word of kind `fcs`, and any stuff bit it needs, the send half sends *w* more bits, each bit 31 of the register XOR CXOR, with the word's line code. After each one the register shifts left by one, with no feedback. When the last one ends, the register takes its initial value again, so the next frame of the same send starts its CRC from it. The stuffer counts the appended bits and adds any stuff bit they need. Then the send goes on as at any word end: the next word, the tail, or an underrun. The append uses the register only while CTX is 1: if CTX moves to 0 during it, the remaining appended bits are CXOR and the register is left to the receive half (section 10, item 17).

**The check.** At the end of a reception, with CMODE 1 and CTX 0, end flag 0 sets if CRC equals CHK, which the code writes `(CHKH << 16) | CHKL`. A good reception leaves a residue that doesn't depend on the data or the initial value: 0 when the sender's CXOR is 0, and with CXOR 1 the register that *w* ones give from 0. The table below has CHK for each standard. HDLC's check includes the first six bits of the closing flag, 011111, which reach the CRC before the destuffer knows they belong to a flag, so its CHK is its residue 0x1D0F fed those six bits: 0xD0F9. The check covers every covered bit up to the end, so a bit the protocol lets a sender add before its end marker, such as USB's dribble bit, makes a good packet fail (section 11.5).

**Parity.** A width of 1 with POLY *x* + 1, CW 0 and POLYH 0x8000, and CINIT 0, makes the register the running XOR of the covered bits. Then the append is a parity bit, even with CXOR 0 and odd with CXOR 1, and the check is a parity check, with CHK 0 for even parity and 0x80000000 for odd. CINIT 1 starts the register at 1, which swaps even and odd.

**Six standards.** CHK is used on receive only. The check value is the CRC of the nine ASCII bytes `123456789`, sent as the unit appends it and read in the standard's own bit order; each is the catalogue's value.

| Standard | *w* | POLYH, POLYL | CINIT, CXOR | CHKH, CHKL | Bits | CODING, send and receive | Check value |
|---|---|---|---|---|---|---|---|
| Ethernet FCS, CRC-32 | 32 | 0x04C1, 0x1DB7 | 1, 1 | 0xC704, 0xDD7B | LSB first | 0x02FF, 0x00FF | 0xCBF43926 |
| USB data, CRC-16 | 16 | 0x8005, 0 | 1, 1 | 0x800D, 0 | LSB first | 0xE6EF, 0xE4EF | 0xB4C8 |
| USB token, CRC-5 | 5 | 0x2800, 0 | 1, 1 | 0x6000, 0 | LSB first | 0xE6E4, 0xE4E4 | 0x19 |
| HDLC FCS-16 (X.25, IBM-SDLC) | 16 | 0x1021, 0 | 1, 1 | 0xD0F9, 0 | LSB first | 0x56EF, 0x54EF | 0x906E |
| CAN, CRC-15 | 15 | 0x8B32, 0 | 0, 0 | 0, 0 | MSB first | 0x5A8E, 0x588E | 0x059E |
| 1-Wire, CRC-8 (MAXIM-DOW) | 8 | 0x3100, 0 | 0, 0 | 0, 0 | LSB first | 0x0287 on an idle send half | 0xA1 |

The CODING values set CMODE 1 and CTX, and the stuffer: ones with N = 6 for USB, ones with N = 5 for HDLC, equal bits with N = 5 for CAN. USB's values also set REOP. Only the receive half reads it, so the send value keeps it set: a receive half that hunts on the same pins during the send then ends its reception of that packet at the EOP (section 11.5). One configuration checks one CRC, so a USB device rewrites CODING, POLYHI and CHKHI between a token and a data packet.

**End flags.** Three flags travel with a reception's last word. End flag *k* is bit *k* of the value `srd Rd, end` reads:

| Bit | Set when |
|---|---|
| 0 | The CRC check passed: CMODE 1, CTX 0 and CRC equal to CHK at the end |
| 1 | A stuff violation ended the reception |
| 2 | The bit after the violation continued the run |

They form in RXEF during the reception, move to RXBE with the last word, and to RXPE when `rxw` takes it, so `srd Rd, end` gives the end flags of the word `rxw` took last: a last word's flags, or 0 for a full word. Read them after the `rxw` that sets T and before the next `rxw`.

**Stuffing.** SMODE picks the run and SN its length N. With SMODE 2, an SN of 0 or 1 acts as 2: after a stuff bit the run is 1, so with N = 1 the receiver would take the next bit for a stuff position where the sender puts a data bit. The run is the number of bits in a row, up to 7, that equal the last bit; with SMODE 1 only ones count, and a 0 makes it 0.

- **Send.** After a bit that is stuffed (a word of kind 1 to 3, or an appended CRC bit) and makes the run N, a stuff bit follows: the complement of the run's value, so a 0 after ones. A stuff bit is not in the CRC, but it counts in the run: with SMODE 2 it starts a run of 1, as CAN counts it, and with SMODE 1 it ends the run of ones. A stuff bit never calls for another. A stuff bit due after a word's last bit is sent before the CRC, the next word, the tail or the end, as USB, HDLC and CAN require.
- **Receive.** The destuffer counts the run from the reception's start, the sync search included, because USB counts its SYNC's last 1 in the first run. It acts only once the sync pattern is found, so an HDLC opening flag, which is the sync pattern, is no violation. Then the bit after a run of N is the stuff position. The complement there is a stuff bit, and is dropped. The run's own value there is a stuff violation: that bit is dropped too, end flag 1 sets, and the next bit ends the reception, setting end flag 2 if it continues the run. Neither of those two bits goes to the CRC or into a word. The bit count of words and RCNT count only the bits that remain.

What a violation means, per standard:

- **USB** (SMODE 1, N = 6): 7 ones is a bit-stuff error. The reception ends one bit later with end flag 1 set; drop the packet. A good packet ends at the end test.
- **CAN** (SMODE 2, N = 5): 6 equal bits is a stuff error. The reception ends one bit later with end flag 1 set. An error frame, if the node sends one, is firmware's.
- **HDLC** (SMODE 1, N = 5): 6 ones is a flag or an abort, and the next bit tells which. A 0 makes it a flag: the frame ended normally, with flag 1 set and flag 2 clear, and its last word holds the flag's first six bits, 011111, after the FCS, which firmware drops. A 1 makes it an abort, with flags 1 and 2 set: drop the frame.
- **HDLC frames one after another.** The closing flag ends the reception at its last bit. With HUNT the half then hunts for an edge, and a new reception needs a whole flag, its sync pattern, after that edge. So a run of flags gives one reception for every few flags, not one per flag. With a falling-edge hunt and the sync pattern 0x7E00 with SYNCLEN 8, the next falling edge is the next flag's last bit, and each reception uses three flags: the one that closes it, the one the hunt starts in, and the one that opens the next. A frame is then received only if its opening flag falls on that cycle, as when 3, 6 or 9 flags separate it from the frame before; with 1, 2, 4 or 5 flags it is lost. A sender that idles on flags gives a six-bit reception (011111, end flag 1) for every three flags, and its next frame arrives only if the idle flags end on the cycle. With SYNCLEN 7 and a rising-edge hunt the cycle is two flags. Frames that share one flag never both arrive, and a frame after a gap of ones (mark idle) always starts a reception.

**The end test.** With REOP, a timed sample at which RPIN and RCLK both read 0 in SYNC2 ends the reception, and that sample gives no bit. For a USB receiver RPIN is D+ and RCLK is D-, so the test sees the SE0 that starts an end of packet, and the CRC ends at the packet's last bit. Only timed samples (RKIND 0) test it.

**LFSR mode** (CMODE 2). The register runs as a Galois LFSR of POLY: each covered bit advances it with bit 31 alone as the feedback. On send, a covered data bit goes out XOR bit 31; on receive, a covered bit is XORed with bit 31 before it goes into the word. A sender and a receiver that start from the same initial value, at a send's first covered bit and a reception's first covered bit, undo each other: an additive scrambler. Words of zeros send the LFSR's own sequence, a PRBS, whose period is 2^*w* - 1 bits for a primitive POLY of width *w*. The LFSR needs CINIT 1: from 0 it stays at 0. Kind `fcs` acts as `crc`, and nothing is appended or checked. Self-synchronizing scramblers, which feed back the line bits, aren't offered.

**POLY for a standard's PRBS or scrambler.** POLY, with its *x*^*w* term, is the characteristic polynomial of the sequence the line carries: bit *n* + *w* is the XOR of the bits *n* + *i* for every lower term *x*^*i* of POLY. Standards define their PRBS generators and additive scramblers as a Fibonacci register named by the polynomial *x*^*w* + *x*^*m* + 1, whose stages *m* and *w* feed back, so bit *n* is the XOR of bits *n* - *m* and *n* - *w*. That sequence's characteristic polynomial is the reciprocal, *x*^*w* + *x*^(*w* - *m*) + 1, and that is the POLY to set; the named polynomial set as POLY sends the same sequence in reverse time, which a standard's checker or descrambler can't lock to. So PRBS7 (*x*^7 + *x*^6 + 1) is POLYHI 0x0600, PRBS9 (*x*^9 + *x*^5 + 1) 0x0880, PRBS15 (*x*^15 + *x*^14 + 1) 0x0006, and the 802.11 scrambler (*x*^7 + *x*^4 + 1) 0x1200, each with POLYLO 0.

**An idle send half as a CRC engine.** With no pin switched to its outputs, a send at TPER 0 takes one bit per cycle: 16 cycles for a word of kind `crc`, against 8 slots, 32 cycles, per bit for `crc32.s`. Firmware reads the register with `srd` once status bit 5 is clear. For 1-Wire's ROM code, it pushes the 64 bits and checks that both halves read 0. Use words of kind `crc` only: a word of kind `fcs` appends the CRC and then loads the initial value, so the register no longer holds the CRC.

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

rx_run_len(v):
    if SMODE == 1 and v == 0:
        RXRL := 0
    elif RXRL == 0 or v != RXRV:
        RXRL := 1
    elif RXRL < 7:
        RXRL := RXRL + 1
    RXRV := v
```

`tx_done` runs at the end of every bit of a send (5.9.5), and `rx_bit` (5.9.6) holds the destuffer's steps and the receive half's call of `crc_bit`.

**Implementation note.** This is not normative. On receive the destuffer's decision adds about two gates between a decoded bit and the word insert, because whether a bit is at the stuff position depends only on flops (RXRL, RXSYN, SMODE, SN). The CRC's next value runs beside the word path and doesn't lengthen it, and in LFSR mode the descrambling adds one XOR. On send the choice of a bit's source, data, stuff or CRC, adds a 3-to-1 multiplexer in front of the output tables. Only the half that CTX names uses the register in a cycle (section 3.2), so the register's next value is the send half's or the receive half's, chosen by CTX.

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
| Timed receive samples | The cycle, relative to the start | No RESYNC. With RESYNC and RGAIN 0, relative to the last edge |
| Manchester decoder bits and ends | The cycle | Given the values in SYNC2. Which cycle first samples a pad edge is section 10, item 2 |
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

Timing is in cycles, not nanoseconds, so 40 to 45 MHz silicon works by changing constants, and programs that take their bit time from the host need no change at all. UART, SPI, I2C write, WS2812 and 1-Wire ran at 40 and 45 MHz with only constants changed. Two protocols depend on the clock: USB low speed needs 33 cycles per bit, so 50 MHz, and CAN at 500 kbit/s needs at least 46 MHz to receive and 45.5 MHz to transmit.

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

These follow from one instruction per 4 cycles. The shift units of section 5.9 move bit timing into hardware, so they lift the limits that come from bit timing. Their limits are not measured yet.

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
| SYNC1, SYNC2, SYNC3 | 0 |
| OUT, SHD | 0 |
| OE, SHOE, OD, BYP, SHBYP | 0 |
| SEL, SHSEL | 0, so every pin shows OUT |
| OWN | Every pin owned by thread 0 |
| BUSY | 0 |
| CFG[0..15] of every thread | 0, so the CRC register and the stuffer are off |
| Every shift unit | Idle: TXARM, TXRUN, TXTL, RXARM, RXHUNT, RXRUN, RXEP 0, both buffers and both shift registers empty, LA and LB 0, TXUR, RXOVR and RXSEEN 0. RXKIND, RXPH, RXG, RXOPEN and RXCAND 0, CRC and RXPE 0, and every other state of section 5.9.2 0 |
| Memory | Undefined |

So after reset every `uio` pin is undriven, and every `uo` pin is low. A UART TX line sits low until firmware sets it, which looks like a start bit, and an active-low chip select is asserted, so boards should allow for that. Nothing runs until the host starts it. Firmware should set its idle levels before it first blocks on the host. A line that must stay high or recessive through reset belongs on an `io` pin in `OD` mode with an outside pull-up.

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
2. Exactly which cycle an asynchronous pad change is first sampled, and any metastability in the synchroniser. A cycle-level simulator takes the value presented for that cycle.
3. Sub-cycle behaviour of a bypass path, and of its switch at a commit, and of the output switch: a commit that changes SEL[p], or a `pcfg` that changes OWN[p] or clears SEL[p], may glitch the pad, or the enable of a pin in `OD`, between clock edges. A cycle-level simulator may lag a bypass path by one cycle, and must say so.
4. Host SCK above clk/6, an SCK high or low shorter than 2 cycles, CS high for less than 3 cycles, CS moving while SCK is high, or CS low with SCK high as reset ends (section 9.1). Commands may be misread.
5. Everything before the first reset.
6. The cycle in which a STATUS byte, a host memory access, its BUSY or an RDREG read is served, within the limits in section 9. A byte-level simulator may do all of them in cycle *h*, and must say so.

**Reserved.** Defined today, but firmware must not rely on it, because it may change.

1. `ld`/`st` addresses of DEPTH or more, which wrap today.
2. Every illegal word. Illegal today, and may gain a meaning.
3. `edge -, v, n` with `v` other than 0.
4. Host command codes not listed, and STATUS byte 3 bits 7..1.
5. TXMODE bit 15, RKIND 3, CMODE 3, SMODE 3, SN 0, SN 1 with SMODE 2, SYNCLEN 17 to 31, and RPER 256 and up for the Manchester decoder. Today reserved bits are not stored, RKIND, CMODE and SMODE 3 act as 0, SN 0 acts as 1 with SMODE 1, SN 0 and 1 act as 2 with SMODE 2, SYNCLEN above 16 acts as 16, and the Manchester decoder takes RPER above 255 as 255.
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
8. Writing a slot of a half that is armed, running, hunting or in its tail. The half uses the new value from the next cycle, mid-word. RKIND is the exception: a reception keeps the decoder it started with, and a new RKIND acts from the next reception.
9. `smap` on a pin that no `pcfg` has claimed, and `pcfg` on a pin that is on a shift output. The second takes the pin off its output at once.
10. A third `txw` before `sgo tx` and its commit. It blocks until a commit starts the send, and with no event pending the thread never reaches one.
11. An outside clock pin, TCLK or RCLK, that is driven by one of the same unit's shift outputs. It is defined, and it loops.
12. A receive loop slower than one word per 16 bit times. Full words are lost and RXOVR sets. The last word of a reception still arrives.
13. `sgo rx` with HUNT on a pin that is already at HLVL. The hunt waits for the pin to leave HLVL and come back.
14. RACQ on a Manchester reception that does not begin with alternating bits. The wide window can take a bit-boundary edge for a clock edge, which loses the first bits or ends the reception.
15. RGAIN 2 or 3 against a sender whose clock is far from ours. Those gains follow only a small offset, so the bits slip and the reception ends early or decodes wrong.
16. `sgo rx` while the receive half is receiving, hunting or holding its last word. It stays armed (status bit 15) and starts a reception at the first commit after the half is idle, which may be a commit meant for something else. With HUNT that is only after HUNT is cleared.
17. Writing CODING, CSKIP, POLYHI, POLYLO, CHKHI or CHKLO while the half that uses the CRC register or the stuffer is running. As item 8, the half uses the new value from the next cycle, mid-frame. In particular CTX moved during a send or a reception takes the register from that half at once, in the middle of an append too, whose remaining bits are then CXOR. The half that gets the register goes on from the value the other half left, so its CRC is meaningless until its next start.
18. A POLY with a bit set below 32 - *w*. The bits below the width then don't stay 0, they move up into the CRC, and the CRC is not the standard's.
19. A word of kind `fcs` that is the last word of a send whose CRC is off, or a kind on a word while the stuffer is off. The kind has no effect.
20. REOP with a decoder other than timed samples. It has no effect.
21. HDLC frames that follow one another between flags. The closing flag ends the first reception and the half hunts again, so the next frame is received only if the flags between them complete the hunt's cycle (three flags with a falling-edge hunt and SYNCLEN 8), or after a gap of ones. Frames that share one flag never both arrive (section 5.9.7).
22. Writing RXCNT with a count that RXTOT has already passed. `rx_bit` ends a reception only when RXTOT equals RCNT, so that reception ends some other way, or when RXTOT wraps at 2048.

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

Every program below ran on the simulators. Programs with pins ran against device models that see only pad levels. The device models and decoders follow the protocol specs but were written alongside the firmware, so a shared misreading wouldn't show in them alone. `programs/sigrok/` also decodes the buses of 38 programs with sigrok's protocol decoders, which other people wrote from the protocol documents, on the simulator and on the RTL. It found two firmware bugs, in `i2c` and `cec`, fixed in the rows below. None has run on hardware yet.

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
- For the Manchester decoder, RXPER is cycles per bit, not per half bit, with RFRAC for the fraction. It takes RXPER above 255 as 255, so slower Manchester codes stay in firmware.
- Clear RACQ unless the reception begins with alternating bits, as 802.3's preamble does.
- A Manchester reception ends at the first bit with no clock edge. REND is only a backstop.
- RGAIN 2 and 3 filter hard and follow only a small clock offset. Use them where both clocks come from crystals, and RGAIN 0 or 1 where the sender's clock may be a percent or more off.
- A bit error in a preamble can complete the sync pattern early. That reception gives a runt of a few bits, and the next one, still in the preamble, gives the frame. Drop receptions shorter than the protocol's minimum.
- A new RKIND acts from the next reception, not the one in progress.
- A thread stop doesn't stop a send. Use ABORT to cut one.

**CRC and stuffing**

- The CRC register serves one half at a time. Set CTX to 1 before a send that appends, and back to 0 before the next reception begins: a reception that begins with CTX 1 starts its CRC from the register's old value, not its initial value, so its check is meaningless.
- One configuration checks one CRC: CW, CINIT and POLY at a reception's start and bits, CHK at its end. A USB device writes CODING, POLYHI and CHKHI for CRC-16 after a SETUP or OUT token and before the data packet's SYNC, and back for CRC-5 after the data packet.
- On a shared line, such as USB's pair, CAN or a loopback, the receive half receives its own unit's send, because a `uio` pin reads its own drive. Take or drop those words, or `sstop rx` before the send and arm the half again after it. Otherwise the self-reception's last word waits in RXSR, the half doesn't hunt, and the reply is lost. For USB keep REOP set in the send value of CODING (0xE6EF), or the self-reception doesn't end at the EOP and runs into the reply.
- USB lets the last bit before an EOP stretch by up to six bit times (dribble). The check covers every bit up to the SE0, so such a packet fails it although it is good, and its bits after the PID are then not a whole number of bytes. Check those packets again in firmware, with another unit's idle send half as the CRC engine, or accept them on the bit count.
- A word with no kind is not stuffed and breaks the run. Give every word the stuffer must see its kind, USB's SYNC and PID included (`stuff`).
- The receive CRC covers every bit from CSKIP after the sync pattern to the end, the received CRC included, so a good reception leaves the residue, not 0. With HDLC it also covers the closing flag's first six bits, so CHK is 0xD0F9, not 0x1D0F.
- An HDLC reception's last word ends with the closing flag's first six bits, 011111. Drop them. The closing flag ends the reception and the half hunts again, so it receives one frame per hunt: with a falling-edge hunt and SYNCLEN 8, a run of flags gives one six-bit reception for every three flags, and a frame arrives only if its opening flag falls on that cycle, as after 3, 6 or 9 flags or after a gap of ones. Drop the six-bit receptions. A sender that puts 1, 2, 4 or 5 flags between frames, or idles on flags, loses frames.
- The end flags belong to the last word. Read them with `srd Rd, end` after the `rxw` that sets T and before the next `rxw`.
- `srd` reads the register as it is: the remainder in wire order, left-aligned, neither reflected nor complemented.
- An idle send half used as a CRC engine needs words of kind `crc`: after a word of kind `fcs` the register holds its initial value again.
- POLY is the reciprocal of the polynomial a standard names for a Fibonacci PRBS or scrambler, and the LFSR needs CINIT 1 (section 5.9.7).
- The end test needs RCLK set to the second line, and works only with timed samples.
- A stuff violation on USB or CAN ends a reception early, and the line goes on: with HUNT, the half hunts again in the middle of the frame. Drop receptions with end flag 1 set, and on CAN expect the error frame next.
- On CAN, with HUNT, the half hunts again as soon as RCNT ends a frame, so a stuff bit after the CRC, or another node's ACK, can start a short reception that ends on a stuff violation in the recessive tail. Drop it: it has end flag 1 set.
- On CAN, write RXCNT before RXTOT reaches the new count. `rx_bit` ends a reception only when RXTOT equals RCNT, so a late write never ends it: it runs into the recessive tail, ends on a stuff violation and fails its check. With MSB-first words the DLC is complete only with the second word, at bit 32, 2 bit times before the end of a frame with no data.
- With RESYNC every edge of RPIN pulls the sample countdown, rising and falling alike. CAN resynchronizes only on recessive-to-dominant edges, so with a transceiver whose rising and falling delays differ, the sample point moves by that difference after each dominant-to-recessive edge.
- A data packet with no data, such as a USB zero-length packet, can't use `fcs`, which covers its own word: send the CRC of no data as a word of kind `stuff`, 0x0000 for USB's CRC-16.
