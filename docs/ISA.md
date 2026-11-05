# ISA v0.1

Draft, 2026-11-05. This instruction set can change until the hardware freezes.

**Status of the evidence.** Where this document says something ran, it ran on Python simulators: two hand-written, and one compiled directly from the instruction table and code blocks of this file. They agree on every cycle compared. Sections 7 and 11.2 list what ran. Treat anything they don't list as not run. The cycle figures in section 9 come from an RTL design, which has only been simulated and synthesized. No real hardware is behind this document. One of the simulators is in `sim/`. The test benches and the RTL aren't published yet.

## 1. Overview

A thread changes the outside world only at an **event**. It prepares pin values in a shadow copy, then names the cycle, or the input pattern, at which they should take effect. In that one cycle the hardware does three things at once:

- commits all of that thread's prepared pin changes;
- snapshots all inputs;
- records the cycle number.

Between events a thread is ordinary software working on registers and a one-bit wire called T. A protocol then reads like a small synchronous circuit: an event is its clock edge, and the code between events is its next-state logic.

A chip has four threads in fixed rotation, one memory for code and data, 16 output pins, 16 input pins, a 16-bit free-running timebase and an SPI link to a host.

## 2. Programmer's model

**Threads.** Four hardware threads share one datapath in fixed rotation. Thread *i* owns every cycle where `cycle mod 4 = i`, so each thread runs one instruction every 4 cycles, 12.5 million per second at 50 MHz. A slot is never lent to another thread. A build with fewer threads keeps the four-slot rotation and leaves the missing threads' slots idle, so firmware timing doesn't change.

There is no per-thread clock divider and no issue-rate prescaler. TB and D only schedule events.

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

**Global state:**

| State | Bits | Purpose |
|---|---|---|
| TB | 16 | Free-running timebase, one count per clock, never stopped or divided |
| M0..M7 | 16 each | Mailbox words, reachable by every thread and by the host |
| FULL0..FULL7 | 1 each | Mailbox full flags |
| SYNC1, SYNC2 | 16 each | Two-flop input synchroniser |
| OUT, SHD | 16 each | Pin output flops and their shadow. Bit *p* belongs to output pin *p* (section 5.1) |
| OE, SHOE, OD | 8 each | `uio` output enable, its shadow, and open-drain mode |
| BYP, SHBYP | 5 each | Bypass switch per lane and its shadow |
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
3. **Execute.** The thread owning cycle *c* runs `slot(P_start)` (section 3.6), where P_start is its P flag from before step 2.
4. **Memory.** The thread whose memory cycle is *c* runs `memory()` (section 4.2).
5. **Update.** Every thread runs `update_stale()` (section 3.5). Then `SYNC2 := SYNC1`, `SYNC1 := {uio_in, ui_in}`, and *c* advances.

A host command takes effect after step 5 of its cycle. A memory access and the read of RDREG come later (section 9.1).

Apart from that one read, no step reads a flop that an earlier step wrote in the same cycle, so the steps behave like simultaneous flop updates. World instructions and blocked waits stall on P_start, which is what keeps this true. `stop()` is the exception because a stop in the event's own cycle must see the wait as finished. In hardware, the P it reads is the P flop at the start of the cycle and not firing in this cycle. The one overlap is a write conflict, and the instruction wins: `pcfg p, OD` on a pin that its old owner commits in that cycle.

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

**World instructions.** An instruction with W = `y` in the table touches the outside world. While an event is pending it stalls, using up its slot, and the same instruction runs again next slot. That covers pin writes and reads, `rds`, `rdd`, `wrd`, `tlate`, `halt`, every event instruction and every mailbox operation. A shift with W = `pin` stalls only if it names a pin. ALU work, branches and memory access keep running. So program order is the order the world sees: a mailbox message can never overtake a pin edge.

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
uo_out[3]  = host data out (section 9)
uo_out[i]  = BYP[i] ? ui_in[SRC(i)] : OUT[i]       for i != 3
uio_out[i] = OD[i] ? 0        : OUT[8 + i]
uio_oe[i]  = OD[i] ? !OUT[8 + i] : OE[i]
```

Bypass lanes exist only for `uo0` and `uo4..uo7`, with `SRC(0) = 3` and `SRC(i) = i` for the others. `uo1` and `uo2` always show OUT.

Pads have no pulls and no hysteresis. `uo` pads can't tri-state. Only `uio` pads can do open drain, turnaround or read-back. A `uio` input reads the pad even while the chip drives it, so a thread can read another thread's output with no outside wire, which gives a loopback self-test.

### 5.3 Claiming pins

Every `uio` and `uo` pin has one owner. `pcfg` claims a pin for the thread running it and sets how the pin behaves:

```
claim(p):
    OWN[p] := TID

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

`LANES` is 0, 4, 5, 6 and 7.

| Mode | Applies to | Behaviour |
|---|---|---|
| `IN` | `uio` | Not driven, from the next commit. Direction can flip later with `oe`. A pin leaving `OD` releases at once |
| `PP` | `uo`, `uio` | Push-pull. On `uio`, drive starts at the next commit. On a lane, the switch leaves bypass at the next commit. A pin leaving `OD` releases at once and floats until that commit |
| `OD` | `uio` | Open drain. A prepared 1 releases the pin, a 0 pulls it low. Takes effect at once, released, and clears OE and SHOE |
| `BYP` | `uo0`, `uo4..uo7` | Output follows its lane source combinationally, from the next commit |

At reset thread 0 owns every pin. A thread's writes to pins it doesn't own are dropped, and its commit moves only pins it owns. `pcfg` takes a pin with no check, even from a running thread, so two protocols are kept off each other's lines only if firmware claims each pin once.

A hand-over the threads agree on is safe for a pin in `PP`. SHD is global, so the pin keeps its level when the owner changes, and `pcfg p, PP` on a pin already in `PP` moves nothing. The new owner's next commit moves the pin to whatever SHD holds, including a value the old owner prepared and never committed, so the old owner should commit first. `pcfg p, OD` is different: it releases the pin at once, even one held low.

**Open-drain safety is structural.** In `OD` mode the output data to the pad is forced to 0 and only the enable moves, so the pin can't drive high whatever firmware does. Entering `OD` always starts released, so `pcfg` doesn't pull a line low during init. OE and SHOE stay 0 for as long as a pin is in `OD`, so leaving `OD` can't drive a stale value either.

### 5.4 Shadow and commit

Nothing reaches a pad until an event commits it, and all of a thread's changes commit on the same cycle. So data and clock can change together, and a direction turnaround lands exactly where it should. The two exceptions are `pcfg` into or out of `OD`, which acts at once, and host ABORT (section 9.3).

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
            if p >= 8:
                OE[p-8] := SHOE[p-8]
            elif p in LANES:
                BYP[p] := SHBYP[p]
```

OD and OWN have no shadow. `oe` and `oe8` do nothing on a pin in `OD` mode.

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
| `edge p, v, i` | `1110 pppp vv 0 iiiii` | `p:opin- v:eact i:u5` | 1 | y | `shadow(p, [0, 1, T, 1 - T][v]); arm(i)` |
| `edge p, v, Rr` | `1110 pppp vv 1 00 rrr` | `p:opin- v:eact r:reg` | 1 | y | `shadow(p, [0, 1, T, 1 - T][v]); arm(Rr)` |
| `shl8 Rr, i, o` | `1111 0 0 rr iiii oooo` | `r:reg4 i:ipin- o:opin-` | 1 | pin | `if i != 0: T := bit(S, i); f := bit(Rr, 7); Rr := (Rr & 0xFF00) \| ((Rr << 1) & 0xFE) \| T; T := f; shadow(o, T)` |
| `shl16 Rr, i, o` | `1111 0 1 rr iiii oooo` | `r:reg4 i:ipin- o:opin-` | 1 | pin | `if i != 0: T := bit(S, i); f := bit(Rr, 15); Rr := (Rr << 1) \| T; T := f; shadow(o, T)` |
| `shr8 Rr, i, o` | `1111 1 0 rr iiii oooo` | `r:reg4 i:ipin- o:opin-` | 1 | pin | `if i != 0: T := bit(S, i); f := bit(Rr, 0); Rr := (Rr & 0xFF00) \| ((Rr & 0xFF) >> 1) \| (T << 7); T := f; shadow(o, T)` |
| `shr16 Rr, i, o` | `1111 1 1 rr iiii oooo` | `r:reg4 i:ipin- o:opin-` | 1 | pin | `if i != 0: T := bit(S, i); f := bit(Rr, 0); Rr := (Rr >> 1) \| (T << 15); T := f; shadow(o, T)` |
<!-- isa-table-end -->

81 rows. Of the 65,536 words, 37,005 are legal and 28,531 (43.5%) are illegal. Group `1101` is entirely free for future assists.

### 6.4 Pseudo-instructions

| Written | Assembles to | Words |
|---|---|---|
| `li Rd, imm16` | `ldi Rd, low byte`, then `lui Rd, high byte` if the high byte isn't 0 | 1 or 2 |
| `ret` | 1 |
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

A name can be defined only once. It can't be a mnemonic, a register, a pin name, or one of `T`, `inbox`, `eq`, `ne`, `chg`, `uo`, `ui`, `io`, `IN`, `PP`, `OD` and `BYP`.

| Directive | Effect |
|---|---|
| `.equ NAME expr` | Defines a constant from numbers and earlier constants. A pin name or pin alias makes NAME a pin alias |
| `.org n` | Sets the address of what follows. Placing a second word at an address is an error |
| `.data expr` | Places one word, 0 to 65535 |
| `.if expr`, `.else`, `.endif` | Conditional assembly on numbers and earlier constants. These can nest |

The assembler warns when a line writes a pin that no earlier line names in a `pcfg`, because on every thread but 0 that write is dropped (section 11.5).

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

These follow from one instruction per 4 cycles, with no hardware shifting. Three rates are out, by arithmetic and not by a run: full-speed USB is 4.2 cycles per bit, about one instruction. 10BASE-T is 2.5 cycles per half bit, faster than any pin can change. CAN FD's data phase at 2 Mbit/s or more leaves 6 slots per bit against a 14-slot decode loop. Classical CAN at 1 Mbit/s is too slow as written, at 12 slots per bit against 14 to decode and 23 to transmit, but splitting a receive-only node across more threads wasn't tried.

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
| SYNC1, SYNC2 | 0 |
| OUT, SHD | 0 |
| OE, SHOE, OD, BYP, SHBYP | 0 |
| OWN | Every pin owned by thread 0 |
| BUSY | 0 |
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

**RDREG `reg8`:** 0..7 are R0..R7, 8 is D, 9 is S, 10 is PC, 11 is flags with T in bit 0, LATE in bit 1, STALE in bit 2 and WAITING in bit 3. Other values read 0. RDREG returns the state when it is served, not at *h*: the flops at the start of a cycle from *h + 2* to *h + 5*, by thread and by *h* mod 4. A stopped thread can still move: STALE can set, or a stepped event can fire.

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

abort_pins():
    for i in range(8):
        OE[i] := 0
        SHOE[i] := 0
        if OD[i]:
            OUT[8+i] := 1
            SHD[8+i] := 1
```

So every `uio` pad is undriven, and `uo` pins and bypass switches hold. Use ABORT, not a plain stop, when a target must see a line released. ABORT clears P and WAITING itself, so a thread it stops in a wait reads the same as one whose wait ended, which is deliberate: ABORT abandons the thread, and a host that needs the difference uses a plain stop.

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
3. Sub-cycle behaviour of a bypass path, and of its switch at a commit. A cycle-level simulator may lag it by one cycle, and must say so.
4. Host SCK above clk/6, an SCK high or low shorter than 2 cycles, CS high for less than 3 cycles, CS moving while SCK is high, or CS low with SCK high as reset ends (section 9.1). Commands may be misread.
5. Everything before the first reset.
6. The cycle in which a STATUS byte, a host memory access, its BUSY or an RDREG read is served, within the limits in section 9. A byte-level simulator may do all of them in cycle *h*, and must say so.

**Reserved.** Defined today, but firmware must not rely on it, because it may change.

1. `ld`/`st` addresses of DEPTH or more, which wrap today.
2. Every illegal word. Illegal today, and may gain a meaning.
3. `edge -, v, n` with `v` other than 0.
4. Host command codes not listed, and STATUS byte 3 bits 7..1.

**Illegal words.** A word is illegal if it matches no row in section 6.3, or has a value in a field that its type forbids. That includes any non-zero bit where a row has a fixed 0, group `1101`, `put` on `uo3`, `oe` on a `uo` pin, `pcfg` with `IN` or `OD` on a `uo` pin, `pcfg` with `BYP` on anything but a lane, and `untilm` mode 3.

Executing an illegal word sets ILL and runs `stop()`, whatever else is pending. PC stays on the word, so the host can read it. The check comes before the stall check, so a pending event is cancelled, not left to fire. No other state changes. The assembler can't produce an illegal word, but `.data` can place one.

**Defined, but probably a bug.** The hardware doesn't check these.

1. Two threads claiming one pin with no agreed hand-over. The last `pcfg` wins, and the old owner's writes are dropped.
2. A deadline 32768 or more cycles after the arming cycle, which is late by the rule in section 3.5. The test is on that distance, not on the step: `edge -, 0, R1` with R1 = 32768 straight after `now` lands 32765 cycles ahead, on time.
3. `tick 0` with D in the past, which is late. Use `now`. After `wrd` of a future cycle, `tick 0` or `edge p, v, 0` is on time and is the way to land on exactly that cycle, which section 7.3 relies on.
4. `untilm` with an empty mask. `eq` matches at once, and `ne` never matches.
5. Two parties filling one mailbox word without a convention. Each operation is atomic, but a refused `trysend` is silent unless T is tested.
6. Storing into code that a thread is running. It is defined by memory-cycle order, and a thread can even rewrite its own next instruction.
7. Running a `tick` from a D that a host STEP left behind. It is defined, and usually late.

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

Every program below ran on the simulators. Programs with pins ran against device models that see only pad levels. The device models and decoders follow the protocol specs but were written alongside the firmware, so a shared misreading wouldn't show. No sigrok decoder was used. None has been run outside the simulators.

A program name can appear more than once, as different code.

| Program | Words | Result |
|---|---|---|
| `uart_tx` | 15 | The example in section 11.3. 115200 baud and 1 Mbaud on 16 of 16 threads and alignments, 9600 baud, and 12, 434, 5208 and 32767 cycles per bit |
| `uart_tx` | 14 | Trimmed: the stop bit is a ninth shifted bit. 13 and 434 cycles per bit in the bench, 12 cycles per bit on 16 of 16 in the limit sweep |
| `uart_rx` | 22 | Mid-bit sampling exact at 8 phases. Glitch rejected, framing error flagged in bit 8. 31 of 31 break lengths clean. No false bytes with a slow host |
| `uart_rx` | 23 | Reports a framing error as a separate word, which keeps frames abutting. 115200 baud and 1 Mbaud at 8 phases. Sender error -5% to +4.5% at 115200 baud, -5% to +2.5% at 1 Mbaud |
| `uart_rx` | 20 | Trimmed: the host posts the half bit too. 50 and 434 cycles per bit, and the same mailbox traffic in the shared image as solo |
| `spi` | 22 | All 4 modes, half periods 9, 10 and 25, 16 alignments each |
| `spi` | 20 | 21 with CPOL = 1. All 4 modes x 4 threads x 4 alignments at half periods 8, 9, 25 and 250. Multi-byte frames. MOSI and MISO decoded at the pads |
| `spit_c` | 17 | SPI target, mode 0, half period 10 on 12 start phases |
| `i2c` | 76 | Controller against an EEPROM model. At 100 kHz: write, random read, wrong address, data NACK, stretching on every clock, SDA stuck, SCL stuck at STOP. At 400 kHz: write and random read |
| `i2c_c` | 76 | Same source as `i2c`. Same cases, through a second bench |
| `i2c` | 72 | Trimmed, same host interface. The same cases as `i2c` at 100 and 400 kHz, and write and random read in the shared image |
| `i2c_write` | 42 | 100 and 400 kHz: target ACK, stretching on every clock and after each ACK, NACK on data, wrong address, 3 transactions, 16 of 16 threads and alignments. Every UM10204 interval in range, including tBUF, with rise at the spec maximum. No SCL timeout |
| `i2c_write` | 42 | Write-only controller. 100 and 400 kHz on all four threads, stretching, NACK on address and data, stretch timeout reported |
| `i2c_target` | 62 | 24C02 style, plus 256 words of storage, 512-word memory only. Against `i2c` on another thread of the same chip at 100 and 400 kHz: write, random read, wrong address |
| `i2c_tgt` | 78 | Target with 32 bytes of storage, on 128 words. ACK depends on the address and lands 3 cycles pad to pad at 100 and 400 kHz, on all threads and alignments. Repeated START, pointer wrap, wrong addresses silent |
| `i2c_fp` | 87 | 1 MHz build of the target. Bytes correct. Reaction 3 cycles at minimum tLOW, 3 to 5 at minimum tHIGH. Intervals not checked at this rate |
| `ws2812` | 19 | T0H 360 ns, T1H 700 ns, 1240 ns per bit, no jitter, 72 bits on 16 of 16 threads and alignments, latch gap checked |
| `ws2812` | 17 | Same bit timing, on all four threads |
| `onewire` | 34 | Reset 490 us, presence, 5 bytes written, no-device case reported, 12 of 12 device timing corners, 16 of 16 threads and alignments. No read slots or search |
| `onewire` | 28 | Reset 480.0 us, presence, write byte, no-device case. No read slots or search |
| `jtag` | 37 | Host-driven scan engine on a 2-TAP chain: both IDCODEs, IR scan, 40 bits through BYPASS, Pause-DR mid-scan, with a pad decoder walking the TAP states. Half period 9 in the bench, 8 on 16 of 16 in the limit sweep |
| `jtag` | 51 | Fixed sequence against a TAP model: IDCODE, IR load, BYPASS loop, half periods 25 and 16 on all four threads |
| `swd` | 128 | Host with turnaround: line reset, JTAG-to-SWD, DPIDR, writes, posted AP reads, WAIT retry and limit, FAULT, no response, read and write parity. Contention checked every cycle. No words spare |
| `swd_c` | 64 | Read of DPIDR, ACK and parity errors reported, no contention |
| `mdio_sta` | 47 | Clause 22 station, read and write, 1.25 MHz, PHY output delay 0 to 300 ns, 16 of 16 threads and alignments |
| `mdio_phy` | 60 | Plus 32 registers. Follows an external MDC, and MDIO moves exactly 3 cycles after MDC rises |
| `manch_rx` | 28 | Clock recovery: preamble lock, start mark, end of frame, 4,800-bit frame, 4 threads x 8 phases. Sender clock error -20% to +24%, edge jitter 10% of a bit |
| `manch_rx` | 13 | Fixed format. Sender clock +/-10% |
| `dmx_tx` | 29 | DMX512 transmit. Full 512-slot universe decoded at the pad, break 110 us, MAB 16.2 us, clean through a 1.4 ms host stall |
| `dmx_rx` | 52 | Slot windows 1, 250 and 509, shortest legal break and MAB, sender error 2%, 1 ms mark between slots, non-zero start code ignored. The universe streams through the mailbox |
| `ps2_dev` | 114 | PS/2 device, both directions with parity and device ACK. Commands FF, F2, ED, EE and FE, bad parity answered, host inhibit and request-to-send handled, byte resent |
| `usb_tx` | 43 | Low-speed transmit, NRZI and stuffing at 33 cycles per bit, every D+ edge on the grid |
| `usb_rx` | 69 | Low-speed receive in 2 threads: stuffing-heavy packets, sender error 1.5%, D+/D- skew to 160 ns, 2-bit gaps, jitter to 60 ns. 80 ns loses 4 runs of 12, and 100 ns fails |
| `usb_dev` | 194 | Device link in 3 threads, 512-word memory: SETUP with CRC-16 check, ACK, NAK, IN with DATA1, host ACK, address filter, bad CRC rejected. Worst turnaround 6.06 bit times against 6.5. No enumeration, data toggle or token CRC-5 check |
| `can` | 166 | 3 threads, 512-word memory. 500 kbit/s, stuffing and CRC-15 in firmware, sender error 2%, 440 ns transceiver loop, ACK in the right slot, both arbitration outcomes. RX alone 92 words and TX alone 74 ran on 128. No error frames or retransmit |
| `cec` | 50 | HDMI CEC initiator. All bit windows met, ACK and no-ACK reported. No arbitration |
| `lin_c` | 52 | LIN responder header with auto-baud at +/-14%. Header only |
| `nec_c` | 41 | NEC IR transmit. Carrier exact, 1316 cycles with 439 high |
| `par_c` | 7 | 8-bit parallel capture. Data hold 0 cycles after the strobe, strobe period 28 |
| `hdlc_tx` | 48 | NRZI, stuffing and CRC-16/X-25 in firmware, decoded from the pad trace |
| `crc16` | 14 | CRC-16/CCITT, published check value |
| `crc8` | 12 | CRC-8/MAXIM, published check value |
| `crc32` | 26 | CRC-32, published check value |
| `reflex` | 15 | The reaction test behind section 11.4. Pre-armed 3 cycles, firmware 9 to 12 |
| `memstress` | 17 | Memory load for the timing-isolation check: a thread hammering `ld` and `st` changes nothing on another thread |
| `memhog` | 8 | The same job in the concurrency runs |

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
