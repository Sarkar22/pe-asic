# AI use

This log records how AI tools are used in this project, as the competition asks.

| Date | Tool | Scope | Human check |
|---|---|---|---|
| 2026-10-30 | AI coding assistant | Drafted the UART TX smoke-test RTL, cocotb test, template config and this repo's setup | Tests run on Icarus; a seeded stop-bit bug was caught by the test |
| 2026-10-30 | AI coding assistant | Ported the design to SystemVerilog, tested which language features survive each tool in the flow, and added the netlist-against-RTL test | Hardened netlist compared with the original Verilog on every pin (3 runs of 2M cycles, 0 mismatches); two synthesis problems reproduced in small separate tests; seeded bugs caught by the tests |
