// Coverage calls for the models that verif/coverage.sh builds with Verilator's line and toggle coverage. Compiled
// beside shim.cpp, which owns the Verilator context; that context is the calling thread's.
//
// Each chip registers its coverage points in the context when it is made, and the context reads the counters only
// when it writes them, so a chip's counts must be written before the chip is freed. vlcov.py writes them, adds them
// up, and clears the context, whenever the last chip of a library goes.
//
// SPDX-License-Identifier: Apache-2.0

#include "verilated.h"
#include "verilated_cov.h"

extern "C" {

// Writes every coverage point of the chips alive now to path, in Verilator's coverage file format.
void pe_cov_write(const char *path) { Verilated::threadContextp()->coveragep()->write(path); }

// Forgets every coverage point. Call it only when no chip of this library is alive.
void pe_cov_clear(void) { Verilated::threadContextp()->coveragep()->clear(); }

// Sets every count to 0 and keeps the points, for chips that stay alive after their counts were written.
void pe_cov_zero(void) { Verilated::threadContextp()->coveragep()->zero(); }

}  // extern "C"
