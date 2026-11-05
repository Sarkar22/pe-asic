# SPDX-License-Identifier: Apache-2.0
# Builds the Verilator models behind rtl_chip.py.
#   stub: pe_spi is replaced, so host commands land cycle by cycle from Python.
#   pins: the real pe_spi, driven through the pads.
# Each comes at depths 128 and 512. Output goes to $(BUILD)/<kind>_<depth>/libpe.so.

BUILD ?= build
JOBS  ?= 2
SRC   := ../src
RTL   := $(SRC)/pe_pkg.sv $(SRC)/pe_decode.sv $(SRC)/pe_core.sv $(SRC)/pe_thread.sv $(SRC)/pe_exec.sv \
         $(SRC)/pe_regfile.sv $(SRC)/pe_mailbox.sv $(SRC)/pe_pins.sv $(SRC)/pe_mem.sv $(SRC)/pe_host.sv
FLAGS := --cc --exe --build -j $(JOBS) -O3 --x-assign 0 --x-initial 0 -Wno-fatal -Wno-lint --top-module pe_core \
         -CFLAGS "-O2 -fPIC" -LDFLAGS "-shared"

TARGETS := $(foreach k,stub pins,$(foreach d,128 512,$(BUILD)/$(k)_$(d)/libpe.so))

all: $(TARGETS)

$(BUILD)/stub_%/libpe.so: $(RTL) pe_spi_stub.sv sram_model.v public.vlt shim.cpp
	mkdir -p $(dir $@)
	verilator $(FLAGS) +define+PE_DEPTH=$* --Mdir $(dir $@)obj -o ../libpe.so \
	  $(RTL) pe_spi_stub.sv sram_model.v public.vlt $(abspath shim.cpp)

$(BUILD)/pins_%/libpe.so: $(RTL) $(SRC)/pe_spi.sv sram_model.v public.vlt shim.cpp
	mkdir -p $(dir $@)
	verilator $(FLAGS) +define+PE_DEPTH=$* -CFLAGS -DPE_PINS --Mdir $(dir $@)obj -o ../libpe.so \
	  $(RTL) $(SRC)/pe_spi.sv sram_model.v public.vlt $(abspath shim.cpp)

clean:
	rm -rf $(BUILD)

.PHONY: all clean
