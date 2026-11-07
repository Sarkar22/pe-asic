# SPDX-License-Identifier: Apache-2.0
# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# cocotb 1.x silently runs 0 tests here, and newer versions break gl_test's results check.
ifneq ($(shell cocotb-config --version),2.0.1)
$(error cocotb 2.0.1 required: activate the project venv)
endif

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
CORE_SOURCES = pe_pkg.sv pe_decode.sv pe_core.sv pe_thread.sv pe_exec.sv pe_regfile.sv \
               pe_mailbox.sv pe_pins.sv pe_mem.sv pe_host.sv pe_spi.sv
PROJECT_SOURCES = project.sv $(CORE_SOURCES)

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
VERILOG_SOURCES += $(PWD)/../verif/sram_model.v
VERILOG_SOURCES += $(PWD)/../verif/cells_rtl.v

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
# Cell models for the PDK in $(PDK) (ihp-sg13cmos5l or ihp-sg13g2), e.g. sg13cmos5l_stdcell.v
PDK ?= ihp-sg13cmos5l
PDK_CELL_PREFIX = $(patsubst ihp-%,%,$(PDK))
PDK_LIBS_REF = $(PDK_ROOT)/$(PDK)/libs.ref
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_io/verilog/$(PDK_CELL_PREFIX)_io.v
# Newer PDK versions ship the UDP (user-defined primitive) models in a separate file
VERILOG_SOURCES += $(wildcard $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_stdcell/verilog/$(PDK_CELL_PREFIX)_udp.v)
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_stdcell/verilog/$(PDK_CELL_PREFIX)_stdcell.v
# The SRAM macro's model
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_sram/verilog/RM_IHPSG13_1P_512x16_c2_bm_bist.v
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_sram/verilog/RM_IHPSG13_1P_core_behavioral_bm_bist.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

# The RTL core runs beside the netlist in test_netlist_matches_rtl:
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(CORE_SOURCES))

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
