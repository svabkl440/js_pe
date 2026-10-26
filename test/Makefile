
# Makefile
SIM ?= icarus
FST ?= -fst
TOPLEVEL_LANG ?= verilog

SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = project.v

# Select protocol: make PROTO=uart or make PROTO=spi
PROTO ?= uart

ifeq ($(PROTO),uart)
    COCOTB_TEST_MODULES = test_uart
else ifeq ($(PROTO),spi)
    COCOTB_TEST_MODULES = test_spi_tx
else ifeq ($(PROTO),i2c)
	COCOTB_TEST_MODULES = test_i2c
else
    $(error Unknown PROTO '$(PROTO)'. Use PROTO=uart or PROTO=spi)
endif

ifneq ($(GATES),yes)
    SIM_BUILD = sim_build/rtl
    COMPILE_ARGS += -DFUNCTIONAL
    VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
    VERILOG_SOURCES += models/RM_IHPSG13_1P_core_behavioral_bm_bist.v
    VERILOG_SOURCES += models/RM_IHPSG13_1P_1024x8_c2_bm_bist.v
else
    SIM_BUILD = sim_build/gl
    COMPILE_ARGS += -DGL_TEST -DFUNCTIONAL -DSIM
    VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_io/verilog/sg13g2_io.v
    VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_stdcell/verilog/sg13g2_stdcell.v
    VERILOG_SOURCES += models/RM_IHPSG13_1P_core_behavioral_bm_bist.v
    VERILOG_SOURCES += models/RM_IHPSG13_1P_1024x8_c2_bm_bist.v
    VERILOG_SOURCES += $(PWD)/gate_level_netlist.v
endif

COMPILE_ARGS += -I$(SRC_DIR)
VERILOG_SOURCES += $(PWD)/tb.v

TOPLEVEL = tb

include $(shell cocotb-config --makefiles)/Makefile.sim


