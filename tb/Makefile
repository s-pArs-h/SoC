# System test: firmware on the RTL, driven over the UART pins (cocotb + Icarus).
#   make              (build the firmware first: make -C ../sw)
#   make SEED=7
#
# The simulated clock is 100 MHz but the SoC is told CLK_HZ = 1 MHz and
# BAUD = 125000, i.e. 8 clock cycles per UART bit, so that serial traffic
# does not dominate the simulation time. The point memory is 256 points.

SIM          ?= icarus
TOPLEVEL_LANG = verilog
SEED         ?= 1
SIM_CLK_HZ   ?= 1000000
SIM_BAUD     ?= 125000
PTS_AW       ?= 8

ROOT     = $(abspath ..)
FIRMWARE = $(ROOT)/build/firmware.hex

VERILOG_SOURCES = $(wildcard $(ROOT)/ip/riscv/*.sv) $(wildcard $(ROOT)/ip/kmeans/*.sv) \
                  $(wildcard $(ROOT)/rtl/*.sv)
COCOTB_TOPLEVEL     = soc
COCOTB_TEST_MODULES = test_soc
SIM_BUILD           = sim_build
COMPILE_ARGS += -I$(ROOT)/ip/riscv \
                -Psoc.INIT_FILE=\"$(FIRMWARE)\" \
                -Psoc.CLK_HZ=$(SIM_CLK_HZ) -Psoc.BAUD=$(SIM_BAUD) -Psoc.PTS_AW=$(PTS_AW)

export SEED SIM_CLK_HZ SIM_BAUD PTS_AW
export COVERAGE_REPORT = $(abspath coverage.txt)
export SPEEDUP_REPORT  = $(abspath speedup.txt)

include $(shell cocotb-config --makefiles)/Makefile.sim
