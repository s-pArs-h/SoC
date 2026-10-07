# K-means SoC: top-level flows
#
#   make firmware    RISC-V firmware image (build/firmware.hex)
#   make lint        Verilator -Wall: SoC (all define combinations) and board top
#   make sim         system test: firmware on the RTL, driven over the UART (cocotb)
#   make host-test   host program and protocol tests (pytest, no hardware)
#   make formal      SymbiYosys: FIFO and accelerator proofs
#   make synth       Yosys resource estimate per block (Artix-7)
#   make all         everything above
#   make bitstream   Vivado build (needs Vivado): fpga/build.tcl
#
# Tools: iverilog, verilator, yosys, sby + yices, riscv64-unknown-elf-gcc,
#        python3 with cocotb >= 2.0 and pytest

RTL = $(wildcard ip/riscv/*.sv) $(wildcard ip/kmeans/*.sv) $(wildcard rtl/*.sv)
LINT = verilator --lint-only -Wall -Iip/riscv

.PHONY: all firmware lint sim host-test formal synth bitstream clean

all: lint host-test sim formal synth

firmware:
	$(MAKE) -C sw

lint:
	$(LINT) --top-module soc $(RTL)
	$(LINT) -DFORMAL --top-module soc $(RTL)
	$(LINT) -DFORMAL -DSYNC_FIFO_DATA_CHECK -DKMEANS_ACCEL_BMC --top-module soc $(RTL)
	$(LINT) -DSIM --top-module top_nexys_a7 $(RTL) fpga/top_nexys_a7.sv
	@echo "Verilator -Wall: clean"

sim: firmware
	$(MAKE) -C tb

host-test:
	cd host && python3 -m pytest -q

formal:
	cd formal && sby -f fifo.sby
	cd formal && sby -f accel.sby

synth: firmware
	python3 synth/estimate.py

bitstream: firmware
	vivado -mode batch -source fpga/build.tcl

clean:
	rm -rf build tb/sim_build tb/results.xml tb/__pycache__ tb/coverage.txt tb/speedup.txt
	rm -rf host/__pycache__ host/.pytest_cache formal/fifo_*/ formal/accel_*/
