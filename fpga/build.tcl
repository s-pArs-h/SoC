# Vivado non-project build of the K-means SoC for the Nexys A7-100T.
#   make -C sw                                  (firmware image first)
#   vivado -mode batch -source fpga/build.tcl   (from the repository root)
# Results in build/vivado/: utilization and timing reports, kmeans_soc.bit

set part xc7a100tcsg324-1
set root [file normalize [file join [file dirname [info script]] ..]]
set out  [file join $root build vivado]
file mkdir $out

# The RAM is initialised from firmware.hex ($readmemh, path relative to the
# working directory), so build from the output directory with a copy of it.
file copy -force [file join $root build firmware.hex] [file join $out firmware.hex]
cd $out

read_verilog -sv [glob $root/ip/riscv/*.sv $root/ip/kmeans/*.sv $root/rtl/*.sv \
                       $root/fpga/top_nexys_a7.sv]
read_xdc $root/fpga/nexys_a7.xdc

synth_design -top top_nexys_a7 -part $part -include_dirs $root/ip/riscv
opt_design
place_design
phys_opt_design
route_design

report_utilization -file utilization.rpt
report_utilization -hierarchical -file utilization_hier.rpt
report_timing_summary -max_paths 10 -file timing.rpt
write_bitstream -force kmeans_soc.bit

puts "Done: [file join $out kmeans_soc.bit]"
puts "Worst setup slack: [get_property SLACK [get_timing_paths -max_paths 1 -setup]] ns"
