# Load the bitstream into the board over USB (volatile: lost at power-off).
#   vivado -mode batch -source fpga/program.tcl   (from the repository root)
set root [file normalize [file join [file dirname [info script]] ..]]
open_hw_manager
connect_hw_server
open_hw_target
set dev [lindex [get_hw_devices xc7a100t*] 0]
current_hw_device $dev
set_property PROGRAM.FILE [file join $root build vivado kmeans_soc.bit] $dev
program_hw_devices $dev
close_hw_manager
