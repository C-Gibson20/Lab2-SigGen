#!/bin/sh

~/Documents/iac/lab0-devtools/tools/attach_usb.sh

rm -f obj_dir
rm -f sigdelay.vcd

verilator -Wall --cc --trace sigdelay.sv counter.sv dual_ram.sv --exe sigdelay_tb.cpp

make -j -C obj_dir/ -f Vsigdelay.mk Vsigdelay

obj_dir/Vsigdelay