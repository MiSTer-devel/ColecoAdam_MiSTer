# Testbench for rtl/sdram.sv. Self contained: make && ./obj_dir/Vtb_top
#
# Nothing here is synthesised - sdram_model.sv is a simulation model of the chip
# and is deliberately not in files.qip.

all: obj_dir/Vtb_top
	./obj_dir/Vtb_top

obj_dir/Vtb_top: tb_top.sv sdram_model.sv ../../rtl/sdram.sv tb_main.cpp
	verilator -cc --exe --build -O2 -Wno-fatal \
	  --top-module tb_top \
	  -CFLAGS "-O2" \
	  tb_top.sv sdram_model.sv ../../rtl/sdram.sv tb_main.cpp

clean:
	rm -rf obj_dir

.PHONY: all clean
