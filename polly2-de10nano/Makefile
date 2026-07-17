# polly2-de10nano - display-path testbenches (Verilator)
#
#   make            - build + run all testbenches
#   make spg-sync   - spg.sv sync-geometry check: HS/VS periods and widths,
#                     VS edges aligned to the HS leading edge (CEA-861),
#                     DE exactly 1920x1080 per frame
#   make clean

VERILATOR ?= verilator

NJOBS := $(shell nproc 2>/dev/null || echo 4)

VFLAGS = -Wno-WIDTH -Wno-UNUSEDSIGNAL -j $(NJOBS) --build-jobs $(NJOBS) \
         -CFLAGS "-O3"

all: spg-sync

spg-sync: obj_dir/spg_sync_tb
	./obj_dir/spg_sync_tb

obj_dir/spg_sync_tb: rtl/spg.sv tb/spg_sync_tb.cpp
	$(VERILATOR) --cc --exe --build $(VFLAGS) rtl/spg.sv tb/spg_sync_tb.cpp -o spg_sync_tb

clean:
	rm -rf obj_dir

.PHONY: all spg-sync clean
