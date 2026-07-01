# refsw2 ISP/TSP triangle-setup core - simulation build
#
#   make vectors    - regenerate golden VRAM image + expected coefficients (MODE=tri|quad)
#   make fp         - FP-primitive fuzz test (add/mul/div vs host IEEE-754)
#   make sim        - COMBINATIONAL setup TB, bit-exact (uses MODE)
#   make sim-tri / sim-quad
#   make seq        - SEQUENCED (one mul+add/unit, one reciprocal) setup TB (behavioral FP)
#   make seq-tri / seq-quad
#   make ip         - sequenced design wired to Altera FP IP, run via IP stubs (uses MODE)
#   make ip-tri / ip-quad
#   make all        - fp + comb + sequenced + IP-path, tri & quad
#   make quartus    - synthesize the sequenced+IP design (needs Quartus on PATH)
#   make clean
#
# MODE selects the scene fed to gen_vectors (default: tri).

VERILATOR ?= verilator
VFLAGS    = -Wno-WIDTH -Wno-UNOPTFLAT -Wno-UNSIGNED -Wno-DECLFILENAME

# combinational (bit-exact) design
RTL_COMB  = rtl/fp_add.sv rtl/fp_mul.sv rtl/fp_div.sv \
            rtl/plane_stepper.sv rtl/u8_to_float.sv \
            rtl/isp_setup.sv rtl/tsp_setup.sv rtl/tri_setup_top.sv

# sequenced (area-optimized) design
RTL_SEQ   = rtl/fp_add.sv rtl/fp_mul.sv rtl/fp_div.sv \
            rtl/fp_mac.sv rtl/fp_recip.sv rtl/fmac_seq.sv rtl/u8_to_float.sv \
            rtl/isp_setup_seq.sv rtl/tsp_setup_seq.sv rtl/tri_setup_seq_top.sv

BUILD = build
CWD  := $(shell pwd)
MODE ?= tri
MODEARG = $(if $(filter quad,$(MODE)),quad,)

.PHONY: all vectors fp sim sim-tri sim-quad seq seq-tri seq-quad ip ip-tri ip-quad quartus clean

all: fp sim-tri sim-quad seq-tri seq-quad ip-tri ip-quad

# ---- golden vectors ----
$(BUILD)/gen_vectors: sim/gen_vectors.c | $(BUILD)
	gcc -O2 -Wall -o $@ $< -lm

vectors: $(BUILD)/gen_vectors
	cd $(CWD) && ./$(BUILD)/gen_vectors $(MODEARG)

$(BUILD):
	mkdir -p $(BUILD)

# ---- FP primitive fuzz ----
fp: | $(BUILD)
	$(VERILATOR) --cc --exe --build $(VFLAGS) --top-module fp_prims \
	  -Irtl tb/fp_prims.sv $(CWD)/tb/fp_prims_tb.cpp \
	  --Mdir $(BUILD)/obj_fpprims -o fp_prims_tb
	./$(BUILD)/obj_fpprims/fp_prims_tb

# ---- combinational setup TB (bit-exact) ----
sim: vectors
	$(VERILATOR) --cc --exe --build $(VFLAGS) --top-module tri_setup_top \
	  -CFLAGS "-I$(CWD)/build" \
	  -Irtl $(RTL_COMB) $(CWD)/tb/tri_setup_tb.cpp \
	  --Mdir $(BUILD)/obj_setup -o tri_setup_tb
	cd $(CWD) && ./$(BUILD)/obj_setup/tri_setup_tb

sim-tri:  ; $(MAKE) sim MODE=tri
sim-quad: ; $(MAKE) sim MODE=quad

# ---- sequenced setup TB (one mul+add/unit, one reciprocal; ULP tolerance) ----
seq: vectors
	$(VERILATOR) --cc --exe --build $(VFLAGS) --top-module tri_setup_seq_top \
	  -CFLAGS "-I$(CWD)/build" \
	  -Irtl $(RTL_SEQ) $(CWD)/tb/tri_setup_seq_tb.cpp \
	  --Mdir $(BUILD)/obj_seqtop -o tri_setup_seq_tb
	cd $(CWD) && ./$(BUILD)/obj_seqtop/tri_setup_seq_tb

seq-tri:  ; $(MAKE) seq MODE=tri
seq-quad: ; $(MAKE) seq MODE=quad

# ---- IP-path test: sequenced design wired to the Altera FP IP, exercised in
# Verilator via behavioral stubs (tb/altera_fp_stubs.sv) that match the IP
# ports + latency. Defines SYNTHESIS (-> IP datapath) but NOT USE_MIF_ROM
# (-> $readmemh still loads the scene). Validates latency/valid alignment and
# the neg/sub sign handling of the IP wrappers; arithmetic is RNE here, the real
# IP rounds ties-away, hence the TB's ULP tolerance.
RTL_IP    = rtl/fp_add.sv rtl/fp_mul.sv rtl/fp_div.sv rtl/u8_to_float.sv \
            tb/altera_fp_stubs.sv rtl/fmac_seq.sv rtl/fp_recip.sv \
            rtl/isp_setup_seq.sv rtl/tsp_setup_seq.sv rtl/tri_setup_seq_top.sv
ip: vectors
	$(VERILATOR) --cc --exe --build $(VFLAGS) -DSYNTHESIS -Wno-MULTITOP \
	  --top-module tri_setup_seq_top -CFLAGS "-I$(CWD)/build" \
	  -Irtl $(RTL_IP) $(CWD)/tb/tri_setup_seq_tb.cpp \
	  --Mdir $(BUILD)/obj_ip -o tri_setup_ip_tb
	cd $(CWD) && ./$(BUILD)/obj_ip/tri_setup_ip_tb

ip-tri:  ; $(MAKE) ip MODE=tri
ip-quad: ; $(MAKE) ip MODE=quad

# ---- Quartus synthesis of the sequenced design (requires Quartus on PATH) ----
quartus: vectors
	@command -v quartus_map >/dev/null || { echo "Quartus not on PATH"; exit 1; }
	quartus_map simplex && quartus_fit simplex && quartus_sta simplex
	@echo "Reports in output_files/ (simplex.fit.rpt, simplex.sta.rpt)"

clean:
	rm -rf $(BUILD)
