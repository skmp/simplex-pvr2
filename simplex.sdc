# tile_engine_top runs on the HPS-provided core clock (h2f_user0_clk from the
# cyclonev_hps_interface hard IP). There is no top-level `clk` port.
#
# The clock reaches the fabric through a global clock-enable buffer named
#   ...|h2f_user0_clk[0]~CLKENA0   (drives CLKCTRL_G9, ~411 fanout = whole design)
# The bare HPS boundary name h2f_user0_clk[0] does NOT match get_pins (it is the
# hard-IP edge), so constrain the CLKENA buffer's output pin instead.
#
# The 10 ns period (100 MHz) is a target; reported Fmax is derived from real
# path delay regardless of this value.
create_clock -name core_clk -period 10.000 \
    [get_pins -compatibility_mode {*h2f_user0_clk*~CLKENA0*}]

derive_pll_clocks
derive_clock_uncertainty
