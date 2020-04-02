<<reconos_preproc>>
create_clock -name ap_clk -period <<CLKPRD>> -waveform {0.000 <<CLKPRD2>>} [get_ports ap_clk]
set_property HD.CLK_SRC BUFGCTRL_X0Y0 [get_ports ap_clk]
