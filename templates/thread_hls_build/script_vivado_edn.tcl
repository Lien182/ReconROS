<<reconos_preproc>>
read_verilog [ glob hls/sol/syn/verilog/*.v ]
#read_xdc rt_imp.xdc
synth_design -top rt_imp -part <<PART>> -mode out_of_context

write_edif rt_imp.edn