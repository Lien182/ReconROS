<<reconos_preproc>>
read_verilog [ glob hls/sol/impl/verilog/*.v ]
<<if VIVADO=="2021">>
read_verilog [ glob hls/sol/impl/ip/hdl/ip/*.v ]
<<end if>>
#read_xdc rt_imp_<<NAME>>.xdc
synth_design -top rt_imp_<<NAME>> -part <<PART>> -mode out_of_context

write_edif rt_imp_<<NAME>>.edn
