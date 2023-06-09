<<reconos_preproc>>
set_directive_interface -mode ap_fifo "rt_imp_<<NAME>>" osif_sw2hw
set_directive_interface -mode ap_fifo "rt_imp_<<NAME>>" osif_hw2sw
set_directive_interface -mode ap_fifo "rt_imp_<<NAME>>" memif_hwt2mem
set_directive_interface -mode ap_fifo "rt_imp_<<NAME>>" memif_mem2hwt
set_directive_interface -mode ap_ctrl_none "rt_imp_<<NAME>>"