<<reconos_preproc>>

<<generate for SLOTS(Reconfigurable==True)>>

create_pblock pblock_slot_<<Id>>
add_cells_to_pblock [get_pblocks pblock_slot_<<Id>>] [get_cells -quiet [list design_1_i/slot_<<Id>>]]
resize_pblock [get_pblocks pblock_slot_<<Id>>] -add {<<Region>>}
set_property RESET_AFTER_RECONFIG true [get_pblocks pblock_slot_<<Id>>]
set_property SNAPPING_MODE ON [get_pblocks pblock_slot_<<Id>>]
set_property IS_SOFT FALSE [get_pblocks pblock_slot_<<Id>>]
set_property HD.RECONFIGURABLE true [get_cells design_1_i/slot_<<Id>>]

<<end generate>>
