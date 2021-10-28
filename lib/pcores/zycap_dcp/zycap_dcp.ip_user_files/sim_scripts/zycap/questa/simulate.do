onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib zycap_opt

do {wave.do}

view wave
view structure
view signals

do {zycap.udo}

run -all

quit -force
