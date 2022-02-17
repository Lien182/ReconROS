
rm tmp -r -f
mkdir tmp
BIF_TEMPLATE_PATH=bootgen_template/template.bif
TMP_BIF_PATH=tmp/file.bif
for i in Bitstreams/*.bit; do
    [ -f "$i" ] || break
    sed -r "s|template.bit|$i|" "$BIF_TEMPLATE_PATH" > "$TMP_BIF_PATH"
    bootgen -image "$TMP_BIF_PATH" -arch zynq -process_bitstream bin
done


rm Bitstreams/boot.bin -f
rm bin_creation/download.bit -f
cp Bitstreams/Config_reconf_0_reconf_1_implement_full.bit bin_creation/download.bit
bootgen -image bin_creation/bootimage.bif -o Bitstreams/boot.bin

