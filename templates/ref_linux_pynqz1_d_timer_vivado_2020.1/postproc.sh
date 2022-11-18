
rm tmp -r -f
mkdir tmp
BIF_TEMPLATE_PATH=bootgen_template/template.bif
TMP_BIF_PATH=tmp/file.bif
for i in Bitstreams/*.bit; do
    [ -f "$i" ] || break
    sed -r "s|template.bit|$i|" "$BIF_TEMPLATE_PATH" > "$TMP_BIF_PATH"
    bootgen -image "$TMP_BIF_PATH" -arch zynq -process_bitstream bin
done

