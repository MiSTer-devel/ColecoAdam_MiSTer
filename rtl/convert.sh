#srec_cat  bios.mif --mif   -o bios.hex --ascii_hex
srec_cat EOS.ROM --binary -o eos.hex --ascii_hex
srec_cat WRITER.ROM --binary -o writer.hex --ascii_hex
echo "remove the extra bytes at the top and bottom of the .hex"
