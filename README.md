# DPS1200FB
Basic fan control for DPS1200FB or similar power supplies

# Shoutout
Many thanks to the work done in this repo a few years ago https://github.com/raplin/DPS-1200FB

# AVRDUDE Commands
avrdude -c usbtiny -P usb -p attiny85

avrdude -c usbtiny -P usb -p attiny85 -e -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m -U lock:w:0xFF:m

avrdude -c usbtiny -P usb -p attiny85 -e -U flash:w:firmware.hex:a


