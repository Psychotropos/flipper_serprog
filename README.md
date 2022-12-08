# flipper_serprog
A serprog-based SPI programmer to be used in conjuction with flashrom for the Flipper Zero.

# Wiring guide
PA6 -> MISO
PA7 -> MOSI
PA4 -> CS
PB3 -> SCK
3.3V -> VCC

Note that depending on your SPI chip, additional pins might need to be held high (ideally via pull-up resistors, or bridged to the 3v3 rail). This applies to i.e. the /HOLD and /WP pins in Winbond chips.

If the target device is attached to via a SOP8 clip, it is recommended that the device is attached and the connections verified while the Flipper Zero is turned off. Fluctuations in the power use of the 3v3 rail on the Flipper Zero might otherwise cause issues (such as the SD card interface being unavailable until the device is restarted).

# Usage
Use flashrom to interface with the Flipper Zero via USB. Note that flipper_serprog makes use of the secondary VCP channel by default, meaning that the serprog interface will appear under a different tty number on the Flipper. 

This behaviour can be amended by changing "USB_VCP_CHANNEL" in the source code to 0 and re-compiling. Do note that the standard Flipper RPC interface will be unavailable while flipper_serprog is running if this is done.

Example:
flashrom -p serprog:dev=/dev/tty.usbmodemflip_<flipper_name>3 -r spi.bin
