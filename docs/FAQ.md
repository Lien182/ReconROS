Frequently Asked Questions
==========================

Installation of Xilinx Platform Cable USB
-----------------------------------------

The installation of the Xilinx Platform Cable often causes problems and does
not work as expected. However, when following some simple rules it should work
rather easily.

The cable needs a firmware which is not stored in a non-volatile memory but
must be loaded each time the cable is connected. Therefore, you need to
install `fxload`. It might be included in you distribution's package
repositories or you can build it from scratch.

After that, you need to add specific udev rules to automatically load the
firmware when connecting the cable and, furthermore, granting access to the
cable for normal users. The typical udev file looks as follows, but you might
need to adjust it to your specific udev syntax.

```
SUBSYSTEM=="usb", ATTR{idVendor}=="03fd", ATTR{idProduct}=="0008", MODE="0666"
SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="03fd", ATTR{idProduct}=="0007", RUN+="/sbin/fxload -v -t fx2 -I /usr/share/xusbdfwu.hex -D $tempnode"
SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="03fd", ATTR{idProduct}=="0009", RUN+="/sbin/fxload -v -t fx2 -I /usr/share/xusb_xup.hex -D $tempnode"
SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="03fd", ATTR{idProduct}=="000d", RUN+="/sbin/fxload -v -t fx2 -I /usr/share/xusb_emb.hex -D $tempnode"
SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="03fd", ATTR{idProduct}=="000f", RUN+="/sbin/fxload -v -t fx2 -I /usr/share/xusb_xlp.hex -D $tempnode"
SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="03fd", ATTR{idProduct}=="0013", RUN+="/sbin/fxload -v -t fx2 -I /usr/share/xusb_xp2.hex -D $tempnode"
SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="03fd", ATTR{idProduct}=="0015", RUN+="/sbin/fxload -v -t fx2 -I /usr/share/xusb_xse.hex -D $tempnode"
```

We are referring to the `hex` files in `/usr/share`, but you probably do not
have the files located there. However, you can simply copy or link them from
the Xilinx installation under `/opt/Xilinx/14.7/ISE_DS/ISE/bin/lin64/`. Do not
change the path in the udev file instead of copying/linking the files to
`/usr/share`, since `impact` and `analyzer` expect them there. Furthermore,
make sure to not use the `hex` files from `/opt/Xilinx/14.7/ISE_DS/ISE/bin/lin
64/install_scripts/install_drivers/linux_drivers/pcusb` since they are of a
different version and do not work.

If you follow these guidelines exactly, the cable should work without any
problems and the led should light up when connecting.