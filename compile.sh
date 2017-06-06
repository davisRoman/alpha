#!/bin/bash
export ARCH=arm
export CROSS_COMPILE=/home/davis/timesys/trunk/build_armv7l-timesys-linux-gnueabi/toolchain/bin/armv7l-timesys-linux-gnueabi-

pushd linux-3.10
   make imx_tsscve_defconfig
   make -j8
   echo ""
   echo ""
   echo "kernel will build without a problem"
   echo "Now if we try to look for the following loadable kernel module: linux-3.10/drivers/video/robopeak/rp_usbdisplay.ko"
   echo "it won't be found because it wasn't built.The kernel Kconfig system doesn't see it. I don't know why."
   echo "however, if I directly build the module then it works:"
   echo ""
   make SUBDIRS=drivers/video/robopeak/
popd

echo ""
echo ""
echo "******************************"
ls -la linux-3.10/drivers/video/robopeak/rp_usbdisplay.ko
echo "And now the kernel module appears! I have no idea why I have do this this second step"
echo "******************************"
