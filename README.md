# camera_vehicle_module

A VEHICL with a CAMERA

Purpose: Collect and analyze camera information

chipset: nxp IMX8mp

version: kernel 5.10.72.2.2.0

      u-boot 2021.04
u-boot

1)Unlock the compressed file(u-boot-imx_2025_07_09.tar.gz)

$mkdir u-boot-imx

$cd cd u-boot-imx

$tar cvfz u-boot-imx_2025_07_09.tar.gz

2) u-boot build

   $ ./uboot_make.sh imx8mp_mv_defconfig

   $ ./uboot_make.sh

======================================================================

Kernel

Unlock the compressed files

linux-imx.vol1.egg, linux-imx.vol2.egg, linux-imx.vol3.egg

linux-imx.vol4.egg, linux-imx.vol5.egg, linux-imx.vol6.egg

linux-imx.vol7.egg, linux-imx.vol8.egg, linux-imx.vol9.egg

decompress ========> linux-imx_20240913.tar.gz

Unlock the compressed file(linux-imx_20240913.tar.gz)

$mkdir linux-imx

$tar cvfz linux-imx_20240913.tar.gz

kernel build
$ ./kernel_make.sh mv_v8_defconfig

$ ./kernel_make.sh

====================================================================== 

camera module

    eosys_drv_m.c

    mdin_drv_m.c

    tp2860_drv_m.c

======================================================================
