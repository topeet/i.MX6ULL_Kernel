#!/bin/bash

export ARCH=arm

#export CROSS_COMPILE=/usr/local/arm/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin/arm-none-linux-gnueabi-

export CROSS_COMPILE=arm-linux-gnueabihf-

#make mrproper  # means CLEAN

make imx_v7_defconfig

#if   [ "$1" = "nand" ]
#then
#	cp arch/arm/boot/dts/imx6ul-14x14-evk_nand.dts arch/arm/boot/dts/imx6ul-14x14-evk.dts
#else
#	cp arch/arm/boot/dts/imx6ul-14x14-evk_emmc.dts arch/arm/boot/dts/imx6ul-14x14-evk.dts
#fi

make uImage LOADADDR=0x10008000 -j8

make modules

make topeet_emmc_4_3.dtb
make topeet_emmc_5_0.dtb
make topeet_emmc_7_0.dtb
make topeet_emmc_1024x600.dtb
make topeet_emmc_9_7.dtb
make topeet_emmc_10_1.dtb
make topeet_emmc_hdmi.dtb

make topeet_nand_4_3.dtb
make topeet_nand_5_0.dtb
make topeet_nand_7_0.dtb
make topeet_nand_1024x600.dtb
make topeet_nand_9_7.dtb
make topeet_nand_10_1.dtb
make topeet_nand_hdmi.dtb

cd ./arch/arm/boot/dts/
./create_dtb imx6ul_topeet_nand.dtb topeet_nand_4_3.dtb topeet_nand_7_0.dtb topeet_nand_10_1.dtb topeet_nand_1024x600.dtb topeet_nand_5_0.dtb topeet_nand_9_7.dtb topeet_nand_hdmi.dtb
