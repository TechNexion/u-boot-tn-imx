#!/bin/sh
#################################################################################
# Copyright 2018 Technexion Ltd.
#
# Author: Richard Hu <richard.hu@technexion.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#################################################################################

DRIVE=/dev/sdX

#platform related parameters
#PLATFORM="imx8mm"
#SOC_TARGET="iMX8MM"
#SOC_DIR="iMX8M"
#DTBS="fsl-imx8mq-evk"
#DTBS="pico-imx8m"

BRANCH_VER="lf-5.10.y_2.0.0" #branch used by imx-mkimage and imx-atf under meta-imx
ATF_BRANCH_VER="lf_v2.4"
MKIMAGE_SRC_GIT_ID='5138add7602a50d1730a54d0b67219f0ce0666b4' #refer to 'imx-mkimage_git.inc' in Yocto
ATF_SRC_GIT_ID='ec35fef92b71a79075f214f8cff0738cd4482ed0' #refer to 'imx-atf_2.0.bbappend' in Yocto
DDR_FW_VER="8.12" #refer to the name of 'firmware-imx-8_8.8.bb'

FSL_MIRROR="https://www.nxp.com/lgfiles/NMG/MAD/YOCTO"
FIRMWARE_DIR="firmware_imx8"
MKIMAGE_DIR="imx-mkimage"
MKIMAGE_TARGET="flash_hdmi_spl_uboot"

SPL_ORI="spl/u-boot-spl.bin"
UBOOT_ORI="u-boot-nodtb.bin"
FW_DIR="firmware_imx8mq"
IMX_BOOT="flash.bin"
TWD=`pwd`

setup_platform()
{
	SOC=$( echo "${DTBS}" | cut -d' ' -f1 | grep -o 'imx8m[mqpn]\?' )
	if [ ${SOC} = "imx8m" ] || [ ${SOC} = "imx8mq" ] ; then
		PLATFORM="imx8mq"
		SOC_TARGET="iMX8M"
		SOC_DIR="iMX8M"
		IMX_BOOT_SEEK="33"
	elif [ ${SOC} = "imx8mm" ] ; then
		PLATFORM="imx8mm"
		SOC_TARGET="iMX8MM"
		SOC_DIR="iMX8M"
		IMX_BOOT_SEEK="33"
	elif [ ${SOC} = "imx8mp" ] ; then
		PLATFORM="imx8mp"
		SOC_TARGET="iMX8MP"
		SOC_DIR="iMX8M"
		IMX_BOOT_SEEK="32"
	elif [ ${SOC} = "imx8mn" ] ; then
		PLATFORM="imx8mn"
		SOC_TARGET="iMX8MN"
		SOC_DIR="iMX8M"
		IMX_BOOT_SEEK="32"
	else
		printf "Targest SOC isn't supported by this script\n"
		exit 1
	fi
}

install_firmware()
{
	cd ${TWD}
	#Get and Build NXP imx-mkimage tool
	if [ ! -d ${MKIMAGE_DIR} ] ; then
		git clone https://source.codeaurora.org/external/imx/imx-mkimage -b ${BRANCH_VER} || printf "Fails to fetch imx-mkimage source code \n"
		cd imx-mkimage
		git checkout ${MKIMAGE_SRC_GIT_ID}
	fi
	cd ${TWD}
	#Collect required firmware files to generate bootable binary
	if [ ! -d ${FIRMWARE_DIR} ] ; then
		mkdir ${FIRMWARE_DIR}
	fi

	cd ${FIRMWARE_DIR} && FWD=`pwd`
	
	#Get, build and copy the ARM Trusted Firmware
	if [ ! -d imx-atf ] ; then
		git clone https://source.codeaurora.org/external/imx/imx-atf -b ${ATF_BRANCH_VER} || printf "Fails to fetch ATF source code \n"
		cd imx-atf
		git checkout ${ATF_SRC_GIT_ID}
	fi

	PWD=$(pwd)
	[ -n "${PWD##*imx-atf}" ] && cd imx-atf
	if ( git diff-index --quiet HEAD -- plat/imx/imx8mm/imx8mm_bl31_setup.c ); then
		if [ -z "${DTBS##*imx8mm-axon*}" ]; then
			# AXON: Change UART2 base address to UART1 and released UART4 from M4
			sed -i 's/(RDC_PDAP_UART4, D1R | D1W),/(RDC_PDAP_UART4, D0R | D0W),/g' plat/imx/imx8m/imx8mm/imx8mm_bl31_setup.c
			sed -i 's/(0x30890000)/(0x30860000)/g' plat/imx/imx8m/imx8mm/include/platform_def.h
			rm build/${PLATFORM}/release/bl31.bin
		fi
	else
		if [ -n "${DTBS##*imx8mm-axon*}" ]; then
			git checkout plat/imx/imx8mm/imx8mm_bl31_setup.c
			git checkout plat/imx/imx8m/imx8mm/include/platform_def.h
			rm build/${PLATFORM}/release/bl31.bin
		fi
	fi

	if [ ! -f build/${PLATFORM}/release/bl31.bin ] ; then
		rm -rf build
		make PLAT=${PLATFORM} bl31 || printf "Fails to build ATF firmware \n"
	fi	
	if [ -f build/${PLATFORM}/release/bl31.bin ] ; then
		printf "Copy build/${PLATFORM}/release/bl31.bin to $MKIMAGE_DIR \n"
		cp build/${PLATFORM}/release/bl31.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
	else
		printf "Cannot find release/bl31.bin \n"
	fi
	
	#Get and copy the DDR and HDMI firmware
	cd ${FWD}
	if [ ! -d firmware-imx-${DDR_FW_VER} ] ; then
		wget ${FSL_MIRROR}/firmware-imx-${DDR_FW_VER}.bin && \
		chmod +x firmware-imx-${DDR_FW_VER}.bin && \
		./firmware-imx-${DDR_FW_VER}.bin || \
		printf "Fails to fetch DDR firmware \n"
	fi

	if [ -d firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys ] ; then
		if [ ${SOC} = "imx8mp" ] ; then
			cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_1d_dmem_202006.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
			cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_1d_imem_202006.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
			cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_2d_dmem_202006.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
			cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_2d_imem_202006.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
		else
			cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_1d_dmem.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
			cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_1d_imem.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
			cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_2d_dmem.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
			cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_2d_imem.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
		fi
			cp firmware-imx-${DDR_FW_VER}/firmware/hdmi/cadence/signed_hdmi_imx8m.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
	else
		printf "Cannot find DDR firmware \n"
	fi
}

install_uboot_dtb()
{
	#Copy uboot binary
	cd ${TWD}
	if [ -f u-boot-nodtb.bin ] ; then
		cp u-boot-nodtb.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
	else
		printf "Cannot find u-boot-nodtb.bin. Please build u-boot first! \n"
	fi

	#Copy SPL binary
	cd ${TWD}
	if [ -f spl/u-boot-spl.bin ] ; then
		cp spl/u-boot-spl.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
	else
		printf "Cannot find spl/u-boot-spl.bin. Please build u-boot first! \n"
	fi

	#Copy device tree file
	cd ${TWD}
	for DTB in ${DTBS}
	do
		if [ -f arch/arm/dts/${DTB} ] ; then
			cp arch/arm/dts/${DTB} ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
		else
			printf "Cannot find arch/arm/dts/${DTB} . Please build u-boot first! \n"
		fi
	done
}

generate_imx_boot()
{
	cd ${TWD}
	#Before generating the flash.bin, transfer the mkimage generated by U-Boot to iMX8M folder
	if [ -f tools/mkimage ] ; then
		cp tools/mkimage ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/mkimage_uboot
	else
		printf "Cannot find tools/mkimage. Please build u-boot first! \n"
	fi

	#Generate bootable binary (This binary contains SPL and u-boot.bin) for flashing
	cd ${MKIMAGE_DIR}
	make SOC=${SOC_TARGET} dtbs="${DTBS}" ${MKIMAGE_TARGET} && \
	printf "Make target: ${MKIMAGE_TARGET} and generate flash.bin... \n" || printf "Fails to generate flash.bin... \n"
}

flash_imx_boot()
{
	cd ${TWD}
	if [ ! -b $DRIVE ]; then
     echo "$DRIVE doesn't exist !!!"
     exit
	fi
	sudo umount ${DRIVE}?
	sleep 0.1
	sudo dd if=${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/${IMX_BOOT} of=${DRIVE} bs=1k seek=${IMX_BOOT_SEEK} oflag=dsync status=progress && \
	printf "Flash flash.bin... \n" || printf "Fails to flash flash.bin... \n"
}

usage()
{
    echo -e "\nUsage: install_uboot_imx8mq.sh
    Optional parameters: [-d disk-path] [-b DTBS_name] [-t] [-c] [-h]"
	echo "
    * This script is used to download required firmware files, generate and flash bootable u-boot binary
    *
    * [-d disk-path]: specify the disk to flash u-boot binary, e.g., /dev/sdd
    * [-b dtb_name]: specify the name of dtb, which will be included in FIT image
    * [-t]: target u-boot binary is without HDMI firmware
    * [-c]: clean temporary directory
    * [-h]: help

    For example:

    i.mx8MM:
    * PICO-IMX8MM with PICO-PI-IMX8 baseDTBS:
    ./install_uboot_imx8.sh -b imx8mm-pico-pi.dtb -b imx8mm-pico-wizard.dtb -d /dev/sdX
	
    * EDM-G-IMX8MM with WB:
    ./install_uboot_imx8.sh -b imx8mm-edm-g-wb.dtb -d /dev/sdX

    i.mx8MQ:
    * EDM-IMX8MQ with EDM-WIZARD baseDTBS:
    ./install_uboot_imx8.sh -b imx8mq-edm-wizard.dtb -d /dev/sdX

    * PICO-IMX8MQ with PICO-PI-IMX8 baseDTBS:
    ./install_uboot_imx8.sh -b imx8mq-pico-pi.dtb -b imx8mq-pico-wizard.dtb -d /dev/sdX

    i.mx8MP:
    * AXON-IMX8MP:
    ./install_uboot_imx8.sh -b imx8mp-axon.dtb -d /dev/sdX
    
    * EDM-G-IMX8MP with WB:
    ./install_uboot_imx8.sh -b imx8mp-edm-g.dtb -d /dev/sdX

    i.MX8MN:
    * EDM-G-IMX8MN with WB:
    ./install_uboot_imx8.sh -b imx8mn-edm-g.dtb -d /dev/sdX
"
}

print_settings()
{
	echo "*************************************************************"
	echo "Before run this script, please build u-boot first!
	"
	echo "The disk path to flash u-boot: $DRIVE"
	echo "The default DTB name: ${DTBS}"
	echo "Make target: ${PLATFORM}"
	echo "Make target: ${MKIMAGE_TARGET}"
	echo "SOC platform: ${SOC}"
	echo "*************************************************************
		
	"
}

if [ $# -eq 0 ]; then
	usage
	exit 1
fi

while getopts "tchd:b:" OPTION
do
    case $OPTION in
        d) 
           DRIVE="$OPTARG"
           ;;
        b) 
           DTBS="$DTBS $OPTARG"
           ;;
        t) 
           MKIMAGE_TARGET='flash_spl_uboot';
           ;;
		c) 
		   rm -rf ${FIRMWARE_DIR} ${MKIMAGE_DIR}
		   echo "Clean ${FIRMWARE_DIR} ${MKIMAGE_DIR}..."
		   exit
		   ;;
        ?|h) usage
           exit
           ;;
    esac
done

DTBS=$(echo ${DTBS} | cut -c 1-)

if [ "$(id -u)" = "0" ]; then
   echo "This script can not be run as root"
   exit 1
fi

#if [ ! -b $DRIVE ]
#then
#   echo Target block device $DRIVE does not exist
#   usage
#   exit 1
#fi

setup_platform
print_settings
install_firmware
install_uboot_dtb
generate_imx_boot
flash_imx_boot

