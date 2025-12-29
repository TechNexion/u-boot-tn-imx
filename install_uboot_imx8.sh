#!/bin/bash
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

BRANCH_VER="lf-6.12.49_2.2.0" #branch used by imx-mkimage and imx-atf under meta-imx
ATF_BRANCH_VER="lf_v2.12"
MKIMAGE_SRC_GIT_ID='be80fadd5e7988214149a2bc48daac1b0950d4c2' #refer to 'imx-mkimage_git.inc' in Yocto
ATF_SRC_GIT_ID='a266ff458c2526a6474036a5c6648be6fdc54fe3' #refer to 'imx-atf_2.12.bb' in Yocto
DDR_FW_VER="8.30-3fa84fd" #refer to the name of 'firmware-imx-8.x.bb'
ELE_FW_VER="2.0.4-93492e0" ##refer to the "{PV of firmware-ele-imx_2.0.2.bb}"-"{IMX_SRCREV_ABBREV}"

FSL_MIRROR="https://www.nxp.com/lgfiles/NMG/MAD/YOCTO"
FIRMWARE_DIR="imx-boot_generation"
MKIMAGE_DIR="imx-mkimage"
MKIMAGE_TARGET="flash_hdmi_spl_uboot"

SPL_ORI="spl/u-boot-spl.bin"
UBOOT_ORI="u-boot-nodtb.bin"
IMX_BOOT="flash.bin"
TWD=`pwd`
ATF_BOOT_UART_BASE="0x30890000"

# Config for i.mx95
IMX_SM_GIT_REPO="https://github.com/nxp-imx/imx-sm.git"
IMX_SM_BRANCH_VER="master"
IMX_SM_CONFIG="mx95evk"
IMX_OEI_GIT_REPO="https://github.com/TechNexion/imx-oei.git"
IMX_OEI_BRANCH_VER="tn-imx_6.12.49_2.2.0"
IMX_OEI_CONFIG="edm-imx95"
ARM_TOOLCHAIN_VER_DEFAULT="14.2.rel1"

setup_platform()
{
	SOC=$( echo "${DTBS}" | cut -d'-' -f1 )
	case "${SOC}" in
		imx8m|imx8mq)
			PLATFORM="imx8mq"
			SOC_TARGET="iMX8M"
			SOC_DIR="iMX8M"
			IMX_BOOT_SEEK="33"
			;;
		imx8mm)
			PLATFORM="imx8mm"
			SOC_TARGET="iMX8MM"
			SOC_DIR="iMX8M"
			IMX_BOOT_SEEK="33"
			;;
		imx8mp)
			PLATFORM="imx8mp"
			SOC_TARGET="iMX8MP"
			SOC_DIR="iMX8M"
			IMX_BOOT_SEEK="32"
			;;
		imx8mn)
			PLATFORM="imx8mn"
			SOC_TARGET="iMX8MN"
			SOC_DIR="iMX8M"
			IMX_BOOT_SEEK="32"
			;;
		imx91)
			PLATFORM="imx91"
			SOC_TARGET="iMX91"
			SOC_DIR="iMX91"
			SILICON_REV=${SILICON_REV:-A0}
			IMX_BOOT_SEEK="32"
			MKIMAGE_TARGET="flash_singleboot"
			;;
	    imx93)
			PLATFORM="imx93"
			SOC_TARGET="iMX9"
			SOC_DIR="iMX93"
			SILICON_REV=${SILICON_REV:-A1}
			IMX_BOOT_SEEK="32"
			MKIMAGE_TARGET="flash_singleboot"
			;;
		imx95)
			PLATFORM="imx95"
			SOC_TARGET="iMX95"
			SOC_DIR="iMX95"
			SILICON_REV=${SILICON_REV:-B0}
			IMX_BOOT_SEEK="32"
			MKIMAGE_TARGET="flash_a55"
			RAM_SIZE=${RAM_SIZE:-8gb}
			;;
		*)
			printf "Target SOC isn't supported by this script\n"
			exit 1
			;;
	esac
}

install_firmware()
{
	cd ${TWD}
	#Get and Build NXP imx-mkimage tool
	if [ ! -d ${MKIMAGE_DIR} ] ; then
		git clone https://github.com/nxp-imx/imx-mkimage.git -b ${BRANCH_VER} || printf "Fails to fetch imx-mkimage source code \n"
		cd imx-mkimage
		git checkout -b ${BRANCH_VER}_local ${MKIMAGE_SRC_GIT_ID}
		sed -i 's|dtb = evk.dtb|dtb = $(dtbs)|g' iMX8M/soc.mak
	fi
	cd ${TWD}
	#Collect required firmware files to generate bootable binary
	if [ ! -d ${FIRMWARE_DIR} ] ; then
		mkdir ${FIRMWARE_DIR}
	fi

	cd ${FIRMWARE_DIR} && FWD=`pwd`

	#Get, build and copy the ARM Trusted Firmware
	if [ ! -d imx-atf ] ; then
		git clone https://github.com/nxp-imx/imx-atf.git -b ${ATF_BRANCH_VER} || printf "Fails to fetch ATF source code \n"
		cd imx-atf
		git checkout -b ${BRANCH_VER}_local ${ATF_SRC_GIT_ID}
	fi

	PWD=$(pwd)
	[ -n "${PWD##*imx-atf}" ] && cd imx-atf

	# Patch for imx-atf: Fix compilation error for imx95
	# This patch is specific for imx-atf with branch "lf_v2.12"
	if [ ${SOC_DIR} == "iMX95" ]; then
		if ( git diff-index --quiet HEAD -- plat/imx/imx9/imx95/imx95_m7.c ); then
				sed -i '139a {
			152i }
			' plat/imx/imx9/imx95/imx95_m7.c
		fi
	fi
	if ( git diff-index --quiet HEAD -- plat/imx/imx8mm/imx8mm_bl31_setup.c ); then
		if [ -z "${DTBS##*imx8mm-axon*}" ]; then
			# AXON: Change UART2 base address to UART1 and released UART4 from M4
			sed -i 's/(RDC_PDAP_UART4, D1R | D1W),/(RDC_PDAP_UART4, D0R | D0W),/g' plat/imx/imx8m/imx8mm/imx8mm_bl31_setup.c
			rm build/${PLATFORM}/release/bl31.bin
			ATF_BOOT_UART_BASE="0x30860000"
		fi
	else
		if [ -n "${DTBS##*imx8mm-axon*}" ]; then
			git checkout plat/imx/imx8mm/imx8mm_bl31_setup.c
			rm build/${PLATFORM}/release/bl31.bin
		fi
	fi

	if [ ! -f build/${PLATFORM}/release/bl31.bin ] ; then
		rm -rf build
		make PLAT=${PLATFORM} IMX_BOOT_UART_BASE=${ATF_BOOT_UART_BASE} bl31 || printf "Fails to build ATF firmware \n"
	fi
	if [ -f build/${PLATFORM}/release/bl31.bin ] ; then
		printf "Copy build/${PLATFORM}/release/bl31.bin to $MKIMAGE_DIR \n"
		cp build/${PLATFORM}/release/bl31.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
	else
		printf "Cannot find release/bl31.bin \n"
	fi

	#Fetch and copy the DDR and HDMI firmware
	cd ${FWD}
	if [ ! -d firmware-imx-${DDR_FW_VER} ] ; then
		wget ${FSL_MIRROR}/firmware-imx-${DDR_FW_VER}.bin && \
		chmod +x firmware-imx-${DDR_FW_VER}.bin && \
		./firmware-imx-${DDR_FW_VER}.bin || \
		printf "Fails to fetch DDR firmware \n"
	fi

	if [ -d firmware-imx-${DDR_FW_VER}/firmware ] ; then
		case ${SOC} in
			imx8mp)
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_1d_dmem_202006.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_1d_imem_202006.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_2d_dmem_202006.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_2d_imem_202006.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/hdmi/cadence/signed_hdmi_imx8m.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				;;
			imx93|imx91)
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_imem_1d_v202201.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_dmem_1d_v202201.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_imem_2d_v202201.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_dmem_2d_v202201.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				;;
			imx95)
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr5_dmem_qb_v202409.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr5_dmem_v202409.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr5_imem_qb_v202409.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr5_imem_v202409.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				;;
			*)
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_1d_dmem.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_1d_imem.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_2d_dmem.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				cp firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys/lpddr4_pmu_train_2d_imem.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
				;;
		esac
	else
		printf "Cannot find firmware \n"
	fi

	#Fetch and copy EdgeLock Secure Enclave firmware
	if [ "${SOC_DIR}" = "iMX93" ] || [ "${SOC_DIR}" = "iMX91" ] || [ "${SOC_DIR}" = "iMX95" ]; then
		SOC_LOWER=$(echo $SOC_DIR | sed 's/^i//' | tr '[:upper:]' '[:lower:]')
		REV_LOWER=$(echo "${SILICON_REV}" | tr '[:upper:]' '[:lower:]')
		AHAB_IMG="${SOC_LOWER}${REV_LOWER}-ahab-container.img"

		if [ "${SOC_DIR}" = "iMX93" ] && [ "${SILICON_REV}" = "A0" ]; then
			if [ ! -d firmware-sentinel-0.11 ] ; then
				wget ${FSL_MIRROR}/firmware-sentinel-0.11.bin
				chmod +x firmware-sentinel-0.11.bin
				./firmware-sentinel-0.11.bin
			fi

			cp firmware-sentinel-0.11/mx93a0-ahab-container.img ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
			printf "Copy firmware-sentinel-0.11/mx93a0-ahab-container.img to $MKIMAGE_DIR \n"
		else
			if [ ! -d firmware-ele-imx-${ELE_FW_VER} ] ; then
				wget ${FSL_MIRROR}/firmware-ele-imx-${ELE_FW_VER}.bin
				chmod +x firmware-ele-imx-${ELE_FW_VER}.bin
				./firmware-ele-imx-${ELE_FW_VER}.bin
			fi
			cp firmware-ele-imx-${ELE_FW_VER}/${AHAB_IMG} ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
			printf "Copy firmware-ele-imx-${ELE_FW_VER}/${AHAB_IMG} to $MKIMAGE_DIR \n"
		fi
	fi
}

install_uboot_dtb()
{
	#Copy uboot binary
	cd ${TWD}
	if [ "${SOC_DIR}" = "iMX93" ] || [ "${SOC_DIR}" = "iMX91" ] || [ "${SOC_DIR}" = "iMX95" ] ; then
		cp u-boot.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
	elif [ -f u-boot-nodtb.bin ] ; then
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

fetch_oei()
{
	cd ${TWD}
	cd ${FIRMWARE_DIR} && FWD=`pwd`
	if [ ! -d imx-oei ] ; then
		git clone ${IMX_OEI_GIT_REPO} -b ${IMX_OEI_BRANCH_VER} || printf "Fails to fetch imx-oei source code from TechNexion git repo\n"
		cd imx-oei
	fi
}

fetch_sm()
{
	cd ${TWD}
	cd ${FIRMWARE_DIR} && FWD=`pwd`
	if [ ! -d imx-sm ] ; then
		git clone ${IMX_SM_GIT_REPO} -b ${IMX_SM_BRANCH_VER} || printf "Fails to fetch imx-sm source code from TechNexion git repo\n"
		cd imx-sm
	fi

}

prepare_arm_toolchain()
{
	cd ${TWD}
	cd ${FIRMWARE_DIR} && FWD=`pwd`
	if [ -d arm-gnu-toolchain-*-x86_64-arm-none-eabi ] ; then
		return
	fi

	if [ -f imx-sm/sm/makefiles/common.mak ] ; then
		ARM_TOOLCHAIN_VER=$(grep "TC_VERSION ?=" imx-sm/sm/makefiles/common.mak | cut -d'=' -f2 | xargs)
	fi
	if [ -z "${ARM_TOOLCHAIN_VER}" ] ; then
		ARM_TOOLCHAIN_VER=${ARM_TOOLCHAIN_VER_DEFAULT}
	fi
	wget "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VER}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VER}-x86_64-arm-none-eabi.tar.xz" && \
	( tar xvf arm-gnu-toolchain-${ARM_TOOLCHAIN_VER}-x86_64-arm-none-eabi.tar.xz && \
	rm arm-gnu-toolchain-${ARM_TOOLCHAIN_VER}-x86_64-arm-none-eabi.tar.xz ) || \
	printf "Fails to fetch ARM toolchain \n"
}

generate_sm_image()
{
	cd ${TWD}
	cd ${FIRMWARE_DIR} && FWD=`pwd`

	if [ -e imx-sm/build/${IMX_SM_CONFIG}/m33_image.bin ] ; then
		cp imx-sm/build/${IMX_SM_CONFIG}/m33_image.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
		return
	fi

	if [ -d arm-gnu-toolchain-*-x86_64-arm-none-eabi ] ; then
		export TOOLS=${FWD}
	else
		printf "Cannot find ARM toolchain \n"
	fi

	cd imx-sm && \
	make config=${IMX_SM_CONFIG} all && \
	cp build/${IMX_SM_CONFIG}/m33_image.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR} || \
	printf "Fails to generate SM firmware \n"
}

generate_oei_image()

{
	cd ${TWD}
	cd ${FIRMWARE_DIR} && FWD=`pwd`

	if [ -e imx-oei/build/${IMX_OEI_CONFIG}/ddr/oei-m33-ddr.bin ] ; then
		cp imx-oei/build/${IMX_OEI_CONFIG}/ddr/oei-m33-ddr.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
		return
	fi

	if [ -d arm-gnu-toolchain-*-x86_64-arm-none-eabi ] ; then
		export TOOLS=${FWD}
	else
		printf "Cannot find ARM toolchain \n"
	fi

	cd imx-oei
	if [ "${SILICON_REV}" = "A0" ] ; then
		make board=${IMX_OEI_CONFIG} oei=ddr r=A0 DEBUG=1 DDR_CONFIG=XIMX95LPD5EVK19_6400mbps_train_timing_a1
	elif [ "${RAM_SIZE}" = "4gb" ] ; then
		make board=${IMX_OEI_CONFIG} oei=ddr r=B0 DEBUG=1 DDR_CONFIG=lpddr5_6400mbps_train_timing_4gb
	elif [ "${RAM_SIZE}" = "8gb" ] ; then
		make board=${IMX_OEI_CONFIG} oei=ddr r=B0 DEBUG=1 DDR_CONFIG=lpddr5_6400mbps_train_timing_8gb
	elif [ "${RAM_SIZE}" = "16gb" ] ; then
		make board=${IMX_OEI_CONFIG} oei=ddr r=B0 DEBUG=1 DDR_CONFIG=lpddr5_6400mbps_train_timing_16gb
	fi

	cp build/${IMX_OEI_CONFIG}/ddr/oei-m33-ddr.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR} || \
	printf "Fails to generate OEI firmware \n"
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
	if [ "${SOC_DIR}" = "iMX93" ] && [ "${SILICON_REV}" = "A0" ]; then
		make SOC=${SOC_TARGET} REV=${SILICON_REV} dtbs="${DTBS}" ${MKIMAGE_TARGET} && \
		printf "Make target: ${MKIMAGE_TARGET} and generate flash.bin... \n" || printf "Fails to generate flash.bin... \n"
	elif [ "${SOC_DIR}" = "iMX95" ]; then
		if [ "${SILICON_REV}" = "A0" ] || [ "${SILICON_REV}" = "B0" ]; then
			make SOC=${SOC_TARGET} REV=${SILICON_REV} OEI=YES LPDDR_TYPE=lpddr5 dtbs="${DTBS}" ${MKIMAGE_TARGET} && \
			printf "Make target: ${MKIMAGE_TARGET} and generate flash.bin... \n" || printf "Fails to generate flash.bin... \n"
		fi
	else
		make SOC=${SOC_TARGET} dtbs="${DTBS}" ${MKIMAGE_TARGET} && \
		printf "Make target: ${MKIMAGE_TARGET} and generate flash.bin... \n" || printf "Fails to generate flash.bin... \n"
	fi
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
	Optional parameters: [-d disk-path] [-b DTBS_name] [-s rev] [-t] [-c] [-h]"
	echo "
	* This script is used to download required firmware files, generate and flash bootable u-boot binary
	*
	* [-d disk-path]: specify the disk to flash u-boot binary, e.g., /dev/sdd
	* [-b dtb_name]: specify the name of dtb, which will be included in FIT image
	* [-s rev]: specify the silicon revision for i.mx9 family to apply corresponding ELE firmware
				Options for i.mx93: A0, A1(default)
							i.mx95: A0, B0(default)
	* [-r ram_size]: specify the size of RAM for i.mx95
				Options for i.mx95: 8gb(default), 4gb, 16gb
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

	* EDM-G-IMX8MP with WB/WIZARD:
	./install_uboot_imx8.sh -b imx8mp-edm-g.dtb -d /dev/sdX

	* SC-IMX8MP:
	./install_uboot_imx8.sh -b imx8mp-sc.dtb -d /dev/sdX

	* TEK-IMX8MP:
	./install_uboot_imx8.sh -b imx8mp-tek.dtb -d /dev/sdX

	* TEK-IMX8MP with flexspi boot (only generate flash.bin):
	./install_uboot_imx8.sh -b imx8mp-tek.dtb -f -d /dev/null

	* TEP-IMX8MP:
	./install_uboot_imx8.sh -b imx8mp-tep.dtb -d /dev/sdX

	* TEP-IMX8MP with flexspi boot (only generate flash.bin):
	./install_uboot_imx8.sh -b imx8mp-tep.dtb -f -d /dev/null

	i.MX8MN:
	* EDM-G-IMX8MN with WB:
	./install_uboot_imx8.sh -b imx8mn-edm-g.dtb -d /dev/sdX

	i.MX93/i.MX91:
	* AXON-IMX93:
	./install_uboot_imx8.sh -b imx93-axon.dtb -d /dev/sdX

	* IMX93_EVK with silicon revision `beta`:
	./install_uboot_imx8.sh -b imx93-11x11-evk.dtb -s A0 -d /dev/sdX

	* EDM-IMX93:
	./install_uboot_imx8.sh -b imx93-edm.dtb -d /dev/sdX

	* PICO-IMX93:
	./install_uboot_imx8.sh -b imx93-pico.dtb -d /dev/sdX

	* AXON-IMX91:
	./install_uboot_imx8.sh -b imx91-axon.dtb -d /dev/sdX

	* EDM-IMX91:
	./install_uboot_imx8.sh -b imx91-edm.dtb -d /dev/sdX

	* PICO-IMX91:
	./install_uboot_imx8.sh -b imx91-pico.dtb -d /dev/sdX

	* EDM-IMX95:
	./install_uboot_imx8.sh -b imx95-edm-evm.dtb -d /dev/sdX
	./install_uboot_imx8.sh -b imx95-edm-evm.dtb -r 16gb -d /dev/sdX

	* EDGE-Ai-IMX95:
	./install_uboot_imx8.sh -b imx95-edge-ai.dtb -d /dev/sdX
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

while getopts "tcfhd:s:b:r:" OPTION
do
	case $OPTION in
		d)
			DRIVE="$OPTARG"
			;;
		b)
			DTBS="$DTBS $OPTARG"
			;;
		s)
			SILICON_REV="$OPTARG"
			;;
		r)
			RAM_SIZE="$OPTARG"
			;;
		t)
			MKIMAGE_TARGET='flash_spl_uboot';
			;;
		f)
			MKIMAGE_TARGET='flash_evk_flexspi';
			;;
		c)
			rm -rf ${FIRMWARE_DIR} ${MKIMAGE_DIR}
			echo "Clean ${FIRMWARE_DIR} ${MKIMAGE_DIR}..."
			exit
			;;
		?|h)
			usage
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
if [ "${SOC_DIR}" = "iMX95" ]; then
	fetch_oei
	fetch_sm
	prepare_arm_toolchain
	generate_oei_image
	generate_sm_image
fi
install_uboot_dtb
generate_imx_boot
flash_imx_boot
