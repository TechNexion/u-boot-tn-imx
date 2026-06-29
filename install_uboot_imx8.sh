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
script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(cd "$(dirname "$script_path")" && pwd)"

DRIVE=/dev/sdX

#platform related parameters
#PLATFORM="imx8mm"
#SOC_TARGET="iMX8MM"
#SOC_DIR="iMX8M"
#DTBS="fsl-imx8mq-evk"
#DTBS="pico-imx8m"

BRANCH_VER="lf-5.10.72_2.2.0" #branch used by imx-mkimage and imx-atf under meta-imx
ATF_BRANCH_VER="lf_v2.4"
MKIMAGE_SRC_GIT_ID='1112c88470f339dc631e2a7117087f416af6d6b5' #refer to 'imx-mkimage_git.inc' in Yocto
#ATF_SRC_GIT_ID='7a277c8a1a21ff921d217889dde6a9f84e6d2168' #refer to 'imx-atf_2.0.bbappend' in Yocto 此ID 不存在
ATF_SRC_GIT_ID='5f3ed37b85745d360f6a942fa97df04e05779ffe' #
DDR_FW_VER="8.14" #refer to the name of 'firmware-imx-8_8.x.bb'

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
	echo "SOC:${SOC}"
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
		*)
			error_msg "Target SOC isn't supported by this script\n"
			;;
	esac
}

log_msg() {
  echo "${FUNCNAME[1]} $1"
}

info_msg() {
  echo "[INFO][${FUNCNAME[1]}] $1"
}
error_msg() {
  echo "[ERROR][${FUNCNAME[1]}] $1"
  exit -1
}
warning_msg() {
  echo "[WARN][${FUNCNAME[1]}] $1"
}
debug_msg() {
	if [ "$DEBUG_MODE" == "yes" ]; then
	echo "[DBG][${FUNCNAME[1]}] $1"
	fi
}

run_cmd() {
	local label=$1
	local err_exit=$2
	local cmd_str=$3

	info_msg "l:${label}, Running command: $cmd_str"
	eval "$cmd_str"
	local ret=$?
	if [ "$ret" == "0" ]; then
		return 0
	fi

	if [ "$ret" != "0" ]; then
		error_msg "[$label] Command failed: $cmd_str"
	fi

	if [ "$err_exit" == "yes" ]; then
		exit -1
	fi
	return 1
}

copy_file() {
	local src=$1
	local dest=$2
	printf "Copy file $src to $dest ..."
	cp $src $dest
	if [ "$?" != "0" ]; then
		error_msg "Fails to copy ${src} to ${dest}"
		exit -1
	fi
	echo " success"
}

git_clone() {
	local id=$1
	local repo=$2
	local branch=$3
	local hash=$4
	local dir=$5

	if [ -d $dir ]; then
		info_msg "Git $id have exist, skip clone"
		return
	fi

	info_msg "Start clone $id $dir..."
	mkdir -p $dir
	pushd $dir
	run_cmd "git clone $id" "yes" "git clone $repo -b $branch ."
	run_cmd "git checkout $id" "yes" "git checkout -b ${branch}_local ${hash}"
	popd
	info_msg "Finish clone $id finish"
}

install_firmware()
{
	cd ${TWD}
	#Get and Build NXP imx-mkimage tool

	git_clone "imx-mkimage" "https://github.com/nxp-imx/imx-mkimage.git" "${BRANCH_VER}" "${MKIMAGE_SRC_GIT_ID}" "${script_dir}/${MKIMAGE_DIR}"

	cd ${TWD}
	cd ${FIRMWARE_DIR} && FWD=`pwd`
	#Get, build and copy the ARM Trusted Firmware
	git_clone "imx-atf" "https://github.com/nxp-imx/imx-atf.git" "${ATF_BRANCH_VER}" "${ATF_SRC_GIT_ID}" "${FIRMWARE_DIR}/imx-atf"

	PWD=$(pwd)
	cd ${FIRMWARE_DIR}/imx-atf
	# [ -n "${PWD##*imx-atf}" ] && cd imx-atf
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

	run_cmd "Delete bl31.bin" "no" "find -name \"bl31.bin\" -delete"
	run_cmd "Build imx-atf" "yes" "make -j$(nproc) PLAT=${PLATFORM} bl31"
	mkdir -p ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/
	copy_file build/${PLATFORM}/release/bl31.bin "${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/"

	#Get and copy the DDR and HDMI firmware
	cd ${FWD}
	if [ ! -d firmware-imx-${DDR_FW_VER} ] ; then
		info_msg "Download DDR firmware..."
		wget ${FSL_MIRROR}/firmware-imx-${DDR_FW_VER}.bin && \
		chmod +x firmware-imx-${DDR_FW_VER}.bin && \
		./firmware-imx-${DDR_FW_VER}.bin || \
		printf "Fails to fetch DDR firmware \n"
	fi

	local src_base_dir="firmware-imx-${DDR_FW_VER}/firmware"
	local out_dir="${TWD}/${MKIMAGE_DIR}/${SOC_DIR}"
	if [ -d firmware-imx-${DDR_FW_VER}/firmware/ddr/synopsys ] ; then
		if [ ${SOC} = "imx8mp" ] ; then
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_1d_dmem_202006.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_1d_imem_202006.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_2d_dmem_202006.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_2d_imem_202006.bin "${out_dir}/"
		else
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_1d_dmem.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_1d_imem.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_2d_dmem.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_2d_imem.bin "${out_dir}/"
		fi
			cp firmware-imx-${DDR_FW_VER}/firmware/hdmi/cadence/signed_hdmi_imx8m.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}
	else
		printf "Cannot find DDR firmware \n"
		exit -1
	fi
}

install_uboot_dtb()
{
	#Copy uboot binary
	cd ${TWD}
	copy_file u-boot-nodtb.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/

	#Copy SPL binary
	cd ${TWD}
	copy_file spl/u-boot-spl.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/

	#Copy device tree file
	cd ${TWD}
	for DTB in ${DTBS}
	do
		copy_file arch/arm/dts/${DTB} ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/
	done
}


u_boot_build() {
	info_msg "Start build u-boot"
	cd ${script_dir}

	SOC=$( echo "${DTBS}" | cut -d' ' -f1 | grep -o 'imx8m[mqpn]\?' )
	case "${SOC}" in
		imx8mm|imx8mq)
		run_cmd "Build imx-boot ${SOC} config" "yes" "make edm-g-imx8mm_defconfig"
		run_cmd "Build imx-boot ${SOC}"        "yes" "make -j$(nproc)"
		;;
	    *)
		error_msg "Not support build u-boot type:${SOC}"
		;;
	esac

	info_msg "finish build u-boot"
}

generate_imx_boot()
{
	cd ${TWD}
	#Before generating the flash.bin, transfer the mkimage generated by U-Boot to iMX8M folder
	copy_file tools/mkimage "${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/mkimage_uboot"

	#Generate bootable binary (This binary contains SPL and u-boot.bin) for flashing
	cd ${MKIMAGE_DIR}
	run_cmd "Generate ${MKIMAGE_TARGET} image" "yes" "make SOC=${SOC_TARGET} dtbs=\"${DTBS}\" clean"
	run_cmd "Generate ${MKIMAGE_TARGET} image" "yes" "make SOC=${SOC_TARGET} dtbs=\"${DTBS}\" ${MKIMAGE_TARGET} "
	# printf "Make target: ${MKIMAGE_TARGET} and generate flash.bin... \n" || printf "Fails to generate flash.bin... \n"
	info_msg "Generate ${MKIMAGE_TARGET} image finish..."
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
	run_cmd "Flash ${MKIMAGE_TARGET} image" "yes" "sudo dd if=${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/${IMX_BOOT} of=${DRIVE} bs=1k seek=${IMX_BOOT_SEEK} oflag=dsync status=progress"
	sleep 2
	echo "sudo umount /media/$(id -un)/boot"
	sudo umount /media/$(id -un)/boot
	sudo eject ${DRIVE}
	# if [ "$?" != "0" ]; then
	# 	printf "Flash flash.bin... \n" || printf "Fails to flash flash.bin... \n"
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

    * SC-IMX8MP:
    ./install_uboot_imx8.sh -b imx8mp-sc.dtb -d /dev/sdX

    * TEK-IMX8MP:
    ./install_uboot_imx8.sh -b imx8mp-tek.dtb -d /dev/sdX

    * TEK-IMX8MP with flexspi boot (only generate flash.bin):
    ./install_uboot_imx8.sh -b imx8mp-tek.dtb -f -d /dev/null

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
	echo "Make -j$(nproc) target: ${PLATFORM}"
	echo "Make -j$(nproc) target: ${MKIMAGE_TARGET}"
	echo "SOC platform: ${SOC}"
	echo "*************************************************************

	"
}

if [ $# -eq 0 ]; then
	usage
	exit 1
fi

replase_uboot_img() {
	local img_path=$1
	if [ "${DTBS}" == "" ]; then
		error_msg "Need define dtbs data, ${DTBS}"
	fi

	local flash_bin=${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/${IMX_BOOT}
	if [ ! -f "${flash_bin}" ]; then
		error_msg "flash file:${flash_bin} not exist"
	fi

	echo ${img_path} | grep '.xz$'
	if [ "$?" == "0" ]; then
		run_cmd "Extract ${img_path} image" "yes" "xz -d ${img_path}"
		img_path=${img_path%.xz}
	fi

	run_cmd "Replease uboot to ${img_path} image" "yes" "dd if=${flash_bin} of=${img_path} bs=1k seek=${IMX_BOOT_SEEK} oflag=dsync conv=notrunc "
	run_cmd "Create bmap file" "yes" "bmaptool create ${img_path} > ${img_path}.xz.bmap"
	run_cmd "Create XZ file:" "yes" "xz -9 -v -T0 --no-sparse ${img_path}"
}

while getopts "tcfhd:b:iIr:" OPTION
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
        f)
           MKIMAGE_TARGET='flash_evk_flexspi';
           ;;
		c)
		   rm -rf ${FIRMWARE_DIR} ${MKIMAGE_DIR}
		   echo "Clean ${FIRMWARE_DIR} ${MKIMAGE_DIR}..."
		   exit
		   ;;
		i)
		    echo "Flash to $DRIVE"
		    setup_platform
		    flash_imx_boot
		    exit
			;;
		I)
		   export NOT_BUILD=1
		   ;;
		r)
			export REPLASE_IMG="$OPTARG"
			;;
		?|h)
			usage
           exit
           ;;
    esac
done

DTBS=$(echo ${DTBS} | cut -c 1-)

if [ "$(id -u)" == "0" ]; then
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

if [ "$NOT_BUILD" != "1" ]; then
	u_boot_build
	install_firmware
	install_uboot_dtb
	generate_imx_boot
fi

if [ "$DRIVE" != "/dev/sdX" ]; then
	flash_imx_boot
fi

if [ "${REPLASE_IMG}" != "" ]; then
	replase_uboot_img "${REPLASE_IMG}"
fi