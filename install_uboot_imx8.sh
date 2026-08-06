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

BRANCH_VER="lf-6.18.20_2.0.0" #branch used by imx-mkimage and imx-atf under meta-imx
ATF_BRANCH_VER="lf_v2.14"
MKIMAGE_SRC_GIT_ID='1b577853ae1afe1f26cdef27548da52fb424af48' #refer to 'imx-mkimage_git.inc' in Yocto
ATF_SRC_GIT_ID='0779f89a5475a03193f7707f3bbb50cec11707c0' #refer to 'imx-atf_2.x.bb' in Yocto
DDR_FW_VER="8.32-1991416" #refer to the name of 'firmware-imx-8.x.bb'
ELE_FW_VER="2.0.6-c0b284c" ##refer to the "{PV of firmware-ele-imx_2.0.2.bb}"-"{IMX_SRCREV_ABBREV}"

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
IMX_SM_BRANCH_VER="lf-6.18.20-2.0.0"
IMX_SM_CONFIG="mx95evk"
IMX_OEI_GIT_REPO="https://github.com/TechNexion/imx-oei.git"
# imx-oei need to change local server"
IMX_OEI_BRANCH_VER="tn-imx_6.18.20_2.0.0"
IMX_OEI_CONFIG="edm-imx95"
ARM_TOOLCHAIN_VER_DEFAULT="15.2.rel1"

DDR_TYPE=lpddr5_multi

setup_platform() {
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
			;;
		*)
			printf "Target SOC isn't supported by this script\n"
			exit 1
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

	if [ "$ret" == "0" ]; then
		return 0
	fi

	if [ "$err_exit" == "yes" ]; then
		error_msg "[$label] Command failed: $cmd_str"
	else
		warning_msg "[$label] Command failed: $cmd_str"
	fi
}

copy_file() {
	local src=$1
	local dest=$2
	info_msg "Copy file $src to $dest ..."
	cp $src $dest
	if [ "$?" != "0" ]; then
		error_msg "Fails to copy ${src} to ${dest}"
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

is_ddr4() {
	local ddr_type=${DDR_TYPE}
	case $ddr_type in
		lpddr4x_multi | lpddr4x_2g | lpddr4x_4g | lpddr4x_8g) echo "1";;
		*) echo "0" ;;
	esac
}

get_ddr_name() {
	local _is_ddr_4=$(is_ddr4)
	local ddr_name="lpddr5"
	if [ "$_is_ddr_4" == "1" ]; then
		ddr_name="lpddr4x"
	fi
	echo "${ddr_name}"
}

install_firmware() {
	cd ${script_dir}
	#Get and Build NXP imx-mkimage tool
	git_clone "imx-mkimage" "https://github.com/nxp-imx/imx-mkimage.git" "${BRANCH_VER}" "${MKIMAGE_SRC_GIT_ID}" "${script_dir}/${MKIMAGE_DIR}"
	pushd ${MKIMAGE_DIR}/
	sed -i 's|dtb == evk.dtb|dtb == $(dtbs)|g' iMX8M/soc.mak
	popd

	cd ${script_dir}

	#Get, build and copy the ARM Trusted Firmware
	git_clone "imx-atf" "https://github.com/nxp-imx/imx-atf.git" "${ATF_BRANCH_VER}" "${ATF_SRC_GIT_ID}" "${FIRMWARE_DIR}/imx-atf"

	cd ${FIRMWARE_DIR}/imx-atf

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

	run_cmd "Delete bl31.bin" "no" "find -name bl31.bin -delete"
	run_cmd "Build imx-atf" "yes" "make -j$(nproc) PLAT=${PLATFORM} IMX_BOOT_UART_BASE=${ATF_BOOT_UART_BASE} bl31"

	mkdir -p ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/
	copy_file build/${PLATFORM}/release/bl31.bin "${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/"

	#Fetch and copy the DDR and HDMI firmware
	cd ${script_dir}/${FIRMWARE_DIR}
	if [ ! -d firmware-imx-${DDR_FW_VER} ] ; then
		info_msg "Download DDR firmware..."
		wget ${FSL_MIRROR}/firmware-imx-${DDR_FW_VER}.bin && \
		chmod +x firmware-imx-${DDR_FW_VER}.bin && \
		./firmware-imx-${DDR_FW_VER}.bin || \
		printf "Fails to fetch DDR firmware \n"
	fi

	local src_base_dir="firmware-imx-${DDR_FW_VER}/firmware"
	local out_dir="${script_dir}/${MKIMAGE_DIR}/${SOC_DIR}"
	if [ -d ${src_base_dir} ] ; then
		case ${SOC} in
			imx8mp)
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_1d_dmem_202006.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_1d_imem_202006.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_2d_dmem_202006.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_2d_imem_202006.bin "${out_dir}/"
				copy_file ${src_base_dir}/hdmi/cadence/signed_hdmi_imx8m.bin "${out_dir}/"
				;;
			imx93|imx91)
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_imem_1d_v202201.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_dmem_1d_v202201.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_imem_2d_v202201.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_dmem_2d_v202201.bin "${out_dir}/"
				;;
			imx95)
				local ddr_name=$(get_ddr_name)
				copy_file ${src_base_dir}/ddr/synopsys/${ddr_name}_dmem_qb_v202409.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/${ddr_name}_dmem_v202409.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/${ddr_name}_imem_qb_v202409.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/${ddr_name}_imem_v202409.bin "${out_dir}/"
				;;
			*)
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_1d_dmem.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_1d_imem.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_2d_dmem.bin "${out_dir}/"
				copy_file ${src_base_dir}/ddr/synopsys/lpddr4_pmu_train_2d_imem.bin "${out_dir}/"
				;;
		esac
	else
		error_msg "Cannot find firmware"
	fi

	#Fetch and copy EdgeLock Secure Enclave firmware
	if [ "${SOC_DIR}" == "iMX93" ] || [ "${SOC_DIR}" == "iMX91" ] || [ "${SOC_DIR}" == "iMX95" ]; then
		SOC_LOWER=$(echo $SOC_DIR | sed 's/^i//' | tr '[:upper:]' '[:lower:]')
		REV_LOWER=$(echo "${SILICON_REV}" | tr '[:upper:]' '[:lower:]')
		AHAB_IMG="${SOC_LOWER}${REV_LOWER}-ahab-container.img"

		if [ "${SOC_DIR}" == "iMX93" ] && [ "${SILICON_REV}" == "A0" ]; then
			if [ ! -d firmware-sentinel-0.11 ] ; then
				wget ${FSL_MIRROR}/firmware-sentinel-0.11.bin
				chmod +x firmware-sentinel-0.11.bin
				./firmware-sentinel-0.11.bin
			fi
			copy_file firmware-sentinel-0.11/mx93a0-ahab-container.img ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/
		else
			if [ ! -d firmware-ele-imx-${ELE_FW_VER} ] ; then
				wget ${FSL_MIRROR}/firmware-ele-imx-${ELE_FW_VER}.bin
				chmod +x firmware-ele-imx-${ELE_FW_VER}.bin
				./firmware-ele-imx-${ELE_FW_VER}.bin
			fi
			copy_file firmware-ele-imx-${ELE_FW_VER}/${AHAB_IMG} ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/
		fi
	fi

	info_msg "Install firmware finish"
}

install_uboot_dtb() {
	#Copy uboot binary
	cd ${TWD}
	if [ "${SOC_DIR}" == "iMX93" ] || [ "${SOC_DIR}" == "iMX91" ] || [ "${SOC_DIR}" == "iMX95" ] ; then
		copy_file ./out/u-boot.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/
	else
		copy_file ./out/u-boot-nodtb.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/
	fi

	#Copy SPL binary
	cd ${TWD}
	copy_file ./out/spl/u-boot-spl.bin ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/

	#Copy device tree file
	cd ${TWD}
	for DTB in ${DTBS}
	do
		copy_file ./out/arch/arm/dts/${DTB} ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/
	done
}

fetch_oei() {
	git_clone "imx-oei" "${IMX_OEI_GIT_REPO}" "${IMX_OEI_BRANCH_VER}" "origin/${IMX_OEI_BRANCH_VER}" "${script_dir}/${FIRMWARE_DIR}/imx-oei"
}

fetch_sm() {
	git_clone "imx-sm" "${IMX_SM_GIT_REPO}" "${IMX_SM_BRANCH_VER}" "${IMX_SM_BRANCH_VER}" "${script_dir}/${FIRMWARE_DIR}/imx-sm"
}

prepare_arm_toolchain() {
	cd ${script_dir}/${FIRMWARE_DIR} && FWD=`pwd`
	if [ -d arm-gnu-toolchain-*-x86_64-arm-none-eabi ] ; then
		return
	fi

	if [ -f imx-sm/sm/makefiles/common.mak ] ; then
		ARM_TOOLCHAIN_VER=$(grep "TC_VERSION ?=" imx-sm/sm/makefiles/common.mak | cut -d'=' -f2 | xargs)
	fi
	if [ -z "${ARM_TOOLCHAIN_VER}" ] ; then
		ARM_TOOLCHAIN_VER=${ARM_TOOLCHAIN_VER_DEFAULT}
	fi
	wget "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VER}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VER}-x86_64-arm-none-eabi.tar.xz"
	tar xvf arm-gnu-toolchain-${ARM_TOOLCHAIN_VER}-x86_64-arm-none-eabi.tar.xz
	info_msg "Fails to fetch ARM toolchain"
}

generate_sm_image() {
	cd ${script_dir}/${FIRMWARE_DIR} && FWD=`pwd`

	if [ -d arm-gnu-toolchain-*-x86_64-arm-none-eabi ] ; then
		export TOOLS=${FWD}
	else
		printf "Cannot find ARM toolchain \n"
	fi
	pushd ${script_dir}/${FIRMWARE_DIR}/imx-sm
	run_cmd "Build imx-sm" "yes" "make -j$(nproc) config=${IMX_SM_CONFIG}"
	# run_cmd "M33_IMAGE elf to bin" "yes" "arm-none-eabi-objcopy -O binary ./build/mx95evk/m33_image.elf ./m33_image.bin"
	copy_file "build/mx95evk/m33_image.bin" "${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/"
	popd
}

ddr_type_to_ddr_config() {
  local ddr_type=$1
  local ddr_conf=""

  case $ddr_type in
	lpddr4x_multi)   ddr_conf="lpddr4x_multi"                             ;;
	lpddr4x_2g)      ddr_conf="lpddr4x_4266mbps_train_timing_2gb"         ;;
	lpddr4x_4g)      ddr_conf="lpddr4x_4266mbps_train_timing_4gb"         ;;
	lpddr4x_8g)      ddr_conf="lpddr4x_4266mbps_train_timing_8gb"         ;;
	lpddr5_multi)    ddr_conf="lpddr5_multi"                              ;;
	lpddr5_4g)       ddr_conf="lpddr5_6400mbps_train_timing_4gb"          ;;
	lpddr4_8g)       ddr_conf="lpddr5_6400mbps_train_timing_8gb"          ;;
	lpddr5_16g)      ddr_conf="lpddr5_6400mbps_train_timing_16gb"         ;;
	lpddr5_evl_a1)   ddr_conf="XIMX95LPD5EVK19_6400mbps_train_timing_a1"  ;;
	*)               ddr_conf="lpddr5_multi" ;;
  esac
  echo "${ddr_conf}"
}

generate_oei_image() {
	cd ${script_dir}/${FIRMWARE_DIR} && FWD=`pwd`

	if [ -d arm-gnu-toolchain-*-x86_64-arm-none-eabi ] ; then
		export TOOLS=${FWD}
	else
		printf "Cannot find ARM toolchain \n"
	fi

	cd imx-oei
	local ddr_conf=$(ddr_type_to_ddr_config "${DDR_TYPE}")
	if [ "${ddr_conf}" == "" ]; then
		error_msg "Not support DDR type:${DDR_TYPE}"
	fi
	run_cmd "Build imx-oei" "yes" "make -j$(nproc)  board=${IMX_OEI_CONFIG} oei=ddr r=${SILICON_REV} DDR_CONFIG=${ddr_conf} DEBUG=1"
	copy_file "build/${IMX_OEI_CONFIG}/ddr/oei-m33-ddr.bin" "${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/"
}

get_uboot_defconf_name() {
	local conf_name=""
	case "${DTBS}" in
		imx8mm-pico-pi.dtb | imx8mm-pico-wizard.dtb)     conf_name="pico-imx8mm_defconfig"  ;;
		imx8mm-edm-g.dtb   | imx8mq-edm-wizard.dtb )     conf_name="edm-g-imx8mm_defconfig" ;;
		imx8mq-pico-pi.dtb | imx8mq-pico-wizard.dtb)     conf_name="edm-imx8mq_defconfig"   ;;
		imx8mp-axon.dtb)                                 conf_name="axon-imx8mp_defconfig"  ;;
		imx8mp-edm-g.dtb)                                conf_name="edm-g-imx8mp_defconfig" ;;
		imx8mp-tek.dtb)                                  conf_name="tek-imx8mp_defconfig"   ;;
		imx8mp-tep.dtb)                                  conf_name="tep-imx8mp_defconfig"   ;;
		imx8mn-edm-g.dtb)                                conf_name="edm-g-imx8mn_defconfig" ;;
		imx93-axon.dtb)                                  conf_name="axon-imx93_defconfig"   ;;
		imx93-11x11-evk.dtb)                             conf_name="imx93_11x11_evk_defconfig" ;;
		imx93-edm.dtb)                                   conf_name="edm-imx93_defconfig"    ;;
		imx93-pico.dtb)                                  conf_name="pico-imx93_defconfig"   ;;
		imx91-axon.dtb)                                  conf_name="axon-imx91_defconfig"   ;;
		imx91-edm.dtb)                                   conf_name="edm-imx91_defconfig"    ;;
		imx91-pico.dtb)                                  conf_name="pico-imx91_defconfig"   ;;
		imx95-edm-evm.dtb)                               conf_name="edm-imx95_defconfig"    ;;
		imx95-edge-ai.dtb)                               conf_name="edge-ai-imx95_defconfig";;
		imx8mp-sc.dtb)                                   conf_name="NOT DEFINE kernel config file name" ;;
	esac

	echo "${conf_name}"
}

u_boot_build() {
	info_msg "Start build u-boot"
	cd ${script_dir}
	mkdir -p out
	local kernel_conf_name=$(get_uboot_defconf_name )
	echo "Kernel config name:${kernel_conf_name}"
	run_cmd "Build imx-boot iMX95 config" "yes" "make O=./out ${kernel_conf_name}"
	run_cmd "Build imx-boot iMX95"        "yes" "make O=./out -j$(nproc)"
	run_cmd "Build imx-boot iMX95"        "no"  "make O=./out savedefconfig"
	info_msg "finish build u-boot"
}

generate_imx_boot() {
	cd ${TWD}
	#Before generating the flash.bin, transfer the mkimage generated by U-Boot to iMX8M folder
	copy_file ./out/tools/mkimage ${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/mkimage_uboot

	pushd ${MKIMAGE_DIR}
	if [ "${SOC_DIR}" == "iMX93" ] && [ "${SILICON_REV}" == "A0" ]; then
		run_cmd "Generate ${MKIMAGE_TARGET} image"       "yes" "make -j$(nproc) SOC=${SOC_TARGET} REV=${SILICON_REV} dtbs=\"${DTBS}\" ${MKIMAGE_TARGET}"
	elif [ "${SOC_DIR}" == "iMX95" ]; then
    local is_ddr4_val=$(is_ddr4)
		local ddr_name="lpddr5"
		if [ "${is_ddr4_val}" == "1" ]; then
			ddr_name="lpddr4x"
		fi

		run_cmd "Generate iMX95:${MKIMAGE_TARGET} image" "yes" "make -j$(nproc) SOC=${SOC_TARGET} REV=${SILICON_REV} OEI=\"YES\" LPDDR_TYPE=${ddr_name} dtbs=${DTBS} ${MKIMAGE_TARGET}"
	else
		run_cmd "Generate ${MKIMAGE_TARGET} image"       "yes" "make -j$(nproc) SOC=${SOC_TARGET} dtbs=\"${DTBS}\" ${MKIMAGE_TARGET}"
	fi
	popd

	info_msg "Build UBOOT finish"
}

flash_imx_boot() {
	cd ${TWD}
	if [ ! -b $DRIVE ]; then
     echo "$DRIVE doesn't exist !!!"
     exit
	fi
	sudo umount ${DRIVE}
	sleep 0.1
	run_cmd "Flash ${MKIMAGE_TARGET} imsge" "yes" "sudo dd if=${TWD}/${MKIMAGE_DIR}/${SOC_DIR}/${IMX_BOOT} of=${DRIVE} bs=1k seek=${IMX_BOOT_SEEK} oflag=dsync status=progress"
	sleep 2
	echo "sudo umount /media/$(id -un)/boot"
	sudo umount /media/$(id -un)/boot
	sudo eject ${DRIVE}
}

usage() {
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

	* EDM-G-IMX8MM with WB/WIZARD:
	./install_uboot_imx8.sh -b imx8mm-edm-g.dtb -d /dev/sdX

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
	./install_uboot_imx8.sh -b imx95-edm-evm.dtb -m lpddr4x_4g -d /dev/sdX
	- Memory type options:
	- lpddr4x_multi, lpddr4x_2g, lpddr4x_4g, lpddr4x_8g
	- lpddr5_multi, lpddr5_4g, lpddr4_8g, lpddr5_16g
	- lpddr5_evl_a1
	※ lpddr4x_multi,lpddr4x_2g, lpddr4x_8g => not ready


	* EDGE-Ai-IMX95:
	./install_uboot_imx8.sh -b imx95-edge-ai.dtb -d /dev/sdX
"
}

print_settings() {
	echo "*************************************************************"
	echo "Before run this script, please build u-boot first!
	"
	echo "The disk path to flash u-boot: $DRIVE"
	echo "The default DTB name: ${DTBS}"
	echo "Make -j$(nproc) target: ${PLATFORM}"
	echo "Make -j$(nproc) target: ${MKIMAGE_TARGET}"
	echo "SOC platform: ${SOC}"
	if [ "${PLATFORM}" == "imx95" ]; then
		local ddr_conf=$(ddr_type_to_ddr_config ${DDR_TYPE})
		echo "DDR TYPE: ${DDR_TYPE}, DDR CONFIG:${ddr_conf}"
	fi
	echo "*************************************************************

	"
}

if [ $# -eq 0 ]; then
	usage
	exit 1
fi

while getopts "tcfhd:s:b:r:iICm:" OPTION
do
	case $OPTION in
		d)  DRIVE="$OPTARG"
			echo "Set disk path: $DRIVE"
			;;
		b)  DTBS="$DTBS $OPTARG"
			DTBS=$(echo ${DTBS} | cut -c 1-)
			;;
		s) SILICON_REV="$OPTARG" ;;
		t) MKIMAGE_TARGET='flash_spl_uboot' ;;
		m) DDR_TYPE="$OPTARG" ;;
		f) MKIMAGE_TARGET='flash_evk_flexspi' ;;
		C)  # Clean build object file.
		    # rm -rf ./imx-boot_generation/firmware-*/
			find imx-boot_generation/imx-atf/ -name "*.o" -delete
			rm -rf  imx-boot_generation/imx-atf/build/
			find imx-boot_generation/imx-oei/ -name "*.o" -delete
			rm -rf  imx-boot_generation/imx-oei/build/
			find imx-boot_generation/imx-sm/ -name "*.o" -delete
			rm -rf  imx-boot_generation/imx-sm/build/
			pushd ./imx-mkimage/iMX95
				git clean -ffxd .
			popd

			echo "Clean ${FIRMWARE_DIR} ${MKIMAGE_DIR}..."
			exit
			;;
		c)  rm -rf ${FIRMWARE_DIR} ${MKIMAGE_DIR} ./out
			echo "Clean ${FIRMWARE_DIR} ${MKIMAGE_DIR}..."
			;;
		i) 	echo "Flash to $DRIVE"
		    setup_platform
		    flash_imx_boot
		    exit
			;;
		I) export NOT_BUILD=1 ;;
		?|h) usage
			 exit
			 ;;
		esac
done

if [ "$(id -u)" == "0" ]; then
   info_msg "This script can not be run as root"
   exit 1
fi

setup_platform
print_settings
install_firmware

if [ "$NOT_BUILD" != "1" ]; then
	if [ "${SOC_DIR}" == "iMX95" ]; then
		fetch_oei
		fetch_sm
		prepare_arm_toolchain
		generate_oei_image
		generate_sm_image
		u_boot_build
fi
	install_uboot_dtb
	generate_imx_boot
fi

if [ "$DRIVE" != "/dev/sdX" ]; then
	flash_imx_boot
fi
