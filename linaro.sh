#!/bin/bash
make distclean
make clean
rm -rf output
VAR="Linaro"
TREE=$PWD
AK="$TREE/AnyKernel2"
IMAGE=$TREE/arch/arm64/boot/Image.gz-dtb
OUTPUTDIR=$TREE/output
PATH=${PATH}:${LINARO}
export ARCH=arm64
make aio_otfp_m_defconfig ARCH=arm64 CROSS_COMPILE=aarch64-elf-
make ARCH=arm64 CROSS_COMPILE=aarch64-elf-

if ![ -e "$IMAGE" ]; then
echo "$red Error 404: Kernel not compiled."
echo "Fix the compilation errors! $defcol"
exit 1; fi;
cp -i $IMAGE $AK

cd $AK
zip -r flash.zip *
mkdir $OUTPUTDIR
mv flash.zip $OUTPUTDIR
