#!/bin/bash
PATH=${PATH}:/media/tanish2k09/Tanish/Tanish/compiling/aarch64-linux-android-6.x/bin
export ARCH=arm64
export KBUILD_BUILD_USER="TESTERS"
export KBUILD_BUILD_HOST="ROCK"
make aio_otfp_m_defconfig ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
make -j4 ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
