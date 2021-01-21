export ARCH=arm64
export SUBARCH=arm64
export ak=$HOME/AnyKernel3
export CROSS_COMPILE=$HOME/aarch64-linux-gnu-gcc/bin/aarch64-linux-gnu-
export CROSS_COMPILE_ARM32=$HOME/arm-none-eabi-gcc/bin/arm-none-eabi-

git submodule init && git submodule update

cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
make clean -j$(nproc --all) && make mrproper -j$(nproc --all)
make floral_defconfig -j$(nproc --all)
make -j$(nproc --all) Image.lz4-dtb

if [ ! -f out/arch/arm64/boot/Image.lz4-dtb ]; then
    exit 1
fi

cp out/arch/arm64/boot/Image.lz4-dtb $ak/Image.lz4-dtb
cd $ak
zip -FSr9 $ak/kernel.zip ./*
