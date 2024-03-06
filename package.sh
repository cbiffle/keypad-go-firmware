#!/bin/bash

set -e -o pipefail

BUILD="cargo build --release --no-default-features --features"
OUT=target/thumbv6m-none-eabi/release
REL=target/firmware-release

mkdir -p $REL

$BUILD stm32g030,panic-halt
cp $OUT/keybad-fw $REL/G03xxx_G04xxx.elf
arm-none-eabi-objcopy -Obinary $OUT/keybad-fw $REL/G03xxx_G04xxx.bin
$BUILD stm32c011,panic-halt
cp $OUT/keybad-fw $REL/C011xx.elf
arm-none-eabi-objcopy -Obinary $OUT/keybad-fw $REL/C011xx.bin

# Support STM32C031 should I have one in stock and install it -- from the
# firmware's perspective it is a C011.
cp $REL/C011xx.elf $REL/C031xx.elf
cp $REL/C011xx.bin $REL/C031xx.bin

VERSION=$(cargo metadata --format-version 1 \
    | jq -r '.resolve.root as $root | .packages[] | select(.id == $root) | .version')
echo $VERSION > $REL/VERSION

cd $REL
zip firmware.zip *
cd -
mv $REL/firmware.zip keypad-go-firmware-$VERSION.zip

echo "Image sizes:"
ls -l $REL/*.bin
