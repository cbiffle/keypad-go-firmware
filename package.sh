#!/bin/bash

set -e -o pipefail

BUILD="cargo build --release --no-default-features --features"
OUT=target/thumbv6m-none-eabi/release

$BUILD stm32g030,panic-halt
cp $OUT/keybad-fw $OUT/G03xxx_G04xxx.elf
arm-none-eabi-objcopy -Obinary $OUT/keybad-fw $OUT/G03xxx_G04xxx.bin
$BUILD stm32c011,panic-halt
cp $OUT/keybad-fw $OUT/C011xx.elf
arm-none-eabi-objcopy -Obinary $OUT/keybad-fw $OUT/C011xx.bin
cp $OUT/C011xx.elf $OUT/C031xx.elf
cp $OUT/C011xx.bin $OUT/C031xx.bin

VERSION=$(cargo metadata --format-version 1 \
    | jq -r '.resolve.root as $root | .packages[] | select(.id == $root) | .version')
echo $VERSION > $OUT/VERSION

cd $OUT
zip firmware.zip VERSION \
    C031xx.bin C011xx.bin G03xxx_G04xxx.bin \
    C031xx.elf C011xx.elf G03xxx_G04xxx.elf
cd -
mv $OUT/firmware.zip keypad-go-firmware-$VERSION.zip
