#!/bin/bash

set -e -o pipefail

BUILD="cargo build --release --no-default-features --features"
OUT=target/thumbv6m-none-eabi/release

$BUILD stm32g030,panic-halt
arm-none-eabi-objcopy -Obinary $OUT/keybad-fw $OUT/G03xxx_G04xxx.bin
$BUILD stm32c011,panic-halt
arm-none-eabi-objcopy -Obinary $OUT/keybad-fw $OUT/C011xx.bin
cp $OUT/C011xx.bin $OUT/C031xx.bin

cd $OUT
zip firmware.zip C031xx.bin C011xx.bin G03xxx_G04xxx.bin
