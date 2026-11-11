#!/usr/bin/env bash
#builds the full firmware image for the stm32f446 (cortex-m4) from the files in this repo
#my code (App + Core) is built with warnings as errors, st's hal with normal warnings
#output: firmware/ci/build/swing_analyzer.elf and .bin, needs arm-none-eabi-gcc
set -euo pipefail
FW="$(cd "$(dirname "$0")/.." && pwd)"
P="$FW/cubeide"
OUT="$FW/ci/build"
rm -rf "$OUT" && mkdir -p "$OUT"

CPU="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
CFLAGS="$CPU -std=gnu11 -O2 -ffunction-sections -fdata-sections -DSTM32F446xx -DUSE_HAL_DRIVER
  -I$FW/App/Inc -I$P/Core/Inc -I$P/Drivers/STM32F4xx_HAL_Driver/Inc
  -I$P/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I$P/Drivers/CMSIS/Include"

objs=()
compile() {
  local src="$1" extra="$2"
  local obj="$OUT/$(basename "${src%.*}").o"
  arm-none-eabi-gcc $CFLAGS $extra -c "$src" -o "$obj"
  objs+=("$obj")
}

for f in "$FW"/App/Src/*.c "$P"/Core/Src/*.c; do
  compile "$f" "-Wall -Wextra -Werror"
  echo "compiled: $(basename "$f")"
done
for f in "$P"/Drivers/STM32F4xx_HAL_Driver/Src/*.c; do
  compile "$f" "-Wall"
done
echo "compiled: st hal ($(ls "$P"/Drivers/STM32F4xx_HAL_Driver/Src/*.c | wc -l) files)"
arm-none-eabi-gcc $CPU -x assembler-with-cpp -c "$P/Core/Startup/startup_stm32f446retx.s" -o "$OUT/startup.o"
objs+=("$OUT/startup.o")

arm-none-eabi-gcc $CPU -T"$P/STM32F446RETX_FLASH.ld" --specs=nano.specs --specs=nosys.specs \
  -Wl,--gc-sections -Wl,-Map="$OUT/swing_analyzer.map" -static \
  -o "$OUT/swing_analyzer.elf" "${objs[@]}" -Wl,--start-group -lc -lm -Wl,--end-group
arm-none-eabi-objcopy -O binary "$OUT/swing_analyzer.elf" "$OUT/swing_analyzer.bin"
arm-none-eabi-size "$OUT/swing_analyzer.elf"
