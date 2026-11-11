#!/usr/bin/env bash
#compiles the firmware for the stm32f446 (cortex-m4) with warnings as errors
#uses the st hal already in firmware/cubeide/Drivers, needs arm-none-eabi-gcc
#only shows that it compiles, not that it works on the board
set -euo pipefail
FW="$(cd "$(dirname "$0")/.." && pwd)"
P="$FW/cubeide"
for f in "$FW"/App/Src/*.c "$P"/Core/Src/main.c; do
  arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
    -std=gnu11 -Wall -Wextra -Werror -O2 -DSTM32F446xx -DUSE_HAL_DRIVER \
    -I"$FW/App/Inc" -I"$P/Core/Inc" -I"$P/Drivers/STM32F4xx_HAL_Driver/Inc" \
    -I"$P/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"$P/Drivers/CMSIS/Include" \
    -c "$f" -o /dev/null
  echo "compiled: $(basename "$f")"
done
