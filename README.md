# STM32 Tennis Swing Analyzer

A swing analyzer for a tennis racket, built on an STM32 Nucleo-F446RE. A BMI270 IMU on the racket is read over SPI at 1 kHz using timer-triggered DMA, a state machine on the microcontroller detects each swing and its peak angular velocity, and a Python dashboard shows swing speed and swing count live over UART.

## Hardware

- STM32 Nucleo-F446RE
- SparkFun 6DoF IMU Breakout (BMI270)
- Breadboard and jumper wires

## How It Works

```
TIM2 (1 kHz) ─▶ SPI1 + DMA ─▶ ring buffer ─▶ main loop ─┬─▶ swing state machine
                 (BMI270)                                └─▶ UART + DMA ─▶ Python dashboard
```

- **Sampling:** a 1 kHz timer interrupt starts each SPI DMA read of the accelerometer and gyroscope, so the CPU is free while the data transfers.
- **Swing detection:** a state machine with separate start and end thresholds, debouncing, and a cooldown reports each swing's peak angular velocity.
- **Data link:** samples and swing events are sent to the PC as binary packets with sequence numbers and a CRC, so dropped or corrupted data can be detected.

## Results

| Metric | Value |
|---|---|
| Sampling rate | |
| Timing jitter (peak-to-peak) | |
| Dropped samples | |
| Gyroscope error vs. turntable reference | |
