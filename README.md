# Ultrasonic Theremin on STM32

A digital theremin built on an **STM32F401RE** (Nucleo board): an **HC-SR04 ultrasonic sensor** measures how far your hand is, and the microcontroller turns that distance into a tone in real time, generated as a PWM signal.

> Exam project for *Digital Programmable Systems*, B.Sc. in Computer and Automation Engineering (Ingegneria Informatica e dell'Automazione), Politecnico di Bari.
>
> 📄 **Full report:** [Ultrasonic Theremin – A Digital Sound Synthesis Project (PDF)](https://github.com/user-attachments/files/19371935/Ultrasonic_Theremin__A_Digital_Sound_Synthesis_Project.pdf)

---

## How it works

```
 Hand ──► HC-SR04 ──echo──► TIM1 (1 µs tick) ──► distance [cm]
                                                    │
                                          linear mapping 1–100 cm → 300–800 Hz
                                                    │
 Buzzer/Speaker ◄── PA1 PWM ◄── TIM2 CH2 (ARR updated at runtime)
                                                    │
                                     UART2 @115200 ──► "Distance: xx cm, Freq: yy Hz"
```

1. **Distance measurement**: a 10 µs pulse on `TRIG` starts a measurement. **TIM1** runs at 1 MHz and times the width of the `ECHO` pulse, which gives the distance (`d = t · 0.034 / 2` cm). The waits have timeouts, so a missing echo can't freeze the loop.
2. **Distance → pitch**: the distance is limited to **1–100 cm** and mapped linearly onto **300–800 Hz**.
3. **Sound generation**: **TIM2 Channel 2** outputs a PWM signal. Each loop iteration recalculates its auto-reload register (ARR), so the output frequency follows the hand.
4. **Debug output**: distance and frequency are printed on **USART2** (115200 baud) and can be read from any serial monitor.

## Hardware

| Component | Notes |
|---|---|
| STM32 Nucleo-F401RE | ARM Cortex-M4 MCU |
| HC-SR04 | Ultrasonic distance sensor |
| Passive buzzer / small speaker | Driven by the PWM output |

| Signal | MCU pin |
|---|---|
| HC-SR04 `TRIG` | **PA10** (GPIO output) |
| HC-SR04 `ECHO` | **PA4** (GPIO input) |
| Audio out (TIM2_CH2 PWM) | **PA1** |
| UART TX / RX | PA2 / PA3 (ST-Link virtual COM port) |

## Tech stack

- **C** with the **STM32 HAL**
- **STM32CubeIDE / CubeMX** for peripheral configuration and code generation
- Peripherals: GPIO, hardware timers (input timing + PWM), UART

## Project structure

```
Core/
├── Inc/        # Headers (main.h, HAL config, interrupt handlers)
├── Src/
│   ├── main.c  # Application logic: distance reading, mapping, PWM update
│   └── ...     # CubeMX-generated init, MSP and system files
└── Startup/    # Startup assembly for STM32F401RETx
```

## Build & flash

1. Open the project in **STM32CubeIDE** (*File → Import → Existing Projects into Workspace*).
2. Build it and flash it to a Nucleo-F401RE over ST-Link.
3. Wire the sensor and the buzzer as shown above, then open a serial terminal at **115200 baud** to watch the live readings.
