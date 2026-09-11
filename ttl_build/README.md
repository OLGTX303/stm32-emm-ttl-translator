# STM32F103RC TTL protocol translator

The Keil target `PRJ/STM32_UART_CMD.uvprojx` builds the compatibility service for the unchanged `software` host.

* USART3 is the host port: PB10 TX, PB11 RX, 115200 baud, 8N1.
* USART1 is the shared EMM TTL bus: PA9 TX, PA10 RX, 115,200 baud, 8N1. Connect the four EMM R/A/H inputs together, their T/B/L outputs together, and share ground. Motor power remains separate.
* IDs are 1 right finger, 2 right arm, 3 left finger, 4 left arm.

Host packets, CRC8, little-endian payloads, status replies, queued trapezoids, homing, grouped starts, disable cancellation, and the software's coordinate scale are implemented in `APP/translator.c`. Software coordinates are converted from 16,384 counts/rev to the configured 3,200 EMM pulses/rev using wide absolute arithmetic. Direction and offsets are in `BSP/motor_config.h`; calibrate those four entries for the installed mechanics without editing `software`.

Current percentage maps to 0–2,500 mA. Clamp segments are limited to 40% by configuration. Temperature is returned as -128 when an EMM firmware explicitly rejects the temperature query; this is recorded in diagnostics and is not treated as a successful measured temperature.

Build and host verification:

```powershell
powershell -ExecutionPolicy Bypass -File ttl_build/tests/run_tests.ps1
powershell -ExecutionPolicy Bypass -File ttl_build/build_keil.ps1
python ttl_build/tests/terminal_hex.py "FF FF 08 01 01 01 00 55"
```

The wire response remains binary for software compatibility; `terminal_hex.py` prints both TX and RX bytes as uppercase hexadecimal.

To run the complete supplied sequence through the unchanged motion implementation:

```powershell
python ttl_build/tests/run_sequence.py COM3
```

This performs the software's normal disable, homing/zeroing, motion sequence, and final disable. Keep the mechanism clear, power the EMM bus, and use the exact 115200 baud USART3 connection. The runner uses `cmd_enable`, `cmd_zero`, and `MotionCtrl.motions`.

The resulting image is `build/ttl_translator_keil.hex`. DAPLink programming uses OpenOCD at 100 kHz SWD; the last verified operation is recorded in `build/daplink_flash.log`. `build/bench_readonly.json` contains the COM3 status qualification. `build/bench_motion.json` records the physical smoke-test attempt; the EMM setup transaction timed out and the firmware returned error 6, so it did not acknowledge motion. Reset or power-cycle the motor bus before another physical motion run.

`build/software_sha256.json` records the unchanged software tree used as the protocol authority. Simulation metrics are in `build/simulation_metrics.json`; they are scheduler measurements, not bench measurements.
