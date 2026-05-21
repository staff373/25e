# MaixCAM Pro UART Module

This folder contains MaixCAM Pro-side scripts for the STM32F407 `hal3.0`
project.

## UART Link

- MaixCAM Pro `A19` -> `UART1_TX` -> STM32 `PD6 / USART2_RX`
- MaixCAM Pro `A18` -> `UART1_RX` <- STM32 `PD5 / USART2_TX` when reverse commands are needed
- Common `GND` is required
- Baudrate: `115200`, `8N1`
- MaixPy device: `/dev/ttyS1`

## Smoke Test

Run `stm32_uart_smoke.py` on MaixCAM Pro. It sends one `$V` ASCII frame every
100 ms. On STM32, use Bluetooth command `VISION?` and check:

- `online=1`
- `rx` is increasing
- `ok` is increasing
- `seq` is increasing
- `dx` changes over time

Frame format:

```text
$V,<seq>,<valid>,<x>,<y>,<dx>,<dy>,<area>*<cs>\r\n
```

`cs` is XOR of all bytes between `$` and `*`, excluding both delimiters.
