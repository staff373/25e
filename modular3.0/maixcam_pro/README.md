# MaixCAM Pro UART Module

This folder contains MaixCAM Pro-side scripts for the STM32F407 `hal3.0`
project.

## MaixVision Usage

- Desktop shortcut: `C:\Users\1\Desktop\MaixVision.lnk`.
- Shortcut target: `E:\maixcam\IDE\MaixVision\MaixVision.exe`.
- MaixVision does not support hot reload in this workflow.
- Do not open and flash a single Python file from the IDE.
- Upload/run this whole folder, `modular3.0/maixcam_pro`, as one MaixCAM Pro project folder after every script change.
- Keep runtime assets that the script needs, including `target_yolo.mud` and the `.cvimodel` referenced inside it, inside this folder unless the script constants are changed.
- For MaixPy API usage, check local official examples and docs under `E:\maixcam\example` first. If local examples are not enough, check official online MaixCAM/MaixPy documentation.

## UART Link

- MaixCAM Pro `A19` -> `UART1_TX` -> STM32 `PD6 / USART2_RX`
- MaixCAM Pro `A18` -> `UART1_RX` <- STM32 `PD5 / USART2_TX` when reverse commands are needed
- Common `GND` is required
- Baudrate: `115200`, `8N1`
- MaixPy device: `/dev/ttyS1`

## Scripts

- `main.py`: app entrypoint; selects the runtime from `app_config.py`.
- `app_config.py`: set `APP_MODE` to `capture` or `yolo` before packaging.
- `capture_uart.py`: one-off UART-triggered image capture tool; saves ordered JPG files under `/root/capture_dataset`.
- `stm32_uart_smoke.py`: sends a synthetic `$V` frame every 100 ms for UART link checks.
- `target_yolo_uart.py`: loads a YOLO `.mud` model, detects the target on a `320x240` camera frame, computes `dx/dy` relative to the configured laser-equivalent aim point, and sends the same `$V` frame to STM32.

## App Packaging

- This folder is a MaixPy app root. `app.yaml` includes both the capture tool and the YOLO runtime assets.
- Default app mode is `capture`, so a packaged app starts the UART-triggered capture tool.
- To package the YOLO UART detector instead, edit `app_config.py` and set `APP_MODE = "yolo"`, then package/install the whole folder again.
- Suggested auto-start app id: `yolo`.

## YOLO Runtime Setup

1. Put the model file in this folder as `target_yolo.mud`, and keep the referenced `.cvimodel` in the same folder.
2. If the model type is not auto-detected correctly, edit `MODEL_TYPE` in `target_yolo_uart.py` to `YOLO11`, `YOLOv8`, or `YOLOv5`.
3. Set `AIM_X` and `AIM_Y` to the laser-equivalent aim point in the `320x240` full-frame coordinate system.
4. Upload the whole `maixcam_pro` folder through MaixVision.
5. Run `target_yolo_uart.py` on MaixCAM Pro.
6. On STM32, use Bluetooth command `VISION?` and check `online=1`, `ok` increasing, `seq` increasing, and `dx/dy` changing with the target.

## Dynamic Debug

- UART sending stays per-frame; do not lower its rate during vision debugging.
- Current lighting/debug threshold is `CONF_THRESHOLD = 0.15`.
- After 5 continuous raw YOLO misses, the script saves limited missed frames to device-side `/root/miss_debug/`.
- Use the saved `/root/miss_debug/*.jpg` images to compare target appearance before and after moving, changing distance, or adding light.

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
