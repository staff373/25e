# Project Map

## Goal
- STM32F407 HAL 竞赛版底盘工程，目标是现场调车优先的循迹/直角弯/蓝牙调参固件。
- 当前主线是 `app_task` 多题顶层调度、四轮 duty 直控、五路灰度循迹、JY61P yaw 直角弯、UART5 蓝牙 ASCII 命令、USART2 MaixCAM 状态接入。
- 当前对齐题目：`E题_简易自行瞄准装置.pdf` 基本要求（1），小车自动沿 100cm 正方形黑线逆时针循迹，圈数 `N=1~5` 可设定，`t <= 20s`，瞄准模块电源断开。
- 非目标：不做 Flash 参数保存，不把 PC/LLM 自动调参链路放回固件，不在 ISR 里做业务决策。

## Hardware
- MCU: STM32F407xx，CubeMX 工程文件为 `hal3.0.ioc`。
- Motors: 四轮 TB 驱动由 `app_motion` 统一拥有；前轮 PWM 用 TIM2 CH3/CH4，后轮 PWM 用 TIM1 CH3/CH4，方向脚在 `Core/Inc/main.h` 和 `modular3.0/app_motion/app_motion.c` 绑定。
- Encoders: 四路编码器由 `app_motion` 观测；LF TIM3，RF TIM4，LB TIM5，RB TIM8。
- Sensors: 五路灰度输入为 PC0/PC1/PC2/PC3/PB9，由 `bsp_sensor` 和 `app_sensor` 读取。
- UART: UART5 115200 蓝牙，USART3 115200 JY61P，USART2 115200 MaixCAM/视觉。
- DMA/IRQ: UART5 RX 使用 DMA1 Stream0 ReceiveToIdle；USART3 RX 配了 DMA1 Stream1 但当前 JY61P BSP 使用单字节 IT 接收；USART2 使用单字节 IT 接收。
- TIM9/TIM12 和 X/Y 相关引脚仍保留在 CubeMX 生成外设里，但当前 `hal3.0` 没有应用模块拥有瞄准/云台动作。

## Code Map
- `Core/`、`Drivers/`、`cmake/stm32cubemx/` 是 CubeMX/HAL 骨架和生成入口，尽量只在 `USER CODE` 区维护。
- `CMakeLists.txt` 把 `modular3.0/` 下的用户模块编进目标 `hal3_0`。
- `modular3.0/bsp_*` 是硬件薄封装：电机、编码器、灰度、JY61P。
- `modular3.0/pid_core` 是平台无关 PID 数学核。
- `modular3.0/app_*` 是比赛应用层：`app_motion`、`app_sensor`、`app_imu`、`app_vision`、`app_turn`、`app_track`、`app_task`、`app_bt`。
- 模块接口速查在 `modular3.0/MODULE_GUIDE.md`；总览、命令和参数说明在 `README_COMPETITION.md`。

## Runtime
- `main.c` 初始化顺序：HAL/clock/GPIO/DMA/TIM/UART 后，依次 `Motion_Init`、`AppSensor_Init`、`Imu_Init`、`Vision_Init`、`Turn_Init`、`Track_Init`、`Task_Init`、`BT_Init`。
- 主循环顺序：`Imu_Poll` -> `Vision_Poll` -> `AppSensor_Poll` -> `Turn_Poll` -> `Track_Poll` -> `Task_Poll` -> `BT_Poll` -> `Motion_Poll`。
- 顶层任务链：蓝牙 `TASK 1`/`TASK START` 或兼容 `TRACK START` -> `app_task` 选择 `Q1_TRACK` -> `Track_Start` -> `Task_Poll` 观察完成/停止/错误。
- 自动比赛链：灰度读取 -> `Track_Poll` 状态机 -> PID 修正或 `Turn_Start` -> `Motion_SetDuty4`。
- 直角弯链：角点确认 -> `Turn_Start` -> JY61P yaw 达到目标角或超时 -> `Turn_Stop` -> 恢复直行。
- `Track_Start` 和 `Turn_Start` 都要求 `Imu_IsOnline()` 成立；IMU 异常时蓝牙会返回 `ERR TRACK` 或 `ERR TURN`。
- 蓝牙链：UART5 ReceiveToIdle DMA -> `HAL_UARTEx_RxEventCallback` in `app_bt` -> 行命令队列 -> `BT_Poll` 解析执行。
- UART 普通接收回调由 `app_bt` 托管，分发到 `Imu_HandleRxCplt` 和 `Vision_HandleRxCplt`；错误回调同样由 `app_bt` 分发恢复。

## Integration Rules
- 新比赛行为优先接入 `app_track`、`app_turn`、`app_motion`、`app_sensor` 等应用层模块。
- 多题选择、总启动/总停止、总完成/失败原因归 `app_task`；单题能力仍留在对应 app 模块里。
- 新蓝牙命令接入 `app_bt`，再调用对应 app 模块的公开接口；不要在 HAL 回调里直接解析业务命令。
- 题目基本要求（1）的圈数 N、已完成圈数、自动停车应归 `app_track`；蓝牙只负责 `SET/GET` 和 `TRACK START/STOP` 入口。
- 新硬件驱动先放入 `bsp_*`，应用策略放入 `app_*`；不要让 BSP 依赖比赛状态机。
- 保持 `main.c` 只做初始化和固定轮询调度，不把复杂状态机塞进主循环。
- 避免在 `Core/Src/stm32f4xx_it.c`、CubeMX 生成初始化函数、HAL 库源码中加入业务逻辑。

## Verification
- 最小构建：
  ```powershell
  cmake --preset Debug
  cmake --build --preset Debug
  ```
- VSCode 默认构建任务：`STM32: Build hal3 Debug`。
- VSCode 烧录任务：`STM32: Flash hal3 Debug ELF`，目标 ELF 为 `build/Debug/hal3_0.elf`。
- 蓝牙串口烟测：`PING`、`STATUS`、`TASK?`、`TASK 1`、`TASK START`、`TASK STOP`、`SENSOR?`、`IMU?`、`VISION?`。
- 上板运动烟测：先用 `MOTOR lf rf lb rb` 校验四轮方向，再单独测 `TURN L`/`TURN R`，最后 `TRACK START`。

## Guardrails
- `app_motion` 是电机和编码器所有权边界，避免其他模块直接操作四轮 PWM/方向脚。
- `app_task` 只做多题顶层调度，不直接操作 GPIO/PWM/UART，也不承载具体循迹或瞄准策略。
- `app_bt` 是 UART5 命令入口和 HAL UART 回调分发点，避免重复定义 HAL UART callback。
- `app_turn` 依赖 IMU online 和 yaw，现场调直角弯时先确认 `IMU?` 正常。
- 当前对齐基本要求（1）时不要重新启用瞄准/云台动作；若后续做扩展题，先明确新模块所有权再接 TIM9/TIM12。
- `Track_SetParam` 和 `Turn_SetParam` 的白名单参数只在运行期生效，不保存到 Flash。
- `Drivers/` 和 `Core/` 生成区不要做大改；涉及引脚、定时器、UART 绑定时优先用 CubeMX 或同步更新 `.ioc`。
