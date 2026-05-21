# Project Map

## Goal
- STM32F407 HAL 竞赛版底盘工程，目标是现场调车优先的循迹/直角弯/蓝牙调参固件。
- 当前主线是 `app_task` 多题顶层调度、四轮 duty 直控、五路灰度循迹、JY61P yaw 直角弯、UART5 蓝牙 ASCII 命令、USART2 MaixCAM 状态接入、TIM9/TIM12 步进云台定步控制。
- 当前对齐题目：`E题_简易自行瞄准装置.pdf` 基本要求（1），小车自动沿 100cm 正方形黑线循迹，圈数 `N=1~5` 可设定，`t <= 20s`，瞄准模块电源断开；当前固件调试配置固定只接受右转角点。
- 近期准备目标：基本要求（2）前先打通 MaixCAM Pro -> STM32 坐标通信；MaixCAM 输出靶心像素坐标/有效标志/新鲜度，STM32 先完成接收和蓝牙可观测状态，再接瞄准控制。
- 云台步进电机后续目标是精确步数/位置控制，不做单纯速度型控制；速度和加速度只作为一次定步运动的参数。
- 非目标：不做 Flash 参数保存，不把 PC/LLM 自动调参链路放回固件，不在 ISR 里做业务决策。

## Hardware
- MCU: STM32F407xx，CubeMX 工程文件为 `hal3.0.ioc`。
- Motors: 四轮 TB 驱动由 `app_motion` 统一拥有；前轮 PWM 用 TIM2 CH3/CH4，后轮 PWM 用 TIM1 CH3/CH4，方向脚在 `Core/Inc/main.h` 和 `modular3.0/app_motion/app_motion.c` 绑定。
- Encoders: 四路编码器由 `app_motion` 观测；LF TIM3，RF TIM4，LB TIM5，RB TIM8。
- Sensors: 五路灰度输入为 PC0/PC1/PC2/PC3/PB9，由 `bsp_sensor` 和 `app_sensor` 读取。
- UART: UART5 115200 蓝牙，USART3 115200 JY61P，USART2 115200 MaixCAM/视觉。
- DMA/IRQ: UART5 RX 使用 DMA1 Stream0 ReceiveToIdle；USART3 RX 配了 DMA1 Stream1 但当前 JY61P BSP 使用单字节 IT 接收；USART2 使用单字节 IT 接收。
- Stepper/Gimbal: X 轴 `TIM9_CH2/PE6` + `PE15 DIR` + `PB13 EN`，Y 轴 `TIM12_CH1/PB14` + `PC13 DIR` + `PD15 EN`；步进 EN 高电平使能，TIM9/TIM12 Update IRQ 用于按 PWM 周期计步。
- MaixCAM Pro 坐标链路当前接入点是 USART2；MaixCAM 侧使用 A19=`UART1_TX`、A18=`UART1_RX`，MaixPy 设备为 `/dev/ttyS1`；`app_vision` 使用 128 字节行缓冲解析 `$V,<seq>,<valid>,<x>,<y>,<dx>,<dy>,<area>*<cs>` 帧。

## Code Map
- `Core/`、`Drivers/`、`cmake/stm32cubemx/` 是 CubeMX/HAL 骨架和生成入口，尽量只在 `USER CODE` 区维护。
- `CMakeLists.txt` 把 `modular3.0/` 下的用户模块编进目标 `hal3_0`。
- `modular3.0/bsp_*` 是硬件薄封装：电机、编码器、灰度、JY61P、步进 STEP/DIR/EN 定步控制。
- `modular3.0/pid_core` 是平台无关 PID 数学核。
- `modular3.0/app_*` 是比赛应用层：`app_motion`、`app_sensor`、`app_imu`、`app_vision`、`app_gimbal`、`app_aim`、`app_turn`、`app_track`、`app_task`、`app_bt`。
- `modular3.0/maixcam_pro` 是 MaixCAM Pro 侧脚本模块；`stm32_uart_smoke.py` 用 A19/A18 的 UART1 向 STM32 发送 `$V` 测试帧。
- 模块接口速查在 `modular3.0/MODULE_GUIDE.md`；总览、命令和参数说明在 `README_COMPETITION.md`。

## Runtime
- `main.c` 初始化顺序：HAL/clock/GPIO/DMA/TIM/UART 后，依次 `Motion_Init`、`AppSensor_Init`、`Imu_Init`、`Vision_Init`、`Gimbal_Init`、`Aim_Init`、`Turn_Init`、`Track_Init`、`Task_Init`、`BT_Init`。
- 主循环顺序：`Imu_Poll` -> `Vision_Poll` -> `Gimbal_Poll` -> `Aim_Poll` -> `AppSensor_Poll` -> `Turn_Poll` -> `Track_Poll` -> `Task_Poll` -> `BT_Poll` -> `Motion_Poll`。
- 顶层任务链：蓝牙 `TASK 1`/`TASK START` 或兼容 `TRACK START` -> `app_task` 选择 `Q1_TRACK` -> `Track_Start` -> `Task_Poll` 观察完成/停止/错误。
- 自动比赛链：灰度读取 -> `Track_Poll` 状态机 -> PID 修正或 `Turn_Start` -> `Motion_SetDuty4`。
- 直角弯链：第一帧右角点 -> `TURN_DELAY_MS` 延时 -> `Turn_Start(TURN_DIR_RIGHT)` -> JY61P yaw 达到目标角或超时 -> `Turn_Stop` -> 恢复直行；左角点当前被 `app_track` 忽略。
- `Track_Start` 和 `Turn_Start` 都要求 `Imu_IsOnline()` 成立；IMU 异常时蓝牙会返回 `ERR TRACK` 或 `ERR TURN`。
- 蓝牙链：UART5 ReceiveToIdle DMA -> `HAL_UARTEx_RxEventCallback` in `app_bt` -> 行命令队列 -> `BT_Poll` 解析执行。
- UART 普通接收回调由 `app_bt` 托管，分发到 `Imu_HandleRxCplt` 和 `Vision_HandleRxCplt`；错误回调同样由 `app_bt` 分发恢复。
- 步进链：TIM9/TIM12 PWM 输出 STEP，Update IRQ -> `HAL_TIM_PeriodElapsedCallback` -> `BSP_Stepper_TIM_PeriodElapsedCallback` 计步、调速、到步停机；`app_gimbal` 提供二维相对步数和像素误差转步数接口，`app_aim` 消费 `Vision_GetTarget` 做一次瞄准/连续跟踪骨架。

## Integration Rules
- 新比赛行为优先接入 `app_track`、`app_turn`、`app_motion`、`app_sensor` 等应用层模块。
- 多题选择、总启动/总停止、总完成/失败原因归 `app_task`；单题能力仍留在对应 app 模块里。
- 新蓝牙命令接入 `app_bt`，再调用对应 app 模块的公开接口；不要在 HAL 回调里直接解析业务命令。
- MaixCAM 坐标协议解析应归 `app_vision`；后续瞄准/云台闭环应由新的应用层模块消费 `app_vision` 的公开目标数据，不要放进 `app_bt`、HAL 回调或 `main.c`。
- 云台步进 BSP 应提供按步数运动、剩余步数、忙闲、停止/急停等位置型接口；`app_gimbal` 再负责二维云台语义、状态机、蓝牙烟测和后续视觉闭环入口。
- 题目基本要求（1）的圈数 N、已完成圈数、自动停车应归 `app_track`；蓝牙只负责 `SET/GET` 和 `TRACK START/STOP` 入口。
- 当前固定右转策略归 `app_track`，不要在 `app_sensor`、`app_bt` 或 `main.c` 里改角点方向过滤。
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
- 蓝牙串口烟测：`PING`、`STATUS`、`TASK?`、`TASK 1`、`TASK START`、`TASK STOP`、`SENSOR?`、`IMU?`、`VISION?`、`GIMBAL?`、`AIM?`。
- MaixCAM 坐标烟测：MaixCAM 与 STM32 共地，MaixCAM A19/TX 接 STM32 PD6/USART2_RX；若需要 STM32 反向发命令，再接 STM32 PD5/USART2_TX 到 MaixCAM A18/RX。运行 `modular3.0/maixcam_pro/stm32_uart_smoke.py` 后，用蓝牙 `VISION?` 确认 `online=1`、`rx` 增长、`ok` 增长、`seq` 递增、`dx` 变化。
- 步进云台烟测：硬件未确认限位前只用低速小步数；先 `GIMBAL?`，再 `GIMBAL EN 1`，然后 `GIMBAL MOVE X 20 200`、`GIMBAL MOVE Y 20 200`、`GIMBAL ZERO`、`GIMBAL STOP`/`GIMBAL ESTOP`。
- 上板运动烟测：先用 `MOTOR lf rf lb rb` 校验四轮方向，再单独测 `TURN L`/`TURN R`，最后 `TRACK START`。

## Guardrails
- `app_motion` 是电机和编码器所有权边界，避免其他模块直接操作四轮 PWM/方向脚。
- `app_task` 只做多题顶层调度，不直接操作 GPIO/PWM/UART，也不承载具体循迹或瞄准策略。
- `app_bt` 是 UART5 命令入口和 HAL UART 回调分发点，避免重复定义 HAL UART callback。
- `app_turn` 依赖 IMU online 和 yaw，现场调直角弯时先确认 `IMU?` 正常。
- 当前对齐基本要求（1）时不要启动瞄准/云台动作；基本要求（2）（3）和发挥题的瞄准策略应放在 `app_aim`，云台运动语义放在 `app_gimbal`。
- `Track_SetParam` 和 `Turn_SetParam` 的白名单参数只在运行期生效，不保存到 Flash。
- `Drivers/` 和 `Core/` 生成区不要做大改；涉及引脚、定时器、UART 绑定时优先用 CubeMX 或同步更新 `.ioc`。
