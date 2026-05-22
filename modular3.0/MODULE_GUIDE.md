# modular3.0 模块调用说明

## 1. 主体调用关系

### 初始化顺序

`main.c` 当前初始化顺序：

1. `Motion_Init()`
2. `AppSensor_Init()`
3. `Imu_Init()`
4. `Vision_Init()`
5. `Gimbal_Init()`
6. `Aim_Init()`
7. `Turn_Init()`
8. `Track_Init()`
9. `Task_Init()`
10. `BT_Init()`

### 运行时主线

1. `Imu_Poll()`：刷新 JY61P 解析缓存。
2. `Vision_Poll()`：维持 USART2 接收开关，并解析 MaixCAM `$V` 坐标帧。
3. `Gimbal_Poll()`：刷新二维云台忙闲状态。
4. `Aim_Poll()`：执行一次瞄准或连续跟踪骨架。
5. `AppSensor_Poll()`：刷新五路灰度状态。
6. `Turn_Poll()`：若正在直角转弯，持续输出四轮 duty。
7. `Track_Poll()`：执行第一问循迹跑圈状态机；转弯阶段不再做循迹，右角点首帧后只等待延时。
8. `Task_Poll()`：观察当前题目的完成、停止和错误状态。
9. `BT_Poll()`：处理蓝牙命令。
10. `Motion_Poll()`：更新编码器计数。

## 2. 模块基础功能

### `bsp_dcmotor`

- 功能：单个直流电机的 PWM 占空比和方向脚输出。
- 关键接口：
  - `BSP_DCMotor_Init`
  - `BSP_DCMotor_SetDuty`
  - `BSP_DCMotor_Stop`
- 调用者：`app_motion`

### `bsp_encoder`

- 功能：单个编码器的增量计数、累计计数、角速度采样。
- 关键接口：
  - `BSP_Encoder_Init`
  - `BSP_Encoder_Update`
  - `BSP_Encoder_GetAccumCount`
  - `BSP_Encoder_GetDeltaCount`
- 调用者：`app_motion`

### `bsp_sensor`

- 功能：读取五路灰度，计算压缩状态 `0x00 ~ 0x1F`，并给出归一化误差和角点类型。
- 关键接口：
  - `Sensor_InitWithConfig`
  - `Sensor_Read`
  - `Sensor_GetRawState`
  - `Sensor_GetNormError`
  - `Sensor_GetStateType`
- 调用者：`app_sensor`

### `bsp_jy61p`

- 功能：JY61P 单字节 UART 接收、姿态帧解析、在线状态维护。
- 关键接口：
  - `BSP_JY61P_Init`
  - `BSP_JY61P_Update`
  - `BSP_JY61P_HandleRxCplt`
  - `BSP_JY61P_HandleUartError`
- 调用者：`app_imu`

### `bsp_stepper`

- 功能：双轴 STEP/DIR/EN 步进驱动，按 PWM 周期精确计步，支持相对步数、剩余步数、保持使能、停止和急停。
- 当前绑定：
  - X：`TIM9_CH2/PE6`，`PE15 DIR`，`PB13 EN`
  - Y：`TIM12_CH1/PB14`，`PC13 DIR`，`PD15 EN`
- 关键接口：
  - `BSP_Stepper_MoveSteps`
  - `BSP_Stepper_Stop`
  - `BSP_Stepper_EmergencyStop`
  - `BSP_Stepper_SetHoldEnabled`
  - `BSP_Stepper_GetPosition`
  - `BSP_Stepper_GetRemaining`
- 调用者：`app_gimbal`

### `pid_core`

- 功能：只保留简单 PID 数学计算，不接 HAL、不接串口、不接遥测。
- 关键接口：
  - `PID_Core_Init`
  - `PID_Core_Reset`
  - `PID_Core_Calculate`
- 调用者：`app_track`

## 3. 应用层模块

### `app_motion`

- 功能：底盘四轮 duty 直控；编码器只做观测，不做速度闭环。
- 对外接口：
  - `Motion_Init`
  - `Motion_Poll`
  - `Motion_SetDuty4(lf, rf, lb, rb)`
  - `Motion_Stop`
  - `Motion_GetDuty`
  - `Motion_GetEncoderAccum`
  - `Motion_GetEncoderDelta`
- 主要调用者：
  - `app_track`：循迹输出四轮 duty
  - `app_turn`：直角弯输出四轮 duty
  - `app_bt`：手动 `MOTOR` 命令直控

### `app_sensor`

- 功能：把 `bsp_sensor` 包装成比赛用接口，直接输出 raw 状态、归一化误差和左右角点方向。
- 对外接口：
  - `AppSensor_Init`
  - `AppSensor_Poll`
  - `AppSensor_ReadNow`
  - `AppSensor_GetRawState`
  - `AppSensor_GetNormError`
  - `AppSensor_GetStateType`
  - `AppSensor_GetCornerDirection`
  - `AppSensor_FormatStatus`
- 主要调用者：
  - `app_track`
  - `app_bt`

### `app_imu`

- 功能：维护 JY61P 当前 yaw、gyro_z、online。
- 对外接口：
  - `Imu_Init`
  - `Imu_Poll`
  - `Imu_HandleRxCplt`
  - `Imu_HandleUartError`
  - `Imu_IsOnline`
  - `Imu_GetYawDeg`
  - `Imu_GetGyroZDps`
  - `Imu_FormatStatus`
- 主要调用者：
  - `app_turn`
  - `app_track`
  - `app_bt`

### `app_vision`

- 功能：保留 USART2 MaixCAM 接收开关，使用 128 字节行缓冲接收并解析 `$V,<seq>,<valid>,<x>,<y>,<dx>,<dy>,<area>*<cs>` 坐标帧，维护在线、新鲜度、有效目标和坏帧计数。
- 对外接口：
  - `Vision_Init`
  - `Vision_Poll`
  - `Vision_HandleRxCplt`
  - `Vision_HandleUartError`
  - `Vision_SetEnabled`
  - `Vision_IsEnabled`
  - `Vision_IsOnline`
  - `Vision_GetTarget`
  - `Vision_GetState`
  - `Vision_GetStateName`
  - `Vision_FormatStatus`
- 主要调用者：
  - `app_bt`
  - `app_aim`

### `app_gimbal`

- 功能：二维云台应用层，封装 X/Y 相对步数移动、保持使能、软件零点、像素误差到步数的 2x2 标定矩阵。
- 对外接口：
  - `Gimbal_Init`
  - `Gimbal_Poll`
  - `Gimbal_MoveRelativeSteps`
  - `Gimbal_MoveByPixelError`
  - `Gimbal_Zero`
  - `Gimbal_Stop`
  - `Gimbal_EStop`
  - `Gimbal_SetCalibration`
  - `Gimbal_FormatStatus`
- 主要调用者：
  - `app_aim`
  - `app_bt`

### `app_aim`

- 功能：瞄准策略骨架，消费 `Vision_GetTarget`，通过 `app_gimbal` 执行一次瞄准或连续跟踪；画圆等发挥题策略后续继续放在这里。
- 对外接口：
  - `Aim_Init`
  - `Aim_Poll`
  - `Aim_StartOnce`
  - `Aim_StartTrack`
  - `Aim_Stop`
  - `Aim_FormatStatus`
- 主要调用者：
  - `app_bt`

### `maixcam_pro`

- 功能：MaixCAM Pro 侧脚本模块，不参与 STM32 固件构建。
- 文件：
  - `README.md`：A19/A18 接线、协议和烟测方法。
  - `stm32_uart_smoke.py`：参考 MaixPy 官方 UART 例程，使用 `/dev/ttyS1` 每 100ms 发送 `$V` 测试帧。
- 烟测入口：MaixCAM 运行脚本后，STM32 蓝牙发送 `VISION?` 查看 `online`、`rx`、`ok`、`seq`、`dx`。

### `app_turn`

- 功能：执行固定目标角度的 90 度直角弯，外侧前进、内侧反转，靠 JY61P yaw 停止。
- 对外接口：
  - `Turn_Init`
  - `Turn_Poll`
  - `Turn_Start(direction)`
  - `Turn_Stop`
  - `Turn_IsActive`
  - `Turn_WasLastTimeout`
  - `Turn_SetParam`
  - `Turn_GetParam`
  - `Turn_FormatStatus`
- 主要调用者：
  - `app_track`
  - `app_bt`

### `app_track`

- 功能：第一问循迹跑圈状态机，负责循迹、直线段 JY61P yaw 弱航向保持、固定右角点首帧锁存、延时右转切换、恢复直行。
- 对外接口：
  - `Track_Init`
  - `Track_Poll`
  - `Track_Start`
  - `Track_Stop`
  - `Track_GetState`
  - `Track_GetStateName`
  - `Track_GetStopReasonName`
  - `Track_IsRunning`
  - `Track_SetTargetLaps`
  - `Track_GetTargetLaps`
  - `Track_GetLapsDone`
  - `Track_GetCornerCount`
  - `Track_GetElapsedMs`
  - `Track_SetParam`
  - `Track_GetParam`
  - `Track_FormatStatus`
- 状态说明：
  - `IDLE`：初始化后未开跑
  - `LINE_FOLLOW`：正常循迹，灰度 PID 为主，JY61P yaw 航向保持为弱补偿
  - `TURN_DELAY`：右角点首帧后的转弯延时中
  - `TURNING`：直角弯执行中
  - `RECOVER_LINE`：转弯后按基础 duty 直行恢复
  - `FINISHED`：目标圈数完成后自动停车
  - `STOPPED`：命令停止或保护停机

### `app_task`

- 功能：多题顶层调度，当前支持 `Q1_TRACK`，负责选择题目、启动当前题目、停止、复位和汇总完成/失败原因。
- 对外接口：
  - `Task_Init`
  - `Task_Poll`
  - `Task_Select`
  - `Task_SelectQuestion`
  - `Task_Start`
  - `Task_Stop`
  - `Task_Reset`
  - `Task_GetMode`
  - `Task_GetModeName`
  - `Task_GetState`
  - `Task_GetStateName`
  - `Task_GetStopReason`
  - `Task_GetStopReasonName`
  - `Task_GetElapsedMs`
  - `Task_FormatStatus`
- 状态说明：
  - `IDLE`：没有选择题目
  - `SELECTED`：已选择题目，等待启动
  - `RUNNING`：当前题目运行中
  - `FINISHED`：当前题目正常完成
  - `STOPPED`：用户停止
  - `ERROR`：启动失败、子任务异常停止或不支持的题目

### `app_bt`

- 功能：UART5 蓝牙 ASCII 行命令入口，同时托管 UART 回调分发。
- 对外接口：
  - `BT_Init`
  - `BT_Poll`
- 内部负责：
  - `HAL_UARTEx_RxEventCallback`：处理 UART5 DMA 空闲接收
  - `HAL_UART_RxCpltCallback`：分发给 `app_imu` 和 `app_vision`
  - `HAL_UART_ErrorCallback`：分发串口错误恢复

## 4. 常用调用场景

### 手动直控电机

1. 蓝牙发送：`MOTOR 30 30 30 30`
2. `app_bt` 解析后调用 `Motion_SetDuty4`
3. `app_motion` 直接下发到 `bsp_dcmotor`

### 云台定步烟测

1. 蓝牙发送：`GIMBAL?`
2. 蓝牙发送：`GIMBAL EN 1`
3. 低速小步数测试：`GIMBAL MOVE X 20 200`、`GIMBAL MOVE Y 20 200`
4. 用 `GIMBAL?` 查看 `x_pos/y_pos` 和 `x_rem/y_rem`
5. 异常时发送：`GIMBAL ESTOP`

### 视觉瞄准骨架

1. MaixCAM 发送 `$V` 坐标帧，`app_vision` 更新 `dx/dy`
2. 蓝牙用 `GIMBAL CAL SET a b c d` 写入像素到步数矩阵
3. `AIM ONCE` 调用 `Aim_StartOnce`，误差超阈值时经 `Gimbal_MoveByPixelError` 发一段定步运动
4. `AIM TRACK` 每约 50ms 在云台空闲时按最新像素误差发一小段定步运动

### 自动循迹

1. 蓝牙发送：`TRACK START`
2. `app_bt` 通过 `app_task` 选择并启动 `Q1_TRACK`
3. `app_task` 调用 `Track_Start`
4. `app_track` 在 `LINE_FOLLOW` 中周期调用 `AppSensor_ReadNow`
5. 误差进入 `pid_core`
6. 输出经 `Motion_SetDuty4` 下发到底盘
7. 每完成 4 个有效转弯计 1 圈，达到 `LAPS` 后自动停车

### 自动直角弯

1. `app_sensor` 识别到右角点模式；左角点当前由 `app_track` 忽略
2. `app_track` 进入 `TURN_DELAY`
3. 等待 `TURN_DELAY_MS`，期间不再要求角点持续存在
4. `app_track` 调用 `Turn_Start(TURN_DIR_RIGHT)`
5. `app_turn` 在 `Turn_Poll` 中持续看 `Imu_GetYawDeg`
6. 到达 `TURN_ANGLE` 或超时 `MAX_TURN_MS` 后 `Turn_Stop`
7. `app_track` 进入 `RECOVER_LINE`

## 5. 蓝牙白名单参数归属

| 参数 | 归属模块 | 注释说明 |
| --- | --- | --- |
| `BASE` | `app_track` | 循迹基础速度，直线和出弯恢复都靠它 |
| `KP` | `app_track` | 比例修正强度，大了更积极修方向 |
| `KD` | `app_track` | 微分阻尼强度，大了更压摆动 |
| `TURN_DELAY_MS` | `app_track` | 首帧读到右角点后延时多久再开始右转 |
| `RECOVER_MS` | `app_track` | 转完后先直走多久，再回循迹 |
| `HEAD_EN` | `app_track` | 直线段 JY61P yaw 航向保持开关 |
| `HEAD_KP` | `app_track` | yaw 误差修正强度 |
| `HEAD_KD` | `app_track` | gyro_z 阻尼强度 |
| `HEAD_MAX` | `app_track` | 航向保持最大 duty 修正 |
| `TURN_OUT` | `app_turn` | 外侧轮前进速度，决定转弯快慢 |
| `TURN_IN` | `app_turn` | 内侧轮反转强度，决定转弯利索程度 |
| `TURN_ANGLE` | `app_turn` | 目标 yaw 角度，决定转多少度停 |
| `MAX_TURN_MS` | `app_turn` | 超时保护，防止异常时一直转 |
| `LAPS` / `N` | `app_track` | 目标圈数，范围 1~5 |
