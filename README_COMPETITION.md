# hal3.0 竞赛版说明

## 工程来源

- 源工程：`E:\25e-2.0\STM32F407VET6\hal2.0`
- 目标工程：`E:\25e-2.0\STM32F407VET6\hal3.0`
- 目标：保留现有 CubeMX 外设映射，重构为现场调车优先、可扩展多道赛题的竞赛版底盘工程。

## 目录主线

- `Core/`、`Drivers/`、`cmake/`、启动文件、链接脚本、`CMakePresets.json`：直接沿用源工程基础骨架。
- `modular3.0/`
  - `bsp_dcmotor`：四轮 PWM + 方向脚输出。
  - `bsp_encoder`：四路编码器采样。
  - `bsp_sensor`：五路灰度读取。
  - `bsp_jy61p`：JY61P 串口姿态解析。
  - `bsp_stepper`：TIM9/TIM12 双轴 STEP/DIR/EN 定步控制。
  - `pid_core`：简单 PID 数学核，供 `app_track` 灰度 PID 巡线使用。
  - `app_motion`：四轮 duty 直控。
  - `app_sensor`：灰度状态包装。
  - `app_imu`：JY61P yaw / gyro_z / online。
  - `app_vision`：MaixCAM USART2 `$V` 坐标帧接收、解析和状态查询；`app_vision_tools.c` 集中放 `CAP` 采图和 `VISION STREAM` 调试流。
  - `app_gimbal`：二维云台相对步数、保持使能、像素到步数矩阵。
  - `app_aim`：一次瞄准和连续跟踪策略骨架。
  - `app_turn`：90 度直角弯执行。
  - `app_track`：第一问循迹跑圈状态机。
  - `app_task`：多题顶层调度，当前默认选择第一问 `Q1_TRACK`。
  - `app_bt`：UART5 蓝牙收发、行队列、回调托管；同目录 `app_bt_commands.c` 负责 ASCII 命令分发。
- 模块接口速查：见 `modular3.0/MODULE_GUIDE.md`。

## 模块功能与调用

### 主循环调用顺序

`main.c` 里的主循环顺序是：

`Imu_Poll -> Vision_Poll -> Gimbal_Poll -> Aim_Poll -> AppSensor_Poll -> Turn_Poll -> Track_Poll -> Task_Poll -> BT_Poll -> Motion_Poll`

这样安排的目的只有一个：先更新姿态和传感器，再让 `turn` / `track` 决策，然后由 `task` 汇总题目状态，最后由蓝牙命令和底盘轮速采样补齐本轮状态。

### 模块职责

| 模块 | 基础功能 | 主要调用 |
| --- | --- | --- |
| `bsp_dcmotor` | 单电机 PWM + 方向脚输出 | `Motion_Init` / `Motion_SetDuty4` |
| `bsp_encoder` | 单路编码器增量和累计计数 | `Motion_Poll` |
| `bsp_sensor` | 五路灰度原始读取、状态压缩、误差估计 | `AppSensor_ReadNow` |
| `bsp_jy61p` | JY61P 单字节接收和姿态解析 | `Imu_Init` / `Imu_Poll` |
| `bsp_stepper` | TIM9/TIM12 双轴步进定步输出和计步 | `Gimbal_MoveRelativeSteps` |
| `pid_core` | 简单 PID 数学内核 | `Track_Poll` |
| `app_motion` | 四轮 duty 直控，附带编码器采样 | `Track_Poll` / `Turn_Poll` / `BT_Poll` |
| `app_sensor` | 灰度状态包装，输出 raw / norm / corner 方向 | `Track_Poll` / `BT_Poll` |
| `app_imu` | 输出 yaw / gyro_z / online | `Turn_Poll` / `Track_Start` / `BT_Poll` |
| `app_vision` | USART2 接收 MaixCAM `$V` 坐标帧，输出 online / seq / valid / x / y / dx / dy / area / ok / bad；调试采图和逐帧流集中在 `app_vision_tools.c` | `BT_Poll` |
| `app_gimbal` | 云台 X/Y 相对步数、软件零点、保持使能、像素到步数矩阵 | `app_aim` / `BT_Poll` |
| `app_aim` | 基于视觉 `dx/dy` 的一次瞄准和连续跟踪骨架 | `BT_Poll` |
| `app_turn` | 90 度直角弯动作执行 | `Track_Poll` / `BT_Poll` |
| `app_track` | `IDLE/LINE_FOLLOW/CORNER_ADVANCE/TURNING/RECOVER_LINE/FINISHED/STOPPED` 循迹跑圈状态机 | `app_task` / `BT_Poll` |
| `app_task` | 多题选择、总启动/停止、总完成/失败原因 | `main.c` / `BT_Poll` |
| `app_bt` | UART5 收发和 UART 回调分发；命令解析在 `app_bt_commands.c` | `main.c` |

### 关键调用链

1. 自动比赛链：
   `AppSensor_ReadNow -> Track_Poll -> 灰度 PID + CENTER_BIAS 巡线 / 灰度角点触发 CORNER_ADVANCE -> Turn_Start -> Motion_SetDuty4`
2. 直角弯链：
   `右角点首帧 -> BASE/BASE 前进 CORNER_ADVANCE_MS -> Turn_Start(TURN_DIR_RIGHT) -> 剩余角度查角速度曲线 + duty 斜坡 -> SETTLE -> RECOVER_LINE`
3. 蓝牙调车链：
   `UART5 DMA 空闲接收 -> BT_Poll -> 命令解析 -> Track/Turn/Motion/Vision 参数或动作`
4. 多题入口链：
   `TASK 1 / TASK START -> app_task -> app_track -> Motion_SetDuty4`
5. 瞄准云台链：
   `Vision_GetTarget -> app_aim -> app_gimbal -> bsp_stepper -> TIM9/TIM12 STEP`

## UART / 硬件映射

- `UART5`：蓝牙，115200，`BT_TX` / `BT_RX`。
- `USART3`：JY61P，保留 yaw / gyro_z / online。
- `USART2`：MaixCAM / 视觉，`VISION_TX` / `VISION_RX`，支持收发开关、`$V` 坐标协议解析和状态查询。
- MaixCAM Pro 侧脚本在 `modular3.0/maixcam_pro/`；通信接线为 MaixCAM A19/TX -> STM32 PD6/USART2_RX，反向命令可接 STM32 PD5/USART2_TX -> MaixCAM A18/RX，两板共地。
- 步进云台：X 轴 `TIM9_CH2/PE6`、`PE15 DIR`、`PB13 EN`；Y 轴 `TIM12_CH1/PB14`、`PC13 DIR`、`PD15 EN`；EN 高电平使能。
- 四路 PWM、方向脚、四路编码器、五路灰度均沿用 `hal2.0` 当前 CubeMX 引脚与定时器绑定。

## 蓝牙命令

| 命令 | 说明 |
| --- | --- |
| `PING` | 连通性测试 |
| `STOP` | 停止循迹、停止转弯、停止电机 |
| `STATUS` | 查询当前主状态 |
| `SENSOR?` | 查询五路灰度当前状态 |
| `TRACK?` | 查询循迹状态、圈数、灰度角点、PID 修正、CENTER_BIAS 和停机原因 |
| `IMU?` | 查询 JY61P init / online / yaw / gyro_z |
| `VISION?` | 查询视觉接收、在线、坏帧、最新坐标和最近一帧状态 |
| `VISION ON` | 打开 USART2 接收 |
| `VISION OFF` | 关闭 USART2 接收 |
| `VISION STREAM ON/OFF/?` | 调试用逐帧蓝牙上报 MaixCAM `$V` 成功解析帧，默认关闭 |
| `GIMBAL?` | 查询云台状态、位置、剩余步数、保持使能和标定状态 |
| `GIMBAL ZERO` | 把当前 X/Y 软件位置清零 |
| `GIMBAL EN 1/0` | 打开或关闭步进保持使能 |
| `GIMBAL MOVE X <steps> <speed>` | X 轴相对移动指定步数 |
| `GIMBAL MOVE Y <steps> <speed>` | Y 轴相对移动指定步数 |
| `GIMBAL MOVE XY <x_steps> <y_steps> <speed>` | X/Y 双轴相对移动 |
| `GIMBAL STOP` | 云台停止 |
| `GIMBAL ESTOP` | 云台急停 |
| `GIMBAL CAL?` | 查询像素到步数 2x2 标定矩阵 |
| `GIMBAL CAL SET <a> <b> <c> <d>` | 设置 `x_steps=a*dx+b*dy`、`y_steps=c*dx+d*dy` |
| `AIM?` | 查询瞄准状态 |
| `AIM ONCE` | 启动一次瞄准骨架 |
| `AIM TRACK` | 启动连续跟踪骨架 |
| `AIM STOP` | 停止瞄准和云台 |
| `TASK?` | 查询顶层任务状态 |
| `TASK 1` | 选择第一问 `Q1_TRACK` |
| `TASK START` | 启动当前选择的题目 |
| `TASK STOP` | 停止当前题目并停车 |
| `TASK RESET` | 清顶层任务状态，保留当前题目选择 |
| `MOTOR lf rf lb rb` | 四轮 duty 直控 |
| `TURN?` | 查询转弯阶段、进度、剩余角度、gyro_z、角速度限制和当前输出 |
| `TURN L` | 原地左 90 度转弯 |
| `TURN R` | 原地右 90 度转弯 |
| `TRACK START` | 兼容入口，等价于选择第一问并启动 |
| `TRACK STOP` | 兼容入口，等价于停止当前任务 |
| `GET` | 查询全部白名单参数 |
| `SET <NAME> <VALUE>` | 修改白名单参数 |

## 白名单参数

| 参数 | 含义 | 当前范围 | 调参提示 |
| --- | --- | --- | --- |
| `BASE` | 循迹基础 duty | `0 ~ 100` | 大了平时跑更快，小了更稳 |
| `KP` | 灰度 PID 比例强度 | `0 ~ float 最大值` | 大了偏线修正更猛，太大容易左右抖 |
| `KD` | 灰度 PID 微分阻尼强度 | `0 ~ float 最大值` | 大了更压摆动，太大转向会发钝 |
| `CENTER_BIAS` / `BIAS` | 居中态固定纠偏 duty | `-100 ~ 100` | 默认 0；车在 `00100` 居中态仍外偏时小幅调整 |
| `CORNER_ADVANCE_MS` | 识别右角点后继续直行时间 | `0 ~ uint32 最大毫秒值` | 默认 220；大了更晚开始转，小了更早开始转 |
| `TURN_OUT` | 直角弯外侧轮 duty | `0 ~ 100` | 大了转弯更快 |
| `TURN_IN` | 直角弯内侧轮反转 duty | `-100 ~ 0` | 越负越利索，越像原地转 |
| `TURN_ANGLE` | yaw 停止角度 | `45 ~ 180` | 大了更容易转过头，小了更容易转不够 |
| `RECOVER_MS` | 转弯后直行恢复时间 | `0 ~ uint32 最大毫秒值` | 大了出弯更稳，小了更快回循迹 |
| `MAX_TURN_MS` | 转弯超时保护 | `100 ~ uint32 最大毫秒值` | 大了更宽松，小了更早保护停机 |
| `TURN_RAMP` | 转弯输出斜坡，每 10ms 最大 duty 改变量 | `0.1 ~ 100` | 小了更柔和但转弯响应慢，先用 `2~5` |
| `TURN_RATE_SCALE` | 15 度角速度限制曲线整体倍率 | `0.20 ~ 3.00` | 小了更稳，太小可能转不够；大了更快 |
| `TURN_RATE_KP` | 超过角速度限制后的 duty 收缩强度 | `0 ~ 0.10` | 大了更压转弯抖动，太大可能动力不足 |
| `TURN_STOP_RATE` | `SETTLE` 阶段放行的 gyro_z 阈值 | `1 ~ 720 deg/s` | 小了等车身更稳，太小会多等到超时兜底 |
| `TURN_R0/R15/.../R90` | 剩余角度为 0/15/.../90 度时的最大 gyro_z | `1 ~ 720 deg/s` | 每 15 度一个标定点，中间线性插值 |
| `LAPS` / `N` | 目标圈数 | `1 ~ 5` | 达到目标圈数后自动停车 |

参数只在运行期通过蓝牙 `SET` 生效，不做 Flash 保存。

## 比赛状态机

`IDLE -> LINE_FOLLOW -> CORNER_ADVANCE -> TURNING -> RECOVER_LINE -> LINE_FOLLOW`

达到目标圈数时进入 `FINISHED` 并停车；命令停止或保护停机进入 `STOPPED`。

- `LINE_FOLLOW`：灰度误差进 `pid_core`，再叠加 `CENTER_BIAS`，输出左右轮 duty；JY61P 不参与巡线。
- `CORNER_ADVANCE`：当前只接受右角点 `00111/01111`；触发后不停车，按 `BASE/BASE` 前进 `CORNER_ADVANCE_MS`，再调用 `Turn_Start(TURN_DIR_RIGHT)`。
- `TURNING`：外侧前进、内侧反转；`app_turn` 按剩余角度查 `TURN_R0~TURN_R90` 角速度限制并线性插值，超速时用 `TURN_RATE_KP` 收缩输出，再用 `TURN_RAMP` 做 duty 斜坡；接近目标角后进入 `SETTLE` 把输出降到 0，最后交给 `RECOVER_LINE`。
- `RECOVER_LINE`：转完后按 `BASE` 直行 `RECOVER_MS`，再恢复循迹。
- `FINISHED`：已完成 `LAPS` 圈，电机保持停止，保留圈数/角点/耗时状态。
- `STOPPED`：命令停止或保护停机。

## 文档位置

- `README_COMPETITION.md`：总览、命令、参数、状态机、调车建议。
- `modular3.0/MODULE_GUIDE.md`：模块功能、接口、调用关系。

## 编译

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

## 下载

VSCode 已固定到 `hal3.0` 当前固件：

- 调试/下载：选择 `STM32Cube: STM32 Launch ST-Link GDB Server`
- 直接烧录：运行任务 `STM32: Flash hal3 Debug ELF`

这两条路径都会先构建当前工程，再使用：

```text
build/Debug/hal3_0.elf
```

## 调车建议

1. 先用 `PING`、`SENSOR?`、`IMU?`、`VISION?` 确认三路串口状态。
2. 用 `MOTOR` 单独确认四轮正反和左右对应关系。
3. 用 `TASK?` 确认顶层任务处于 `SELECTED` / `Q1_TRACK`。
4. 用 `TURN L` / `TURN R` 单独把 `TURN_OUT`、`TURN_IN`、`TURN_ANGLE`、`TURN_RAMP`、`TURN_RATE_SCALE`、`TURN_RATE_KP`、`MAX_TURN_MS` 调稳；用 `TURN?` 看 `remain/rate_lim/scale/out/in`。
5. 再开 `TASK START` 或兼容 `TRACK START`，先调 `BASE`、`KP`、`KD`、`CENTER_BIAS`、`LEFT_TRIM`、`RIGHT_TRIM`、`CORNER_ADVANCE_MS`、`RECOVER_MS`；直线仍抖时看 `TRACK?` 里的 `raw/norm/corner/line/corr`。
6. 现场优先保证直角弯成功率，再压缩速度。
7. 云台未确认机械限位前，只用 `GIMBAL MOVE X 20 200`、`GIMBAL MOVE Y 20 200` 这类低速小步数烟测。
