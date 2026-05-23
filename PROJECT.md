# Project Map

## Goal
- STM32F407 HAL 竞赛版底盘工程，目标是现场调车优先的循迹/直角弯/蓝牙调参固件。
- 当前主线是 `app_task` 多题顶层调度、四轮 duty 直控、五路灰度循迹、JY61P yaw 直角弯、UART5 蓝牙 ASCII 命令、USART2 MaixCAM 状态接入、TIM9/TIM12 步进云台定步控制。
- 当前对齐题目：`E题_简易自行瞄准装置.pdf` 基本要求（1），小车自动沿 100cm 正方形黑线循迹，圈数 `N=1~5` 可设定，`t <= 20s`，瞄准模块电源断开；当前固件调试配置固定只接受右转角点。
- 当前题目一优化方向：恢复灰度 PID 巡线，JY61P 只用于 `app_turn` 直角弯；灰度右角点为 `00111/01111`，角点触发后进入 `CORNER_ADVANCE` 按 `BASE` 继续前进 `CORNER_ADVANCE_MS`（默认 220ms）再右转；`CENTER_BIAS`（默认 0）用于居中态固定纠偏。
- 近期准备目标：基本要求（2）瞄准链路；MaixCAM 输出靶心像素坐标/有效标志/新鲜度，并以激光等效落点 `aim_x/aim_y` 为基准发送 `dx=target_x-aim_x`、`dy=target_y-aim_y`，STM32 只消费相对激光点的误差。
- 当前瞄准链路实现顺序：先做 MaixCAM 正式 YOLO 脚本和采样输出，再采 X/Y 云台数据拟合一套全局 `2x2` 像素误差到步数矩阵，最后用 `GIMBAL CAL SET` + `AIM ONCE` 验证小步闭环。
- 当前视觉瓶颈是现场靶纸外观、距离、角度和移动条件与旧 `red_circle` 数据集不一致；下一步优先重采 MaixCAM 原始 `320x240` 实战图并重训 YOLO，而不是继续调 STM32 或串口节拍。
- 本轮补充数据集目标规模 `500~600` 张，必须覆盖近/中/最远距离、慢速移动、接近小车实战速度的动态样本、低对比/倾斜/边缘困难样本和无靶/干扰负样本；标注只框圆环/靶心外框，不框整张白纸。
- 一次性调试采图框架目标：蓝牙命令触发 STM32 经 USART2 通知 MaixCAM Pro，MaixCAM 立即保存当前相机帧为有序 JPG（如 `0001.jpg`、`0002.jpg`），重启后通过扫描保存目录继续使用下一个编号；该功能只用于本轮训练/调试采图，用完可删除，不进入 `app_task` 比赛任务链路，且必须能在 `TASK START`/循迹运行期间异步触发，不阻塞底盘控制轮询。
- 视觉逐帧调试流默认关闭；蓝牙 `VISION STREAM ON/OFF/?` 只用于按 MaixCAM `$V` 成功解析帧频率上报 `OK VRAW dt/hz/... line=$V...`，便于现场确认发送周期和蓝牙转发丢帧。
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
- MaixCAM Pro 坐标链路当前接入点是 USART2；MaixCAM 侧使用 A19=`UART1_TX`、A18=`UART1_RX`，MaixPy 设备为 `/dev/ttyS1`；`app_vision` 使用 128 字节行缓冲解析 `$V,<seq>,<valid>,<x>,<y>,<dx>,<dy>,<area>*<cs>` 帧，其中 `dx/dy` 必须相对激光等效像素点而不是画面中心。
- MaixCAM Pro 自带屏幕为 `640x480`；当前红圈 YOLO 训练/导出为 `imgsz=320`、模型输入 `320x320 letterbox`，第一版视觉坐标系统一使用相机全图 `320x240`。

## Code Map
- `Core/`、`Drivers/`、`cmake/stm32cubemx/` 是 CubeMX/HAL 骨架和生成入口，尽量只在 `USER CODE` 区维护。
- `CMakeLists.txt` 把 `modular3.0/` 下的用户模块编进目标 `hal3_0`。
- `modular3.0/bsp_*` 是硬件薄封装：电机、编码器、灰度、JY61P、步进 STEP/DIR/EN 定步控制。
- `modular3.0/pid_core` 是平台无关 PID 数学核，当前供 `app_track` 灰度 PID 巡线使用。
- `modular3.0/app_*` 是比赛应用层：`app_motion`、`app_sensor`、`app_imu`、`app_vision`、`app_gimbal`、`app_aim`、`app_turn`、`app_track`、`app_task`、`app_bt`。
- `app_bt/app_bt.c` 只保留 UART5 DMA、行队列、`BT_Init/BT_Poll` 和 HAL UART 回调托管；`app_bt/app_bt_commands.c` 负责蓝牙 ASCII 命令解析和分发。
- `app_vision/app_vision.c` 只保留 USART2 接收、`$V` 坐标帧解析和正式目标状态；`app_vision/app_vision_tools.c` 集中维护 `CAP` 采图和 `VISION STREAM` 调试工具，开关在 `app_vision_config.h`。
- `modular3.0/maixcam_pro` 是 MaixCAM Pro 侧脚本模块；`stm32_uart_smoke.py` 做 UART 烟测，`target_yolo_uart.py` 做正式 YOLO/靶心检测和 `$V` 坐标发帧。
- 模块接口速查在 `modular3.0/MODULE_GUIDE.md`；总览、命令和参数说明在 `README_COMPETITION.md`。

## Runtime
- `main.c` 初始化顺序：HAL/clock/GPIO/DMA/TIM/UART 后，依次 `Motion_Init`、`AppSensor_Init`、`Imu_Init`、`Vision_Init`、`Gimbal_Init`、`Aim_Init`、`Turn_Init`、`Track_Init`、`Task_Init`、`BT_Init`。
- 主循环顺序：`Imu_Poll` -> `Vision_Poll` -> `Gimbal_Poll` -> `Aim_Poll` -> `AppSensor_Poll` -> `Turn_Poll` -> `Track_Poll` -> `Task_Poll` -> `BT_Poll` -> `Motion_Poll`。
- 顶层任务链：蓝牙 `TASK 1`/`TASK START` 或兼容 `TRACK START` -> `app_task` 选择 `Q1_TRACK` -> `Track_Start` -> `Task_Poll` 观察完成/停止/错误。
- 自动比赛链：灰度读取 -> `Track_Poll` 状态机 -> 灰度 PID + `CENTER_BIAS` 巡线；灰度右角点触发后 `CORNER_ADVANCE` 按 `BASE/BASE` 前进 -> `Turn_Start` -> `Motion_SetDuty4`。
- 直角弯链：右角点 `00111/01111` -> 前进 `CORNER_ADVANCE_MS` -> `Turn_Start(TURN_DIR_RIGHT)` -> `app_turn` 按剩余角度查 `TURN_R0~TURN_R90` 角速度限制并用 `TURN_RAMP` 平滑输出 -> 内部 `SETTLE` 降到 0 -> 恢复直行；左角点当前被 `app_track` 忽略。
- `Track_Start` 和 `Turn_Start` 都要求 `Imu_IsOnline()` 成立；IMU 异常时蓝牙会返回 `ERR TRACK` 或 `ERR TURN`。
- 蓝牙链：UART5 ReceiveToIdle DMA -> `HAL_UARTEx_RxEventCallback` in `app_bt` -> 行命令队列 -> `BT_Poll` 调用 `app_bt_commands` 解析执行。
- UART 普通接收回调由 `app_bt` 托管，分发到 `Imu_HandleRxCplt` 和 `Vision_HandleRxCplt`；错误回调同样由 `app_bt` 分发恢复。
- 步进链：TIM9/TIM12 PWM 输出 STEP，Update IRQ -> `HAL_TIM_PeriodElapsedCallback` -> `BSP_Stepper_TIM_PeriodElapsedCallback` 计步、调速、到步停机；`app_gimbal` 提供二维相对步数和像素误差转步数接口，`app_aim` 消费 `Vision_GetTarget` 做一次瞄准/连续跟踪骨架。

## Integration Rules
- 新比赛行为优先接入 `app_track`、`app_turn`、`app_motion`、`app_sensor` 等应用层模块。
- 多题选择、总启动/总停止、总完成/失败原因归 `app_task`；单题能力仍留在对应 app 模块里。
- 新蓝牙命令接入 `app_bt`，再调用对应 app 模块的公开接口；不要在 HAL 回调里直接解析业务命令。
- MaixCAM 坐标协议解析应归 `app_vision`；后续瞄准/云台闭环应由新的应用层模块消费 `app_vision` 的公开目标数据，不要放进 `app_bt`、HAL 回调或 `main.c`。
- MaixCAM 第一版不裁小 AOI，先用 `320x240` 全图 YOLO；稳定跟踪后可在 MaixCAM 侧加动态 AOI，但必须把检测点映射回全图坐标后再计算 `dx/dy`。
- MaixCAM 正式脚本应留在 `modular3.0/maixcam_pro`，用轻量状态机管理 UART、相机、模型加载、检测和发帧；STM32 侧协议格式暂不扩展，避免同时改两端；MaixCAM API/例程优先查 `E:\maixcam` 本地官方例程和文档，本地不足时再查官方在线资料。
- 蓝牙触发采图是一次性测试入口：命令分发在 `app_bt_commands.c`，STM32 侧 `CAP`/`VISION STREAM` 状态机集中在 `app_vision_tools.c`；STM32 只短发送、不阻塞等待，MaixCAM 保存完成后回 ACK，文件命名和保存策略归 MaixCAM 脚本；不要接入 `app_task` 比赛任务链路。
- MaixVision IDE 入口是 `C:\Users\1\Desktop\MaixVision.lnk` -> `E:\maixcam\IDE\MaixVision\MaixVision.exe`；该 IDE 不支持热加载，后续修改 MaixCAM Pro 程序时按整个 `modular3.0/maixcam_pro` 文件夹上传/运行，不按单个文件烧录；项目根需要默认启动文件 `main.py`。
- 基本要求（2）（3）先用“一套全局 `2x2` 矩阵 + `0.5~0.7` 小步闭环”，不按轨道边、前后左右或旧工程单轴非线性 K 分段；进阶动态题再把 `AIM_TRACK` 升级为小周期增量定步或速度模式。
- 云台步进 BSP 应提供按步数运动、剩余步数、忙闲、停止/急停等位置型接口；`app_gimbal` 再负责二维云台语义、状态机、蓝牙烟测和后续视觉闭环入口。
- 激光等效瞄准点 `aim_x/aim_y` 的校准优先放在 MaixCAM 侧；STM32 不默认画面中心就是激光点。若小车到靶面距离变化导致视差明显，应在 MaixCAM 侧做距离分段、动态激光点识别，或保守依赖 `app_aim` 的视觉复查小步闭环。
- 题目基本要求（1）的圈数 N、已完成圈数、自动停车应归 `app_track`；蓝牙只负责 `SET/GET` 和 `TRACK START/STOP` 入口。
- 题目一直线走直当前只调 `BASE`、`KP`、`KD`、`CENTER_BIAS`、`LEFT_TRIM`、`RIGHT_TRIM`；角点后前进距离调 `CORNER_ADVANCE_MS`；灰度 PID 是直线巡线主控制，JY61P 不参与巡线；不要让 `app_sensor` 依赖 IMU 或把航向决策放进 `main.c`/HAL 回调。
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
- 题目一直线循迹烟测：上板后低速 `TRACK START`，观察 `TRACK?`/`STATUS` 中灰度 `raw/corner/norm`、`kp/kd/center_bias`、`corner_advance_ms/advance_left_ms`、`line/corr`、圈数和停机原因；转弯仍用 `IMU?`/`TURN?` 观察 yaw、gyro_z、`rate_lim/scale/out/in`。
- MaixCAM 坐标烟测：MaixCAM 与 STM32 共地，MaixCAM A19/TX 接 STM32 PD6/USART2_RX；若需要 STM32 反向发命令，再接 STM32 PD5/USART2_TX 到 MaixCAM A18/RX。运行 `modular3.0/maixcam_pro/stm32_uart_smoke.py` 后，用蓝牙 `VISION?` 确认 `online=1`、`rx` 增长、`ok` 增长、`seq` 递增、`dx` 变化。
- MaixVision 上传方式：打开桌面 `MaixVision.lnk` 后，把 `modular3.0/maixcam_pro` 当作项目文件夹上传到 MaixCAM Pro；不要依赖 IDE 打开的单个文件热加载。
- MaixCAM YOLO 烟测：把 `target_yolo.mud` 放入 `modular3.0/maixcam_pro`，确保该文件夹根目录存在 `main.py` 默认入口，再上传整个文件夹；默认运行 YOLO 链路时用蓝牙 `VISION?` 确认 `online=1`、`valid` 随目标出现变化、`x/y/dx/dy/area` 合理。
- MaixCAM 动态调试：`target_yolo_uart.py` 保持串口逐帧发送；当前补光/低阈值诊断阈值 `conf=0.15`，日志用 `raw_valid/raw_objects/candidate_objects/miss_streak/max_miss` 区分真实检出、过滤和连续失检；YOLO 当前帧失检时立即发送 `valid=0`，不再复用最近目标或保存失检图。
- STM32 逐帧转发调试：运行 MaixCAM YOLO 后，蓝牙 `VISION STREAM ON`，预期每个成功解析 `$V` 帧输出一条 `OK VRAW`，其中 `dt/hz` 接近 MaixCAM 屏幕 `FPS`；调完用 `VISION STREAM OFF` 关闭，避免 UART5 调试输出影响控制轮询。
- MaixCAM 采图烟测：运行带采图命令解析的 MaixCAM 脚本后，蓝牙发送 `CAP`；预期 STM32 收到保存 ACK，MaixCAM 保存目录出现连续编号 JPG；重启 MaixCAM 后再次 `CAP` 应从已有最大编号继续递增。
- MaixCAM 瞄准点烟测：先在 MaixCAM 画面内确认激光等效落点 `aim_x/aim_y`；后续 `VISION?` 里的 `dx/dy` 应表示靶心相对该点的误差，而不是相对画面中心。
- MaixCAM 视觉尺寸烟测：确认脚本打印或显示 `img=320x240`、`detector=320x320`，靶心点和 `aim_x/aim_y` 都在 `320x240` 全图坐标内。
- 云台标定烟测：固定小车和靶面，分别采 `X+`、`X-`、`Y+`、`Y-` 小步运动后的 `step_x/step_y/target_x/target_y/dx/dy/area`；矩阵写入后确认每次 `AIM ONCE` 都让 `dx/dy` 绝对值变小。
- 步进云台烟测：硬件未确认限位前只用低速小步数；先 `GIMBAL?`，再 `GIMBAL EN 1`，然后 `GIMBAL MOVE X 20 200`、`GIMBAL MOVE Y 20 200`、`GIMBAL ZERO`、`GIMBAL STOP`/`GIMBAL ESTOP`。
- 上板运动烟测：先用 `MOTOR lf rf lb rb` 校验四轮方向，再单独测 `TURN L`/`TURN R` 并用 `TURN?` 看 `phase/remain/gyro_z/rate_lim/scale/out/in`，最后 `TRACK START`。

## Guardrails
- `app_motion` 是电机和编码器所有权边界，避免其他模块直接操作四轮 PWM/方向脚。
- `app_task` 只做多题顶层调度，不直接操作 GPIO/PWM/UART，也不承载具体循迹或瞄准策略。
- `app_bt` 是 UART5 收发和 HAL UART 回调分发点，避免重复定义 HAL UART callback；命令解析留在同目录 `app_bt_commands.c`，不要把调试命令塞回传输层。
- `app_vision_tools` 只承载 `CAP` 和 `VISION STREAM` 这类调试/采数能力，不承载正式瞄准策略。
- `app_turn` 依赖 IMU online 和 yaw，现场调直角弯时先确认 `IMU?` 正常。
- 当前对齐基本要求（1）时不要启动瞄准/云台动作；基本要求（2）（3）和发挥题的瞄准策略应放在 `app_aim`，云台运动语义放在 `app_gimbal`。
- `Track_SetParam` 和 `Turn_SetParam` 的白名单参数只在运行期生效，不保存到 Flash。
- `Drivers/` 和 `Core/` 生成区不要做大改；涉及引脚、定时器、UART 绑定时优先用 CubeMX 或同步更新 `.ioc`。
- MaixCAM Pro 侧脚本修改后必须按文件夹重新上传/运行；不要假设 MaixVision 会热加载或支持单文件烧录。
