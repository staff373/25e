# Project Map

## Goal
- STM32F407 HAL 竞赛版底盘工程，目标是现场调车优先的循迹/直角弯/蓝牙调参固件。
- 当前主线是 `app_task` 多题顶层调度、四轮 duty 直控、五路灰度循迹、JY61P yaw 直角弯、UART5 蓝牙 ASCII 命令、USART2 MaixCAM 状态接入、TIM9/TIM12 步进云台定步控制。
- 当前对齐题目：`E题_简易自行瞄准装置.pdf` 基本要求（1），小车自动沿 100cm 正方形黑线循迹，圈数 `N=1~5` 可设定，`t <= 20s`，瞄准模块电源断开；当前固件调试配置固定只接受右转角点。
- 当前题目一优化方向：恢复灰度 PID 巡线，JY61P 只用于 `app_turn` 直角弯；灰度右角点为 `00111/01111`，角点触发后进入 `CORNER_ADVANCE` 按 `BASE` 继续前进 `CORNER_ADVANCE_MS` 再右转；普通循迹 preset 为 `BASE=33`、`KP=60`、`KD=8`、`CENTER_BIAS=-0.8`、`CORNER_ADVANCE_MS=20`、`LEFT_TRIM/RIGHT_TRIM=0.89/1.00`；发挥题 ADV preset 为 `BASE=26`、`KP=50`、`KD=8`、`CENTER_BIAS=1.3`、`CORNER_ADVANCE_MS=130`、`LEFT_TRIM/RIGHT_TRIM=0.95/1.00`；转弯默认已全局改慢用于发挥（1）视觉跟踪验证，圈速需重新实测。
- 当前基本要求（2）（3）静态瞄准已能稳定命中；当前调试阶段 MaixCAM 暂以屏幕中心作为视觉零点，真实激光点尚未接入；后续接入激光后再改为 `dx=target_x-aim_x`、`dy=target_y-aim_y`。
- 当前下一目标：发挥部分。小车从 AB 段、前沿投影与 AC 线对齐处启动，底盘和瞄准模块同时工作；运动期间激光必须连续射向靶面。发挥（1）N=1、t<=20s、D1<=2cm；发挥（2）N=2、t<=40s、D1<=2cm；发挥（3）N=1、t<=20s，激光沿靶面半径 6cm 红色圆弧同步画圆，光斑轨迹到圆弧最大距离 D2<=2cm，1 圈车对应 1 圈光斑，同步误差 <1/4 圈。
- 当前瞄准链路：MaixCAM 正式 YOLO 脚本输出 `$V`，`app_vision` 解析 `dx/dy`，`app_aim` 只消费新 `seq` 视觉帧并对 3 帧做中值滤波，`app_gimbal` 用全局 `2x2` 像素误差到步数矩阵执行定步运动；`TASK 2` 调用 `Aim_StartOnce(2000)` 做第二问静态闭环；`TASK 3` 调用 `Aim_StartQuestion3(4000)`，先 X 轴固定方向整圈搜索，运动中至少 3 个新有效帧发现目标后停稳 100ms，再静止确认 3 个新有效帧进入闭环；当前锁定阈值为 `2px`，最终锁定前第二问进入 `LOCK_VERIFY`、第三问进入 `Q3_LOCK_VERIFY`，停稳 80ms 后要求最新 raw 误差和 3 帧中值误差都在死区内并连续 3 个新有效帧确认，退出观察阈值为 `4px`；发挥（1）使用 `Aim_StartTrack()` 的连续伺服模式，`TRACKING` 不进入 `LOCKED`，只做死区保持、离区再追、短暂丢帧保持和长时间丢帧报错。
- 当前 `app_gimbal` 上电默认加载半增益矩阵 `[-1.07 -0.05; 0.04 0.91]` 且默认瞄准运动速度为 `1000 sps`、加速度 `6000 sps^2`；该矩阵已验证四个方向均能向中心收敛，满增益 `GIMBAL CAL SET -2.13 -0.10 0.08 1.82` 静态时会在目标附近超调振荡，发挥题动态跟随不要靠全局增大静态增益解决，应使用 ADV 专用前馈/预测或状态分段参数；新版硬件或重新装配后必须重测。
- 新数据集已完成补充，当前 YOLO 识别追踪基本能跟上小车运行速度，只剩低概率短暂失检；后续优先把偶发 `valid=0` 作为 `app_aim` 控制端容错问题处理，而不是在 MaixCAM 侧隐藏失检。
- 本轮视觉训练样本应继续保持近/中/最远距离、慢速移动、接近小车实战速度的动态样本、低对比/倾斜/边缘困难样本和无靶/干扰负样本覆盖；标注只框圆环/靶心外框，不框整张白纸。
- 一次性调试采图框架目标：蓝牙 `CAP`/`CAP?`/`CAP AUTO <ms>`/`CAP AUTO OFF` 触发 STM32 经 USART2 通知 MaixCAM Pro，MaixCAM 保存有序 JPG；该功能只用于本轮训练/调试采图，用完可删除，不进入 `app_task` 比赛任务链路，且必须能在 `TASK START`/循迹运行期间异步触发，不阻塞底盘控制轮询。
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
- Stepper/Gimbal: X 轴 `TIM9_CH2/PE6` + `PE15 DIR` + `PB13 EN`，Y 轴 `TIM12_CH1/PB14` + `PC13 DIR` + `PD15 EN`；当前步进驱动供电即抱死，EN 高/低电平不作为 hold 调参手段；TIM9/TIM12 Update IRQ 用于按 PWM 周期计步。
- MaixCAM Pro 坐标链路当前接入点是 USART2；MaixCAM 侧使用 A19=`UART1_TX`、A18=`UART1_RX`，MaixPy 设备为 `/dev/ttyS1`；`app_vision` 使用 128 字节行缓冲解析 `$V,<seq>,<valid>,<x>,<y>,<dx>,<dy>,<area>*<cs>` 帧，其中 `dx/dy` 必须相对激光等效像素点而不是画面中心。
- MaixCAM Pro 自带屏幕为 `640x480`；当前红圈 YOLO 训练/导出为 `imgsz=320`、模型输入 `320x320 letterbox`，第一版视觉坐标系统一使用相机全图 `320x240`。

## Code Map
- `Core/`、`Drivers/`、`cmake/stm32cubemx/` 是 CubeMX/HAL 骨架和生成入口，尽量只在 `USER CODE` 区维护。
- `CMakeLists.txt` 把 `modular3.0/` 下的用户模块编进目标 `hal3_0`。
- `modular3.0/bsp_*` 是硬件薄封装：电机、编码器、灰度、JY61P、步进 STEP/DIR/EN 定步控制。
- `modular3.0/pid_core` 是平台无关 PID 数学核，当前供 `app_track` 灰度 PID 巡线使用。
- `modular3.0/app_*` 是比赛应用层：`app_motion`、`app_sensor`、`app_imu`、`app_vision`、`app_gimbal`、`app_aim`、`app_turn`、`app_track`、`app_task`、`app_bt`。
- `app_bt/app_bt.c` 只保留 UART5 DMA、行队列、`BT_Init/BT_Poll` 和 HAL UART 回调托管；`app_bt/app_bt_commands.c` 负责蓝牙 ASCII 命令解析和分发。
- `app_vision/app_vision.c` 只保留 USART2 接收、`$V` 坐标帧解析和正式目标状态；`app_vision/app_vision_tools.c` 集中维护 `CAP`/`CAP AUTO` 采图和 `VISION STREAM` 调试工具，开关在 `app_vision_config.h`。
- `modular3.0/maixcam_pro` 是 MaixCAM Pro 侧脚本模块；`stm32_uart_smoke.py` 做 UART 烟测，`target_yolo_uart.py` 做正式 YOLO/靶心检测和 `$V` 坐标发帧。
- `modular3.0/maixcam_pro_debug` 是历史失检/远距调试图片和 contact sheet 目录，不参与 STM32 构建或 MaixCAM 正式上传；除非复盘样本，不要把它当运行入口。
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
- 蓝牙触发采图是一次性测试入口：命令分发在 `app_bt_commands.c`，STM32 侧 `CAP`/`CAP?`/`CAP AUTO` 和 `VISION STREAM` 状态机集中在 `app_vision_tools.c`；STM32 只短发送、不阻塞等待，MaixCAM 保存完成后回 ACK，文件命名和保存策略归 MaixCAM 脚本；不要接入 `app_task` 比赛任务链路。
- MaixVision IDE 入口是 `C:\Users\1\Desktop\MaixVision.lnk` -> `E:\maixcam\IDE\MaixVision\MaixVision.exe`；该 IDE 不支持热加载，后续修改 MaixCAM Pro 程序时按整个 `modular3.0/maixcam_pro` 文件夹上传/运行，不按单个文件烧录；项目根需要默认启动文件 `main.py`。
- 基本要求（2）（3）先用“一套全局 `2x2` 矩阵 + 分段增益小步闭环”，不按轨道边、前后左右或旧工程单轴非线性 K 分段；当前误差增益为大误差 `0.70`、中误差 `0.50`、小误差 `0.30`。已修正旧的 3 帧中值误停问题：最新 raw `dx=6 dy=2` 时不得进入 `LOCKED`，必须回到闭环修正。第三问搜索阶段只动 X 轴，默认 Y 轴由步进记忆预置到可见高度；若仍晃，优先调 `AIM_Q3_SCAN_SPEED_SPS`、`AIM_Q3_SCAN_SETTLE_MS` 和小误差增益，再考虑改矩阵或视觉侧。
- 发挥（1）当前实现入口为 `TASK ADV1`/`TASK 4` -> `TASK START`，选择 `TASK ADV1` 时会立即套 `TRACK_PRESET_ADV` 方便 `GET`/`TRACK?` 预查，启动时再同时启动 `Track_Start` 和 `Aim_StartTrack`；`TASK ADVTRACK`/`TASK AT` 是进阶题巡线-only 测试入口，选择后立即套 `TRACK_PRESET_ADV`，`TASK START` 只启动 `Track_Start` 不启动视觉瞄准；`TASK 1` 选择和启动时会套回 `TRACK_PRESET_NORMAL`，避免普通题和发挥题参数互相污染；为避免转弯时目标瞬间出画面，`app_turn` 全局默认已改成慢转弯：`TURN_OUT=42`、`TURN_IN=-1.5`、`TURN_RAMP=1.0`、`TURN_RATE_SCALE=0.60`，改前快照保存在根目录 `data.md`；`app_track` 仍只负责圈数/停车，`app_aim` 负责连续视觉闭环，`app_task` 负责组合任务完成/失败判定。发挥（2）后续可复用该组合任务并改目标圈数为 2；发挥（3）再在 `app_aim` 增加圆轨迹相位目标，不要把画圆逻辑放进 `app_track` 或 `app_gimbal`。
- 动态可见性当前实测：直线段 MaixCAM 不会稳定跟丢，但贴合度仍差；转弯时目标容易被甩出闭环导致丢失。发挥（1）整合方案：当前激光模块未到，先允许 MaixCAM 以画面中心作为视觉零点；静态瞄准沿用半增益矩阵，不全局增大静态增益；ADV 动态跟随在 `app_aim` 内增加弱视觉速度预测用于直线段，转弯段使用 `CORNER_ADVANCE` 预置步数、`TURNING` 按 yaw 进度/角速度直接发云台步数前馈、`RECOVER_LINE` 清前馈回视觉闭环；前馈参数只运行期调试，不保存 Flash。
- ADV 动态瞄准默认方向：直线预测 `AIM_LINE_PRED_X/Y=1.0/0.15`；右转前馈先按 X 正方向补偿，转弯前馈已绕过像素标定矩阵直接走 `Gimbal_MoveRelativeSteps`，默认 `AIM_TURN_PREFEED_X/Y=220/0`、`AIM_TURN_FF_X/Y_PER_DEG=5.5/0`、`AIM_TURN_FF_GYRO_X/Y=0.35/0`、`AIM_TURN_FF_MAX_STEP=140`、`AIM_TURN_FF_SPEED_SPS=2200`。若右转时 `dx` 被越推越大，优先把 X 相关参数整体改成负值。
- ADV 动态瞄准前馈的集成边界：`app_task` 只把当前 `Track_State`、`Turn` 方向/进度/角速度等上下文喂给 `app_aim` 的公开 hint API；`app_track` 和 `app_turn` 不直接调用 `app_gimbal`；`app_gimbal` 只执行相对步数/像素误差运动，不承载题目相位策略。
- 云台步进 BSP 应提供按步数运动、剩余步数、忙闲、停止/急停等位置型接口；`app_gimbal` 再负责二维云台语义、状态机、蓝牙烟测和后续视觉闭环入口。
- 激光等效瞄准点 `aim_x/aim_y` 的校准优先放在 MaixCAM 侧；当前激光模块未到时允许临时使用画面中心作为调试零点，接入激光后不得继续默认画面中心就是激光点。若小车到靶面距离变化导致视差明显，应在 MaixCAM 侧做距离分段、动态激光点识别，或保守依赖 `app_aim` 的视觉复查小步闭环。
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
- 蓝牙串口烟测：`PING`、`STATUS`、`TASK?`、`TASK 1`、`TASK 2`、`TASK 3`、`TASK START`、`TASK STOP`、`SENSOR?`、`IMU?`、`VISION?`、`CAP?`、`GIMBAL?`、`AIM?`。
- 题目一直线循迹烟测：上板后低速 `TRACK START` 或 `TASK 1`/`TASK START`，观察 `TRACK?`/`STATUS` 中 `preset=NORMAL`、灰度 `raw/corner/norm`、`kp/kd/center_bias`、`corner_advance_ms/advance_left_ms`、`line/corr`、圈数和停机原因；转弯仍用 `IMU?`/`TURN?` 观察 yaw、gyro_z、`rate_lim/scale/out/in`。
- MaixCAM 坐标烟测：MaixCAM 与 STM32 共地，MaixCAM A19/TX 接 STM32 PD6/USART2_RX；若需要 STM32 反向发命令，再接 STM32 PD5/USART2_TX 到 MaixCAM A18/RX。运行 `modular3.0/maixcam_pro/stm32_uart_smoke.py` 后，用蓝牙 `VISION?` 确认 `online=1`、`rx` 增长、`ok` 增长、`seq` 递增、`dx` 变化。
- MaixVision 上传方式：打开桌面 `MaixVision.lnk` 后，把 `modular3.0/maixcam_pro` 当作项目文件夹上传到 MaixCAM Pro；不要依赖 IDE 打开的单个文件热加载。
- MaixCAM YOLO 烟测：把 `target_yolo.mud` 放入 `modular3.0/maixcam_pro`，确保该文件夹根目录存在 `main.py` 默认入口，再上传整个文件夹；默认运行 YOLO 链路时用蓝牙 `VISION?` 确认 `online=1`、`valid` 随目标出现变化、`x/y/dx/dy/area` 合理。
- MaixCAM 动态调试：`target_yolo_uart.py` 保持串口逐帧发送；当前补光/低阈值诊断阈值 `conf=0.15`，日志用 `raw_valid/raw_objects/candidate_objects/miss_streak/max_miss` 区分真实检出、过滤和连续失检；YOLO 当前帧失检时立即发送 `valid=0`，不再复用最近目标或保存失检图。
- STM32 逐帧转发调试：运行 MaixCAM YOLO 后，蓝牙 `VISION STREAM ON`，预期每个成功解析 `$V` 帧输出一条 `OK VRAW`，其中 `dt/hz` 接近 MaixCAM 屏幕 `FPS`；调完用 `VISION STREAM OFF` 关闭，避免 UART5 调试输出影响控制轮询。
- MaixCAM 采图烟测：运行带采图命令解析的 MaixCAM 脚本后，蓝牙发送 `CAP`；预期 STM32 收到保存 ACK，MaixCAM 保存目录出现连续编号 JPG；`CAP?` 应显示状态，`CAP AUTO <ms>`/`CAP AUTO OFF` 应能按间隔自动采图并停止；重启 MaixCAM 后再次 `CAP` 应从已有最大编号继续递增。
- MaixCAM 瞄准点烟测：先在 MaixCAM 画面内确认激光等效落点 `aim_x/aim_y`；后续 `VISION?` 里的 `dx/dy` 应表示靶心相对该点的误差，而不是相对画面中心。
- MaixCAM 视觉尺寸烟测：确认脚本打印或显示 `img=320x240`、`detector=320x320`，靶心点和 `aim_x/aim_y` 都在 `320x240` 全图坐标内。
- 云台标定烟测：固定小车和靶面，分别采 `X+`、`X-`、`Y+`、`Y-` 小步运动后的 `step_x/step_y/target_x/target_y/dx/dy/area`；矩阵写入后确认每次 `AIM ONCE` 都让 `dx/dy` 绝对值变小。
- 第三问烟测：先确认 Y 轴预置高度正确，再把 X 初始方向转到目标不可见，发送 `TASK 3`、`TASK START`；用 `AIM?` 观察 `Q3_SCAN_REV -> Q3_SCAN_SETTLE -> Q3_STABLE_CONFIRM -> Q3_AIM -> Q3_LOCK_VERIFY -> LOCKED`，`q3_seen` 到 3 后才停止搜索，`q3_stable` 到 3 后才进入闭环，`q3_lock` 到 3 后才完成；`q3_rev` 默认 2400 步，后续仍可按实测一圈步数校正。
- 发挥（1）巡线-only 烟测：发送 `TASK ADVTRACK` 后可直接用 `GET`/`TRACK?` 确认 `preset=ADV base=26.0 kp=50.0 trim=0.950/1.000`；再发送 `TASK START` 启动巡线，确认 `TASK? mode=ADV_TRACK_TEST`、`AIM? state=IDLE`；该模式只验证 ADV 低速循迹和转弯，不启动视觉瞄准。
- 发挥（1）完整烟测：发送 `TASK ADV1`、`TASK START`，确认 `TASK? mode=ADV1_AIM_TRACK`、`TRACK? preset=ADV base=26.0 kp=50.0 trim=0.950/1.000`、`AIM? state=TRACKING trk=ACQUIRE/LINE_TRACK/LINE_PREDICT/TURN_PREFEED/TURN_FEED/RECOVER_AIM/MISS_HOLD`，全程不应进入 `LOCKED`；上板后观察 `TRACK?` 圈数、`TURN?` 进度、`AIM?` 的 raw/f/lock_ok/miss_ms/moves/cmd_dx/cmd_dy/ff/v/hint、`VISION?` 的 valid/seq/dx/dy，预期运动中目标不长时间丢失，循迹完成时 `app_task` 停止底盘和瞄准。
- 步进云台烟测：硬件未确认限位前只用低速小步数；先 `GIMBAL?`，再 `GIMBAL EN 1`，然后 `GIMBAL MOVE X 20 200`、`GIMBAL MOVE Y 20 200`、`GIMBAL ZERO`、`GIMBAL STOP`/`GIMBAL ESTOP`。
- 上板运动烟测：先用 `MOTOR lf rf lb rb` 校验四轮方向，再单独测 `TURN L`/`TURN R` 并用 `TURN?` 看 `phase/remain/gyro_z/rate_lim/scale/out/in`，最后 `TRACK START`。

## Guardrails
- `app_motion` 是电机和编码器所有权边界，避免其他模块直接操作四轮 PWM/方向脚。
- `app_task` 只做多题顶层调度，不直接操作 GPIO/PWM/UART，也不承载具体循迹或瞄准策略。
- `app_bt` 是 UART5 收发和 HAL UART 回调分发点，避免重复定义 HAL UART callback；命令解析留在同目录 `app_bt_commands.c`，不要把调试命令塞回传输层。
- `app_vision_tools` 只承载 `CAP`/`CAP AUTO` 和 `VISION STREAM` 这类调试/采数能力，不承载正式瞄准策略。
- `app_turn` 依赖 IMU online 和 yaw，现场调直角弯时先确认 `IMU?` 正常。
- 当前对齐基本要求（1）时不要启动瞄准/云台动作；基本要求（2）（3）和发挥题的瞄准策略应放在 `app_aim`，云台运动语义放在 `app_gimbal`。
- `Track_SetParam` 和 `Turn_SetParam` 的白名单参数只在运行期生效，不保存到 Flash。
- `Drivers/` 和 `Core/` 生成区不要做大改；涉及引脚、定时器、UART 绑定时优先用 CubeMX 或同步更新 `.ioc`。
- MaixCAM Pro 侧脚本修改后必须按文件夹重新上传/运行；不要假设 MaixVision 会热加载或支持单文件烧录。
