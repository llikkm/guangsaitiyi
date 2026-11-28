# Week 4 Coding Day 交付与演示

工程基于已有 AMR_NEW.ioc，目标 NUCLEO-F446RE。用 CubeIDE 的 **Existing Projects into Workspace** 从解压后的目录导入，工程名为 AMR_NEW。如果已有同名工程，请用另一个 CubeIDE 工作区导入。main.c 只在 USER CODE 区域接入启动函数和 TIM7 回调，主体实现位于 Core/Src/coding_day.c。

## 上板与标定

- 架空车轮，并先断开电机电源。核对编码器、雷达、电机驱动、IMU 和 OLED 的供电、电平与共地，再接电机电源。
- I2C2 使用 PB3（SDA）、PB10（SCL），速率 100 kHz。默认 IMU 为 LSM6DS3TR-C，OLED 为 SSD1306 128×64，地址 0x3C。上电时把车放平静置。静态采集 static10 的开头出现过冷启动异常，因此 LSM 配置完成后先等 2 秒，再用随后 100 个新样本估算陀螺仪零偏，约 3 秒后就绪。等待期间仍可读取原始样本。MPU6500 沿用启动后首批 100 个样本，约 1 秒。IMU 未就绪或超过 35 ms 没有新样本时，控制任务请求停车。
- 电机 A 使用 TIM4 CH1/CH2，反馈 E1 使用 TIM3。电机 B 使用 TIM1 CH2/CH1，反馈 E2 使用 TIM8。默认 A/E1 为左轮，B/E2 为右轮。先手转验证计数方向，再低速测试电机方向并确认 PWM 能克服静摩擦。
- 配置代码开头的 WHEEL_DIAMETER、WHEEL_TRACK、COUNTS_PER_REV 和方向符号。65 mm、145 mm、1320 个计数/圈和方向符号是初始化假设。用轮子转一整圈的实际计数标定；陀螺仪 GYRO_SIGN 也要按安装方向验证。调 PID 前先核对前进和原地左转的方向。
- 蓝牙 UART4 默认 115200、8N1。电脑配对后选择可收发的串口；运行 `python -m pip install pyserial` 和 `python pc_monitor.py COM7`，将 COM7 换成实际端口，且串口只能由一个上位机打开。

## IMU 接线、样本与验证

LSM6DS3TR-C 使用 3.3 V 逻辑，与车板共地。CS 拉高进入 I²C 模式，SDO/SA0 接低或高分别选择七位地址 0x6A、0x6B。核对模块的 SDA/SCL 上拉电阻和电源标注，固件里传给 HAL 的地址会左移一位。

默认驱动检查 WHO_AM_I（0x0F）等于 0x6A，复位芯片，写入并读回 CTRL1_XL=0x40、CTRL2_G=0x40、CTRL3_C=0x44。加速度和角速度均以 104 Hz 输出，量程分别为 ±2 g、±250 °/s。TIM7 每 5 ms 唤醒任务，状态寄存器的两类数据就绪位同时有效才读取新样本。固件从 0x20 连读 14 字节得到温度、角速度 XYZ 和加速度 XYZ，按小端有符号整数解码。芯片资料见 [ST 数据手册](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)。

任务中可调用 `ChassisImu_GetLatest(&sample)` 复制最近一帧，返回 1 表示已有数据；连续两次调用可通过 `seq` 判断是否拿到了同一帧。接口定义见 [chassis_imu.h](Core/Inc/chassis_imu.h)。UART4 继续传输 POSE、TIME 和 LIDAR。六轴样本在固件内部可读。

| 固件字段 | 对应采集文件的列 | 使用方法 |
|---|---|---|
| seq | seq | 从 0 起计，每次成功读取新样本后递增。轮询延迟可能漏掉芯片的中间帧 |
| read_tick_ms | t_sample_us | 固件记录读完的 RTOS 毫秒数，回放使用 CSV 自带的微秒时间。比较间隔时先统一单位，各次开机分别计时 |
| gx_raw、gy_raw、gz_raw | 同名三列 | 乘 0.00875 得到 °/s。用于实车航向时还要扣除本次开机零偏并核对安装方向 |
| ax_raw、ay_raw、az_raw | 同名三列 | 乘 0.000061 得到 g |
| temp_raw | temp_raw | 芯片原始温度计数 |

采集与回放说明见 [IMU 实测分析](../imu_analysis/README.md)。台架验证时架空车轮，先看调试变量 `imu_who_am_i`、`latest_imu` 和 `imu_ready`。静置时加速度模长应接近 1 g，正反转动应使 Z 轴角速度反号。记录实际采样间隔，检查 OLED 同时刷新时的掉样与超时。最后在台架断开 IMU 并观察停车响应。

原 MPU6500 底盘可在 CubeIDE 定义 `CHASSIS_IMU=CHASSIS_MPU6500`，随后清理并重建。该配置探测 0x68/0x69，验证 ID 0x70，以 100 Hz 读取大端数据，陀螺仪按 131 count/(°/s) 换算。两种配置均可从本目录执行 `.\build-check.ps1 -Sensor LSM6DS3TR_C` 或 `.\build-check.ps1 -Sensor MPU6500` 编译。生成文件分别位于 `BuildCheck/<型号>/AMR_NEW.elf`。

## 六人演示

| 成员 | 代码内容 | 现场查验 |
|---|---|---|
| 1 | 系统启动、电机 PWM、雷达结果输出、50 ms 遥测 | 上位机显示位姿轨迹和雷达数据，OLED 显示毫米、角度和状态 |
| 2 | 任务优先级、栈、绝对时间调度、DWT 测时 | 用 TIME 遥测保存各任务实际周期间隔和执行耗时 |
| 3 | Q_Lidar、Q_Result、Q_Cmd、Q_Path，及蓝牙 RX 字节队列 | 说明任务如何阻塞唤醒，并用序号和丢包计数验证数据交接 |
| 4 | TIM7、二值信号量和 IMU 采样 | 核对最新样本序号的增长率（LSM 标称 104 Hz），观察 imu_overrun 和 imu_errors |
| 5 | osMutexPrioInherit、OLED 分段更新 | 核对 I2C 事务结束才释放锁，观察 OLED 刷新时 IMU 的间隔 |
| 6 | 蓝牙命令、路点、里程计和航向 PID | 每条路线只发一次目标坐标，记录终点位置和航向误差 |

## Part 1 短分析

USART3 用 460800 baud、8N1，DMA 循环写入两个 512 B 半区。串口连续满载时写满一个半区最短约 11.1 ms。选用 512 B 是为了在降低中断频率的同时给解析留出这段时间。ISR 只把 offset、length、sequence 投进 Q_Lidar，不复制数据。解析任务以较高优先级被唤醒，直接读取相应半区，解析前后核对序号与 DMA NDTR；被覆盖或跳序时丢弃残留节点。SCAN 流含 7 B 描述符和 5 B 测量节点，节点可以跨块。解析结果包含有效测距数量和最近距离，通过 Q_Result 唤醒独立输出任务并发出 LIDAR 行。雷达结果用于监测。解析队列和结果队列均为 4 条，满时丢弃新消息并计数。

任务等待空队列时处于 BLOCKED，ISR 入队后变为 READY，由调度器选择进入 RUNNING。队列需要控制块与消息存储，每个任务需要 TCB 和栈。TIME 输出的是 DWT 测得的最近一次执行间隔和含抢占的墙钟耗时。保存实际数据后填入报告。

## Part 2 调度与试验

| ID | 任务 | 触发 | 优先级 | 栈（字节） |
|---|---|---|---|---:|
| 0 | Control（位姿、PID、输出） | 10 ms | High | 768 |
| 1 | IMU | TIM7，LSM 每 5 ms 检查就绪，MPU 每 10 ms 采样 | High1 | 640 |
| 2 | LiDAR | DMA HT/TC → Q_Lidar | AboveNormal | 768 |
| 3 | Output | Q_Result | Normal | 640 |
| 4 | OLED | 分块连续刷新 | AboveNormal1 | 1024 |
| 5 | Bluetooth TX | 50 ms | BelowNormal | 1024 |
| 6 | Bluetooth RX | UART RX 中断 → 字节队列 | AboveNormal | 1024 |
| 7 | Planner | Q_Cmd | Normal | 1024 |

Q_Cmd 深度 4，Q_Path 深度 2，蓝牙字节队列深度 64。新命令使用序号使旧路径失效。命令或路点入队失败时保持停车并计数。OLED 显示 X/Y（mm）、H（°）、S（0 停车、1 跟踪、2 到达），每屏 8 页、每页 8 块，成功后每块等待 4 tick，另计 I2C 传输和任务调度时间。航向以 +X 为 0°、逆时针为正，起步车头朝 +Y（90°）。

- 每条试验把车放回 (0,0)，车头朝 +Y，重启并静置完成零偏校准。用实测车身中心和航向填写终点，里程计显示可另存作对比。
- 路点间距不超过 0.25 m，往侧支路时先沿纵向到目标 y，再沿横向到目标 x。课件第 2 页第三个目标写 (0.70,-0.70)，第 3 页图示写 (0.70,0.70)；上位机支持两个字面坐标，需照实际场地和 TA 口径选择。正 y 版本经 (0,0.70)。负 y 版本要求场地在负 y 一侧有可通行走廊。
- 位置误差 = sqrt((x实测−x目标)^2+(y实测−y目标)^2)。航向误差采用实测航向与规划末段方向的最小角差。控制在终点距离 35 mm 以内停止，最终实际误差以测量为准。

| 命令 | 参考终点航向 | 实测 x,y | 实测航向 | 位置误差 | 航向误差 |
|---|---:|---|---|---|---|
| (0.00,1.40) | 90° | 待测 | 待测 | 待测 | 待测 |
| (-0.70,0.70) | 180° | 待测 | 待测 | 待测 | 待测 |
| (0.70,-0.70) 或 (0.70,0.70) | 0° | 待测 | 待测 | 待测 | 待测 |

2026-11-28 已用本机 STM32CubeIDE 的 ARM GCC 对两种配置完成编译和链接，分别生成 LSM6DS3TR-C 和 MPU6500 的 ELF 文件。本次未烧录，接线、采样时序和停车响应仍待实车台架验证。
