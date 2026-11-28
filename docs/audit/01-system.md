# 工程、任务与数据流

本文行号固定对应原交付版本 8985608。按简写查阅[固定版本源码与当前函数索引](README.md)。

## 开发环境与构建

| 项 | 源码和配置证据 |
|---|---|
| MCU 与开发板 | STM32F446RET6、LQFP64、Cortex-M4。README 指向 NUCLEO-F446RE，IOC 的板卡字段为 custom。IOC:23–40、214；R:3 |
| IDE 与代码生成 | Eclipse CDT/STM32CubeIDE 管理工程，工程名 AMR_NEW。CubeMX 6.13.0，STM32Cube FW_F4 V1.28.3。以 Existing Projects into Workspace 导入工程目录。不要为做基线试验而先从 IOC 重新生成。AMR_NEW_CodingDay4/.project:3–16；IOC:65–66、117–147 |
| 编译 | 生成文件记录 GNU Tools for STM32 12.3.rel1。ARM GCC，C 使用 gnu11、Thumb、Cortex-M4、fpv4-sp-d16、hard float ABI、STM32F446xx、USE_HAL_DRIVER，Debug 命令为 -O0 -g3，Release 配置为 -Os。Debug/Core/Src/subdir.mk:3、7–16、41–48；.cproject:17–82、101–162（路径均在 AMR_NEW_CodingDay4/） |
| 链接与调试 | Debug/Release 选择 FLASH 链接脚本。FLASH 脚本把向量和代码放在 Flash，RAM 脚本把它们放在 RAM。脚本定义 512 KiB Flash 和 128 KiB RAM。生成构建链接 newlib-nano、nosys、libc、libm，并生成 map、反汇编和 size。旧 makefile 的 -T 是原电脑绝对路径。旧 .launch 记录 ST-LINK 和旧工程名 AMR_NEW_CodingDay4，导入后选当前工程的 ELF 新建启动配置。 .cproject:62、142；STM32F446RETX_FLASH.ld:35–60、132–176；STM32F446RETX_RAM.ld:35–60；Debug/makefile:60–78；AMR_NEW_CodingDay4 Debug.launch:17、37、56、72–79 |
| 独立构建检查 | build-check.ps1 在 C:\ST 递归找第一个 ARM GCC，枚举 Core、HAL、FreeRTOS 和启动汇编，用 -Os 编译并输出 BuildCheck/AMR_NEW.elf。此流程版本选择和优化级别与交付 Debug 有差别。仅在副本中运行。build-check.ps1:1–20 |
| 内置依赖 | CMSIS Core 与 STM32F4 设备头文件、STM32F4 HAL 驱动、FreeRTOS V10.3.1、CMSIS-RTOS2 适配、GCC ARM_CM4F port、heap_4，无独立雷达 SDK。来源见 Debug/sources.mk:24–32、Debug/objects.list、Middlewares/Third_Party/FreeRTOS/Source/include/task.h:46、RT:127 |
| PC 工具 | Python 3、Tkinter 和 pyserial。csv/sys/collections/datetime/pathlib 为标准库。脚本给出可视化轨迹、目标按钮及 CSV 日志，依赖的 pyserial 未固定版本。根目录 pc_monitor(1).py 与工程内副本 SHA-256 一致。PC:1–25、33–49、51–105；R:11 |

## 目录和入口

- Core/Src/coding_day.c 集中实现业务。main.c 初始化 HAL、时钟和外设，初始化内核，创建 defaultTask，调用 CodingDay_Init，再启动调度。freertos.c 是生成的空用户框架。Core/Inc 的 main.h、stm32f4xx_it.h、stm32f4xx_hal_conf.h 和 FreeRTOSConfig.h 提供设备、IRQ、HAL 模块和调度配置。M:96–179；C:774–790；AMR_NEW_CodingDay4/Core/Src/freertos.c:20–58。
- Core/Startup/startup_stm32f446retx.s 的 Reset_Handler 调用 SystemInit、初始化 .data/.bss、运行 C 库初始化，再调用 main。system_stm32f4xx.c 提供 CMSIS 系统初始化和时钟更新，syscalls.c、sysmem.c 为交叉工具链提供 C 库系统调用和堆边界。向量表与 stm32f4xx_it.c 承接中断，MSP 设置时钟、复用引脚和 DMA。启动汇编:60–100、124–203；system_stm32f4xx.c:167–255；syscalls.c:35–175；sysmem.c:53–78；IRQ:168–229。
- Drivers 是 ST HAL/CMSIS，Middlewares 是 FreeRTOS；Debug 中保存历史生成的 makefile、依赖文件、对象、ELF、map、反汇编和栈报告。本地交付目录中的 Debug 历史产物可供复核，Git 跟踪工程配置与源文件，由 CubeIDE 重新生成构建目录。工程根下 .mxproject 保存 CubeMX 前次生成文件索引，.settings 下分别保存编码、语言索引和寄存器视图或 IDE 设置。实际编译单元可在 Debug/objects.list 中逐项核实。

## 运行任务

RTOS 1 kHz tick、32 位 tick、抢占调度、FreeRTOS heap 15360 B。另有内核空闲任务和软件定时器服务任务（优先级 2、栈 256 words），应用无自建软件定时器。任务优先级数值来自 Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h:93–118。RT:59–105、127、137–167。

| ID | 任务及入口 | 优先级 | 栈 B | 周期、输入与输出 |
|---|---|---|---:|---|
| 0 | Control，control_task | High 40 | 768 | 10 ms 目标周期。处理路径，读取计数器及最新 IMU，写 PWM 与共享位姿。C:649–769 |
| 1 | IMU，imu_task | High1 41 | 640 | 探测及初始化后由 TIM7 每 10 ms 发二值信号量。写 gyro_rate、gyro_at、imu_ready。C:348–351、406–448 |
| 2 | LiDAR，lidar_task | AboveNormal 32 | 768 | Q_Lidar 事件。满线速半块间隔约 11.1 ms，非固定采样周期。发布 Q_Result。C:243–253、277–343 |
| 3 | Output，output_task | Normal 24 | 640 | Q_Result 事件。更新 latest_lidar 并从 UART4 发送 LIDAR 行。C:112–133 |
| 4 | OLED，oled_task | AboveNormal1 33 | 1024 | 连续扫描 8 页，每页 8 块，成功后每块调用 osDelay(4)，另计 128 笔 I2C 事务、抢占和故障重试。正常整屏是数百毫秒量级。C:453–537 |
| 5 | BluetoothTX，telemetry_task | BelowNormal 16 | 1024 | 50 ms 目标周期发 POSE。每 5 次附一条 TIME、轮换 8 个 ID，约 2 s 更新同一 ID。C:136–172 |
| 6 | BluetoothRX，bluetooth_task | AboveNormal 32 | 1024 | UART4 每字节接收中断，经 Q_Byte 唤醒；遇换行解析，发布 Q_Cmd。C:254–274、550–604 |
| 7 | Planner，planner_task | Normal 24 | 1024 | Q_Cmd 事件。读取 pose、生成折线路点并发布 Q_Path。C:606–638 |
| 无 | defaultTask，StartDefaultTask | Normal 24 | 512 | 反复 osDelay(1)，无业务。M:56–62、153–158、639–647 |

C:202–226 创建上述八个业务任务。交付 README 的任务表已按源码同步 Output 和 OLED 的配置。Control 和 BluetoothTX 用 periodic_wait 执行绝对 tick 调度，落后时把下次设为当前时间加周期。DWT 的 period_us 是上两次开始时间差，execution_us 包含被抢占和函数内等待。两次事件间隔超过约 268 s 时 16 MHz 的 32 位 DWT 取模，间隔会失真。C:177–200、217–219。

## 任务间通信与中断

| 资源 | 生产和消费 | 深度及行为 |
|---|---|---|
| Q_Lidar | USART3 DMA 半满/满 ISR 至 LiDAR | 4 条 DmaBlock。消息仅含半区 offset、512 B length 和序号，无字节备份。C:231–253、277–343 |
| Q_Result | LiDAR 至 Output，再更新最新统计 | 4 条 LidarResult，满时丢本次结果。C:112–132、231–240、305–333 |
| Q_Byte | UART4 ISR 至 BluetoothRX | 64 字节；满时计错并请求停止。C:231–274、570–604 |
| Q_Cmd / Q_Path | BluetoothRX 至 Planner 至 Control | 4 条 Command、2 条 Path。代号使旧规划失效；满时丢新条目并计数。C:231–240、586–592、616–636、688–702 |
| imu_tick | TIM7 至 IMU | 初值 0，上限 1；积压时无法保留每次采样，只累计 imu_overrun。C:348–351、426–448、774–781 |
| 锁与共享结构 | IMU/OLED 共用 I2C2 锁，Output/Telemetry 共用 UART4 TX 锁 | 两锁以 osMutexPrioInherit 创建。位姿、航向数据及部分统计用临界区快照。C:90–95、119–121、146–150、389–403、439–443、454–464、762–766、774–781 |

DMA1_Stream1、UART4、TIM7 和 PC13 EXTI 的 NVIC 优先级均为 5。RTOS 内核最低 15，允许使用 ISR 安全 API 的最高硬件优先级为 5，CMSIS 适配从 ISR 调用 FreeRTOS 的 FromISR 版本。TIM6 是 HAL 1 ms 时基；SysTick 是独立的 FreeRTOS 1 ms 时基。PC13 ISR 交给 HAL，业务文件无按钮停止回调。M:585–622、658–668；MSP:252–263、454–456；IRQ:168–229；RT:68、139–167；Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c:159–167、1634、1766。
