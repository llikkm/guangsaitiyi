# 硬件接口、BOM 与通信协议

## 接线与 BOM 对照

BOM 指原始 week4接线表(1).xlsx 的 Sheet1。表中出现的 D 编号为接线表标注。实物接线以板卡型号、板号和 MCU 端口复核。供电、电源分配、逻辑电平、保护、驱动电流和采购型号无法从接线表确定。

| 模块 | 代码或文档能够确认的品类 | BOM 与 STM32 端 | 源码证据与待核对项 |
|---|---|---|---|
| 主控 | STM32F446RET6，README 指向 NUCLEO-F446RE | BOM A1 主控板。调试口 PA13 SWDIO、PA14 SWCLK | IOC:23–40、92–95；R:3。核对实物板版、供电跳线和接插件 |
| IMU | README 称 MPU6500。代码只接受 WHO_AM_I 为 0x70 | BOM A2:C3。PB3/D3 SDA，PB10/D6 SCL，I2C2 AF4，开漏无内部上拉 | MSP:93–110；M:228–249；C:389–448；R:8。核对模块、AD0、电压、拉阻及安装朝向 |
| OLED | 源码按 SSD1306 128×64 配置，7 位地址 0x3C | 与 IMU 共用 PB3 SDA、PB10 SCL。BOM A2:C3 | C:26、453–537。核对显示控制器、地址与上下拉 |
| 雷达 | 代码匹配 RPLIDAR 常规 SCAN 格式，具体型号及激光等级未确认 | BOM A4:D5。雷达 TX 接 PC11/USART3_RX，雷达 RX 接 PC10/USART3_TX，AF7 | MSP:461–498；M:554–574；C:276–343。核对波特、逻辑电平、供电、启动/转速控制、视场、铭牌和等级证明 |
| 直流电机驱动 A | 双输入 PWM；代码默认 A/E1 为左轮 | BOM A7:F8。PB6/TIM4_CH1 接 AIN1，PB7/TIM4_CH2 接 AIN2，AF2 | MSP:297–312；M:380–424；C:18–21、70–86。正软件方向默认驱动 PB7，反向 PB6。驱动芯片、使能、刹车真值表未确认 |
| 直流电机驱动 B | 双输入 PWM；代码默认 B/E2 为右轮 | BOM A9:F10。PA9/TIM1_CH2 接 BIN1，PA8/TIM1_CH1 接 BIN2，AF1 | MSP:275–290；M:262–322；C:18–21、70–86。正软件方向默认驱动 PA9，反向 PA8。核对绕线极性 |
| 左编码器 E1 | TIM3 正交计数，软件符号 +1 | BOM A11:F12。PA6/TIM3_CH1 接 E1A，PA7/TIM3_CH2 接 E1B，AF2 | MSP:196–213；M:331–366；C:14–19、641–674。核对输出类型、电平、每圈实际计数 |
| 右编码器 E2 | TIM8 正交计数，软件符号 −1 | BOM A13:F14。PC6/TIM8_CH1 接 E2A，PC7/D9/TIM8_CH2 接 E2B，AF3 | MSP:220–237；M:471–506；C:14–19、641–674。同样需要标定 |
| UART4 蓝牙 | 透明串口用途，蓝牙型号、无线协议和流控接线未给出 | BOM A15:G16。模块 TX 接 PA1/UART4_RX，模块 RX 接 PA0/UART4_TX，AF8 | MSP:431–456；M:521–540；PC:12–19；R:11。核对模块电压和断线状态信号 |
| 用户按钮 | PC13 外部上升沿中断，无内部拉阻 | BOM A6:E6，板载 user-button | M:603–622；IRQ:182–188。无业务回调，无软件急停功能 |
| 底盘、电源与线束 | 软件初始假设轮径 65 mm、轮距 145 mm、1320 计数/轮转 | 原表未给出电源、电机型号、减速比、轮胎规格、线序和保险等 | C:14–22；R:7–10。按实际轮周、轮距和整套编码器链标定；记录电池、驱动和供电规格 |

PB3 复用作 SDA 时要核对调试器的 SWO 和板上引出方式。默认 ST-LINK 虚拟串口与 UART4 是不同接口，板上默认路由需查对应板版。参考 [ST Nucleo-64 UM1724](https://www.st.com/resource/en/user_manual/dm00105823-stlink-v2-in-circuit-debugger-programmer-for-stm8-and-stm32-stmicroelectronics.pdf)。

## 时钟与定时器

M:186–220 的 SystemClock_Config 选择 HSI，关闭 PLL，AHB 和 APB1/APB2 均不分频，按配置取 16 MHz。IOC:149 与之相符。按 16 MHz 除以 (PSC + 1) 与 (ARR + 1) 的乘积推算频率。

| 定时器 | 配置 | 用途和依据 |
|---|---|---|
| TIM1 / TIM4 | PSC=0，ARR=799，向上，PWM1 高有效，CCR 初始 0，两个通道，20 kHz（50 μs） | 电机。TIM1 无 Break、死区 0、无自动输出。M:262–322、380–424；C:70–86、783–788 |
| TIM3 / TIM8 | PSC=0，ARR=65535，TI12 双通道正交，直接输入，极性配置 rising，输入分频 DIV1，滤波 0 | 16 位计数由编码器边沿推进，TI12 使用两路边沿。M:331–366、471–506；C:641–646、783–784 |
| TIM6 | HAL 时基，按当前 APB1 得 PSC=15，ARR=999，1 ms，IRQ 15 | IRQ:210–215；M:658–665；AMR_NEW_CodingDay4/Core/Src/stm32f4xx_hal_timebase_tim.c:40–110；IOC:82–85 |
| TIM7 | PSC=1599，ARR=99，10 ms，IRQ 5 | MPU 初始化完成后才启动，ISR 发采样信号量。M:433–458、667；MSP:252–263；C:348–351、426–432 |
| SysTick / DWT | RTOS tick 1 kHz；DWT 用周期计数换算微秒 | RT:67–74、139–167；C:177–200、217–219 |

参考 [ST STM32F446 RM0390](https://www.st.com/resource/en/reference_manual/dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)。

## 通信协议详表

| 接口 | 请求与响应 | 依据 |
|---|---|---|
| I2C2 | 100 kHz、7 位地址。MPU 读取 0x75 检测 0x70，写 0x6B=1 和 0x1B=0，周期读取 0x47 起两个字节，按大端 int16 解释。OLED 0x3C 使用 0x00 命令前缀和 0x40 数据前缀，页寻址。HAL 调用地址左移 1 | M:228–249；C:26、389–448、453–537 |
| USART3 雷达 | 460800、8N1、无流控。启动发 A5 25，等待 50 ms 后开启 1024 B 循环 DMA，再发 A5 20。匹配 A5 5A 05 00 00 40 81，按 5 B 节点处理。DMA1 Stream1 Channel4，外设至内存、字节对齐、内存自增、低优先级、禁 FIFO。USART3 无独立 NVIC ISR | M:554–594；MSP:461–498；IRQ:168–229；C:243–343 |
| UART4 入站 | 115200、8N1、无流控。ASCII，以 CR 或 LF 终止，最多 47 字节正文。支持大写纯 STOP 或 `x,y` / `(x,y)`。strtof 允许其支持的数值表示，要求两个数有限且每轴绝对值不大于 1.5 m。控制字符、超长或错误标记会使当前行丢弃；无 ACK、校验、会话序号或心跳 | M:521–540；C:254–274、550–604 |
| UART4 POSE | `POSE,tick,x_mm,y_mm,heading_tenth_deg,state,lidar_seq,valid_count,nearest_mm,lost_blocks,lost_results`。tick 单位 ms，0 停止、1 跟踪、2 到达；x/y/角度截断为整数 | C:136–156、658–769；PC:63–70 |
| UART4 LIDAR | `LIDAR,seq,count,nearest_mm`。seq 对应 DMA 半块，最近距离 0 表示该块无有效点；该行不含完整扫描或方向 | C:112–132、305–333；PC:75–77 |
| UART4 TIME | `TIME,task_id,period_us,execution_us,runs,imu_overrun,lost_commands,rx_errors,imu_errors,oled_errors,lidar_start_error`。三种出站消息均用 CRLF，TX 互斥保证发送事务之间不交错，发送失败可丢行 | C:90–95、123–130、151–171；PC:71–78 |

协议对照参考 [Slamtec RPLIDAR S/C 系列协议 v2.8](https://wiki.slamtec.com/download/attachments/83066883/LR001_SLAMTEC_rplidar_S%26C%20series_protocol_v2.8_en.pdf) 和 [InvenSense MPU-6500 寄存器手册 RM-MPU-6500A-00](https://www.ariat-tech.tw/datasheet/5b/83893401.pdf)。这些资料用于核对协议字段，实物具体型号和等级仍需查看模块本体。
