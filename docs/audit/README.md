# STM32 自主导航小车代码审计

审计日期 2026-11-26。范围为工作区中的 AMR_NEW_CodingDay4 工程、原始交付包、PC 监视器和接线表。解压目录中与 ZIP 同名的文件逐项比对 SHA-256，结果一致。本文以 C 源码和工程配置为准，README 的演示描述及已有 ELF 作为交叉证据。

| 结论 | 状态 | 源码依据 |
|---|---|---|
| STM32F446RE 能独立执行左右轮 PWM、编码器里程计、陀螺积分和路点跟踪 | 已实现，精度和稳定性待实测 | C:70–86、406–448、641–769 |
| 能接收坐标，在已知初始位姿及路径无障碍的条件下尝试到达目标 | 已实现。只接受每轴绝对值不超过 1.5 m 的目标，路径固定先走 Y 再走 X | C:550–567、606–636、688–769 |
| 能否到达任意指定坐标 | 不能保证。坐标可解析、路径可通行、车实际到达是三个独立条件。代码只检查局部推算距离及角速度，缺少地图、避障、全局纠偏和终点朝向约束 | C:277–343、616–636、658–715 |
| 定位是否依赖场外设备 | 位姿计算在 STM32 内完成。外部串口用来发目标、停机和看遥测。开机人为把车摆到原点且车头朝 +Y，棋盘绝对坐标需要另有标定或定位来源 | C:42、136–172、570–604、658–686；R:43–47 |
| 雷达是否参与定位、避障或路径规划 | 原始字节解析和统计已实现。角度只用来校验节点，控制与规划均不消费雷达结果 | C:112–156、276–343、616–769 |
| 是否已有电机速度 PID、棋盘和对抗逻辑 | 缺失。现有 PID 的输入是目标方位角误差，输出是左右 PWM 差值。棋盘坐标、棋子识别、赛事决策和动态避障需要开发 | C:641–769；工程编译清单 Debug/objects.list 全文件 |

## 阅读顺序

1. [工程、任务与数据流](01-system.md)
2. [硬件接口、BOM 与通信协议](02-hardware.md)
3. [导航、风险与复用分类](03-navigation.md)
4. [车载 Linux 接入与验证方案](04-integration-tests.md)

下文路径均相对于工作区根目录，文件名简写如下。行号固定对应原交付提交 `8985608`，区间包含两端。下表链接固定到该提交。

查现行固件时按函数名搜索。驱动输出见 `pwm_pair`，雷达接收见 `lidar_task`，目标解析见 `parse_goal`，蓝牙输入见 `bluetooth_task`，规划见 `planner_task`，里程计和控制见 `control_task`，初始化见 `CodingDay_Init`。[当前工作区源码](../../AMR_NEW_CodingDay4/Core/Src/coding_day.c)

| 简写 | 文件 |
|---|---|
| C | [AMR_NEW_CodingDay4/Core/Src/coding_day.c](https://github.com/llikkm/guangsaitiyi/blob/8985608adaade380382851ad8fd32c06973d4ead/AMR_NEW_CodingDay4/Core/Src/coding_day.c) |
| M | [AMR_NEW_CodingDay4/Core/Src/main.c](https://github.com/llikkm/guangsaitiyi/blob/8985608adaade380382851ad8fd32c06973d4ead/AMR_NEW_CodingDay4/Core/Src/main.c) |
| MSP | [AMR_NEW_CodingDay4/Core/Src/stm32f4xx_hal_msp.c](https://github.com/llikkm/guangsaitiyi/blob/8985608adaade380382851ad8fd32c06973d4ead/AMR_NEW_CodingDay4/Core/Src/stm32f4xx_hal_msp.c) |
| IRQ | [AMR_NEW_CodingDay4/Core/Src/stm32f4xx_it.c](https://github.com/llikkm/guangsaitiyi/blob/8985608adaade380382851ad8fd32c06973d4ead/AMR_NEW_CodingDay4/Core/Src/stm32f4xx_it.c) |
| RT | [AMR_NEW_CodingDay4/Core/Inc/FreeRTOSConfig.h](https://github.com/llikkm/guangsaitiyi/blob/8985608adaade380382851ad8fd32c06973d4ead/AMR_NEW_CodingDay4/Core/Inc/FreeRTOSConfig.h) |
| IOC | [AMR_NEW_CodingDay4/AMR_NEW.ioc](https://github.com/llikkm/guangsaitiyi/blob/8985608adaade380382851ad8fd32c06973d4ead/AMR_NEW_CodingDay4/AMR_NEW.ioc) |
| PC | [AMR_NEW_CodingDay4/pc_monitor.py](https://github.com/llikkm/guangsaitiyi/blob/8985608adaade380382851ad8fd32c06973d4ead/AMR_NEW_CodingDay4/pc_monitor.py) |
| R | [AMR_NEW_CodingDay4/README-CodingDay4.md](https://github.com/llikkm/guangsaitiyi/blob/8985608adaade380382851ad8fd32c06973d4ead/AMR_NEW_CodingDay4/README-CodingDay4.md) |
| BOM | 工作区根目录 week4接线表(1).xlsx，Sheet1。它是接线表，器件参数以实物核对结果为准 |

状态用词：**已实现**表示源码有有效入口，**未实现**表示当前工程无相应处理，**待实测**表示输出、精度或硬件行为尚未测定，**推断**表示由协议、接线或时序推出的可能情况。原交付件含历史编译产物，重新编译与实车测试状态见第四份文档。
