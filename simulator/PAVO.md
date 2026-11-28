# PAVO 雷达模型

在仓库根目录运行 `python -m unittest discover -s simulator -p test_lidar_sensor.py` 可检查扫描时序、遮挡、安装偏移和误差。长度用 mm，时间用 s。配置中的角度用度，输出的角度用 rad。车头方向为零，向左为正。

## 四段采集给出的参数

用 `lidar_analysis/analyze_static.py` 的 `read_capture()` 解码四份原始 PCAP，去掉首尾不完整帧。每圈含 4500 个角度格，间隔 0.08°。原始距离非零计数每单位对应 2 mm。四段数据中，出现过有效读数的原始角度介于 45.12° 和 314.96°，正前方为 180°。

| 原文件 | 完整帧 | 圈周期中位数（ms） | 有效角区内空读数占比 |
| --- | ---: | ---: | ---: |
| static_test.pcap | 541 | 91.236 | 12.49% |
| movingTest.pcap | 357 | 99.253 | 11.83% |
| spinning_test.pcap | 572 | 99.686 | 12.37% |
| spinningTest2.pcap | 719 | 99.368 | 11.74% |

静态采集中，按已有分析的 1、2、3 m 棋盘稳定时段，取正前方单束，测距标准差依次为 2.14、2.26、2.15 mm。模型取 2.2 mm 作为距离噪声的起点。各段均有空读数，包含材料和房间几何造成的缺测。默认独立丢点率 1% 是实验假设。上述周期取相邻完整帧开始时刻之差。

## 两种扫描方式

| 配置 | 默认值及作用 |
| --- | --- |
| `lidar_model = "fast"` | 每秒 12 次整帧快照，每帧 91 束，覆盖 270°，量程 0 至 4000 mm。可改 `lidar_fast_hz/rays/fov_deg/min_mm/max_mm` 对应各项 |
| `lidar_model = "dense"` | 默认每秒转 10.06 圈，接近两次运动采集的中间值。每圈沿 270° 有效区域以 0.08° 间隔依次取 3375 束。从第一束到最后一束约 74.5 ms，再经过约四分之一圈的后方盲区。可改 `lidar_dense_hz/fov_deg/step_deg` |
| `lidar_min_mm / lidar_max_mm` | 高密度模式默认 30、10000 mm。采集中的有效值曾低至 22 mm、高至 10000 mm。两个门限均可按设备配置修改 |
| `lidar_height_mm` | 默认 55 mm。低于扫描面的物体不参与选取，最近的可见表面遮住其后方物体。低于最近量程的前景返回空值 |
| `lidar_mount_x_mm / lidar_mount_y_mm / lidar_mount_yaw_deg` | 相对车体原点的前移、左移和左转量，默认均为零 |
| `lidar_clutter` | 光学杂物列表，每项为世界坐标下的 [x, y, 半径, 高度]，按圆柱求交。场景中实际放置的障碍、对手车身和推板也参与按高度求交 |
| `lidar_blind_sectors_deg` | 额外固定盲区列表，每项为车载雷达坐标下的 [起角, 止角]，两端包含。默认空 |

`Ideal` 保留几何遮挡、可见高度和量程，关闭随机误差、额外偏差、量化和传输延迟。`Nominal` 使用配置的误差和固定延迟，默认 12 ms 延迟为假设。`Stress` 将噪声标准差、测距偏差、丢失概率、离群概率和延迟乘以 `lidar_stress_factor`（默认 3），概率到 1 饱和。量化步长默认 2 mm。角度扰动通过偏转实际射线产生，记录保留编码器角度。随机离群用量程内的均匀读数代替原回波。每台雷达使用调用方分配的独立随机数流。

误差参数全名为 `lidar_noise_mm`、`lidar_bias_mm`、`lidar_quantum_mm`、`lidar_angle_error_deg`、`lidar_dropout`、`lidar_frame_dropout`、`lidar_outlier`、`lidar_latency_s`。绝对偏差取零起步，棋盘的放置基准和雷达光心偏移尚待实测。

## 扫描时钟与适配

每个物理步采集该步内到期的束。高密度模式在相邻物理姿态之间插值，保留车辆转弯和行进引起的帧内畸变。其他运动物体使用当步物理空间中的位姿，时间误差最多约一个物理步。模拟器初始化时调用一次 `update(world,car,start_time)`，以确定插值起点。

`ScanFrame.beams` 保存雷达本地角度和距离，空回波用 `None`。`times` 保存各束采样时刻，`time` 对应最后一束，`available_time` 对应传输完成。消费者在随后的第一个物理步取得此帧。`pose` 仅为仿真评估提供末束时刻的车体真值。硬件适配器可以直接用同一帧格式，省略 `pose`。采集协议及抽样文件见 [房间采集分析](../lidar_analysis/README.md)。

Stress 默认另设 −45° 至 −35° 的固定遮挡扇区，并将整帧丢失概率至少设为 3%。这是用于测试的干扰条件。可将 `lidar_stress_blind_deg` 设为 []，将 `lidar_stress_frame_loss` 设为 0，分别关闭这两项。
