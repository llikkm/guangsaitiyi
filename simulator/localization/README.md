# 矩形场地雷达定位

本模块估计小车车体中心在已知矩形场地中的位置和航向，并计算该中心到西、东、南、北围挡内表面的垂直距离。V2.1 工作台按 F5 手动驾驶，右侧面板的 Lidar xy、Lidar yaw 和 Wall W/E、S/N 随每次扫描更新。Tab 切换所观察的车。每车的控制器可以独立读取结果。

## 启动和查看结果

在项目根目录运行：

~~~powershell
py -3.12 simulator/studio.py
~~~

算法实验和回放沿用 [V2.1 手册](../V2.1手册.md)。无窗口记录可运行以下命令，文件名选一个尚未使用的名称。

~~~powershell
py -3.12 simulator/sil.py --seconds 4 --record simulator/runs/wall-demo.jsonl
~~~

遥测中每辆车的 wall_fix 保存该时刻最近一帧的解。按其 time 去重。评估时用同一扫描对应的 scan_pose 作仿真真值。原有 estimate 和控制器 estimated_pose 都表示编码器与陀螺推算结果。当前导航按其原有方式工作，开发者可以在自己的控制器中读取雷达解。

## 控制器和实车共用的接口

在控制器的 update 或 plan 中读取 robot.wall_fix，如下所示。

~~~python
fix = robot.wall_fix
if fix is not None and fix.status == "ok":
    x_mm, y_mm, heading_rad = fix.pose
    west_mm, east_mm, south_mm, north_mm = fix.wall_distances
~~~

场地原点在左下，x 向右、y 向上，航向从 +x 逆时针计。车体系 x 向前、y 向左。墙距按场地命名，即使车掉头，西墙仍是同一面墙。四个值由同一个估计位置计算，可包含当前视野外的墙。控制避碰时还需计入朝向和车身、推板的外形。

| WallFix 字段 | 含义 |
| --- | --- |
| time | 扫描采样时刻，秒。控制周期可能多次读到同一帧 |
| pose | (x mm, y mm, 航向 rad)，失效时为 None |
| wall_distances | (西, 东, 南, 北)，单位 mm，失效时为 None |
| support | 被匹配到墙面的点数占抽样有效回波的比例 |
| rms_mm | 匹配点到墙线的均方根残差，无法计算时为 None |
| status | ok 表示通过本帧检查。其余状态见下表 |

[核心代码](../wall_localization.py) 只依赖 Python 标准库。实车上创建一个 RectangleLocator 实例，输入场地内尺寸和雷达相对车体中心的安装位置。下面的 mount 数值仅用于演示。

~~~python
from wall_localization import RectangleLocator, pavo_scan

locator = RectangleLocator(3000, 2000, mount=(75, 0, 0))
# raw_beams 由驱动按一整圈组帧，每束为 (原始包角度 degree, 已解码距离 m)
beams = pavo_scan(raw_beams)
fix = locator.locate(beams, scan_time_s, pose_at_scan)
~~~

mount 按前向偏移 mm、左向偏移 mm、安装偏航 rad 填写。仿真雷达位于车体中心，使用 (0, 0, 0)。PAVO 转换采用本项目录像核对的包角度，180° 向前、90° 向右，零距离转为无回波。仿真的 read_lidar() 已经是 (车体系角度 rad, 距离 mm 或 None)，可直接传给 locate。实车驱动负责组帧、换算协议量程单位、统一传感器时钟，并在 scan_time_s 取得里程计预测 pose_at_scan。第一帧可用已知发车位姿。雷达装车后应核对正前方目标和右侧墙的方位。当前交付到扫描接口，尚未联调车载网口。

## 如何从点云算位置

1. 对有效光束做安装坐标变换，每帧最多保留 360 束。用采样时刻的位姿预测把这些点投到场地图上。预测帮助区分矩形场地中相差 180° 的两种朝向。
2. 在围挡附近，为每个点选最近的墙线。远离墙的回波会被排除，残差较大的点权重较低，减少对车和障碍物的影响。
3. 同时调整车中心 x、y 和航向，使匹配点到墙线的加权距离减小。西墙和东墙约束 x，南墙和北墙约束 y，沿墙展开的一段点云约束航向。
4. 检查约束和残差。有效时输出位姿，以及 x、场宽减 x、y、场高减 y。无效时清空本帧位姿和墙距，控制器仍可从 estimated_pose 获取里程计估计。

初步单因素试验中，位置偏差至 150 mm 或航向偏差至 0.30 rad 时仍可收敛；200 mm 或 0.40 rad 已有失败样本。这些值随姿态、遮挡和偏差组合而变。12 Hz 扫描名义间隔约 83 ms，200 Hz 物理时钟上的采集间隔为 80 或 85 ms。控制器可用 robot.time 减 fix.time 判断实际数据年龄。

这是围绕当前预测的局部求解。错误发车区、大幅航向偏差或长期里程计漂移可能造成失效或对称误匹配。support 和 rms_mm 适合诊断匹配质量，无法独立排除对称解。本版对一帧内各束使用同一位姿，高速转动时的运动畸变尚待实车验证。

## 维护时从哪里看

| 文件 | 职责 |
| --- | --- |
| wall_localization.py | 单帧求解、PAVO 扫描转换、结果结构 |
| sil_core.py 中 Plant.refresh_scans | 两车各自在新扫描到达时计算一次，扫描时刻取里程计预测 |
| robot_api.py、sil_view.py、experiment.py | 控制器读取、驾驶窗口显示、遥测记录 |
| test_wall_localization.py | 正常控制器入口、已知先验偏差、障碍物、非中心安装及缺失墙面回归测试 |

| 状态 | 排查方向 |
| --- | --- |
| too_few_returns | 有效点不足 12 个，检查量程和无回波处理 |
| missing_wall_direction | 两个坐标方向分别需要至少 5 个匹配点，检查遮挡、地图或先验 |
| short_wall_segments | 有效墙段展开不足，航向难以求稳 |
| poor_match | 最终支持率低于 55%、残差 RMS 超过 35 mm 、位置落在场外，或解与先验相差超过 300 mm / 0.35 rad |
| ambiguous_heading | 候选解及其 180° 镜像都落在先验位置 300 mm 范围内，本帧拒绝输出绝对位姿。中心点即使先验正确也可能返回该状态；需要独立航向或不对称地标才能从几何上判别 |
| no_convergence | 12 轮内没有收敛，核对坐标方向、安装偏移和初始化 |

算法在前四轮使用 160 mm 的墙面匹配距离，其后收紧至 65 mm。稳健权重的转折点为 15 mm。这些数值是第一版工程参数。改动前先保留对照结果，同时比较静止误差、遮挡时的失效率和错误接受率。先排查单位、时间对齐和安装位置，再考虑放宽阈值。

## 复核与交接

~~~powershell
py -3.12 -m unittest discover -s simulator -p test_wall_localization.py -v
py -3.12 -m unittest discover -s simulator -v
~~~

本次 4 秒默认双车实验共检查 96 个新扫描结果，96 个有效。对照同一帧仿真真值，最大位置偏差 0.016 mm 以内，最大航向偏差 0.0007° 以内。仿真使用理想几何和无测距噪声的射线。带障碍物的另一个测试主动给初始位置加入 (+65, −55) mm、航向加入 0.045 rad，并配置非中心安装，用来检查算法确实能修正预测。

采样脚本、四段数据的哈希和可独立重跑的样本见[离线复核](../../lidar_analysis/README.md)。四段真实房间 PCAP 用于检查接口与失效处理。每 30 个完整帧取一帧，共 75 帧，统一给定测试先验 (1500,1000,0) 和比赛地图，均返回无效。房间与比赛地图尺寸不同。实车绝对定位精度需要在尺寸已知的矩形环境中，独立量出车中心位置与航向后评价。优先采发车区、场地中央和近墙转向三类位置，并保留被对手遮挡的帧。
