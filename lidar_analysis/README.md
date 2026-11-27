# 房间雷达采集分析

在项目根目录执行以下命令。抽样复核只需 NumPy；读取原始 PCAP 的现有分析模块还依赖 Matplotlib，运动分析另外使用 SciPy。

```powershell
py -3.12 lidar_analysis/check_wall_fixes.py
py -3.12 lidar_analysis/check_wall_fixes.py lidar_analysis/samples/static_excerpt.pcap
```

第一条命令读取随仓库分发的 `samples/room_scans.npz`，包含四段房间采集的 75 个抽样帧。每帧保存原始量程计数（2 mm/计数）、相对时间、源文件名和帧号。比赛地图取 3000×2000 mm，每帧使用相同先验 (1500,1000,0)。结果为缺少墙向 69 帧、有效墙段短 2 帧、匹配质量不足 3 帧、迭代未收敛 1 帧。第二条命令从静态采集前 750 个数据包的节选验证解码入口，节选保留测距数据，网络地址已替换，主机时间从零开始。

拥有四个完整原文件时，运行下式可重新生成样本。解码器按记录中的角度回绕组帧，舍去首尾帧，然后从第一完整帧起每隔 30 帧取样。结论只检验房间点云面对比赛地图的失效处理。

```powershell
py -3.12 lidar_analysis/check_wall_fixes.py static_test.pcap movingTest.pcap spinning_test.pcap spinningTest2.pcap --export room_scans.npz
py -3.12 lidar_analysis/analyze_static.py static_test.pcap
py -3.12 lidar_analysis/analyze_motion.py movingTest.pcap spinning_test.pcap spinningTest2.pcap
```

静态分析的棋盘前方目标距离及房间边线统计见 `results/`；相对首帧的运动配准轨迹见 `motion_results/`。

| 原文件 | SHA-256 |
| --- | --- |
| static_test.pcap | 84ff3338e2c5e414b2254fa7974353673d90ebd6a3f1f88a53e44c23eed308e1 |
| movingTest.pcap | e139ee7e80f409c0c74595c4e5a8989433170fb220c308970b87ca4b19322223 |
| spinning_test.pcap | 317300c15b6da50c5bf9407a785ce277bbb256bf93fb55a41db1fe5d8d88a78b |
| spinningTest2.pcap | fb08a32d9609b0b6eef7d4595a6a67e071beed288999d540d0d5cfab8e3a9ddd |
