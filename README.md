# 光电象棋智能车

当前交付为二维仿真平台和 STM32 底盘控制基线。棋子识别和自主对弈策略待开发。[雷达采集分析](lidar_analysis/README.md) · [接线与调试入口](开发基本调线.md)

[二维仿真运行说明](simulator/README.md) · [仿真测试记录](simulator/TESTING.md) · [STM32 小车工程说明](AMR_NEW_CodingDay4/README-CodingDay4.md)

首次使用可运行 `py -3.12 simulator/studio.py`，在同一个工作台调车、手动驾驶或比较两套算法。[V2.1 操作与动力学说明书](simulator/V2.1手册.md) · [控制器接口](simulator/CONTROLLERS.md)

## 仿真开发基准

常用实验参数集中在图形工作台，保留手动驾驶。每个操作按钮在界面上显示对应快捷键。每次物理升级同步更新操作说明、参数单位和标准实验结果。

## 提交规则

每次完成项目更改后，自动创建 Git 提交并推送到对应分支。提交说明应写清本次改动的内容。

每次提交后，自动推送到 GitHub 仓库 `llikkm/guangsaitiyi` 的对应分支。当前主分支为 `main`。
