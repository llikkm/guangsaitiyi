# V2 控制器接口

V2.1 继续使用本接口。加载 V2.1 方案时，轮速环输出的 PWM 经电机电流、轮轴扭矩和轮胎接触传至车体；零 PWM 按该车的 brake/coast 参数处理。评估日志增加轮系状态，控制器仍从模拟传感器读取观测。[V2.1 动力学说明](V2.1手册.md)

每队提供一个 Python 文件，导出无参 `Controller` 类。相对路径从实验 TOML 所在目录解析。两队分别创建实例。可以复制 [motion_check.py](controllers/motion_check.py) 开始开发。

```python
class Controller:
    def reset(self, config):
        self.cruise = config.get("speed_mm_s", 180)

    def update(self, robot, dt):
        ranges = robot.read_lidar()
        if not ranges:
            return robot.stop()
        _, front_mm = ranges[len(ranges) // 2]
        if front_mm is not None and front_mm < 350:
            return robot.stop()
        return robot.set_wheel_speed(self.cruise, self.cruise)
```

在 TOML 的 `[team_a]` 中填写 `controller = "controllers/my_controller.py"`。该队的 `[team_a.options]` 会复制给 `reset(config)`。需要按较低频率计算导航目标时，可实现 `plan(robot, dt)`，在实例中保存目标，供 `update` 使用。新一轮加赛会再次调用 reset。有随机行为的控制器应在本队 options 中配置算法种子，并在 reset 时初始化自己的随机数生成器。

## 读写约定

控制器只收到自身的只读 Observation、本队 options 和仿真时间间隔。接口模块 [robot_api.py](robot_api.py) 可独立导入；场地真值和扫描采集时的真实位姿供界面及评估器使用。当前加载器在同一 Python 进程运行受信任的本地算法文件。

| 接口 | 值和单位 |
| --- | --- |
| `robot.time`、`dt` | 仿真秒。dt 为本任务距上次运行的时间，首次为配置周期 |
| `read_lidar()`、`lidar_time` | 最近一次扫描及其时间。每束为 (雷达本地角度 rad, 距离 mm)，无回波时距离为 None |
| `read_imu()`、`imu_time` | 模拟陀螺的偏航角速度 rad/s 及采样时间 |
| `read_encoders()`、`encoder_time` | 左、右轮累计计数及采样时间 |
| `estimated_pose` | 由已知起点、模拟编码器和陀螺推算的 (x mm, y mm, 航向 rad) |
| `return robot.set_pwm(left, right)` | 两轮归一化 PWM，范围 [-1, 1]，正号为前进。也可直接返回 `MotorCommand(left_pwm=..., right_pwm=...)` |
| `return robot.set_wheel_speed(left, right)` | 两轮目标 mm/s。轮速环将目标与编码器测速比较，产生 PWM |
| `return robot.stop()` | 两侧零 PWM，车辆按动力学减速 |

V2.0 使用简化的 PWM 执行器。占空比映射到空载目标轮速，轮速受加速度限制；轮地作用力再推动刚体。轮速环含前馈、PID 及积分限幅处理。参数沿用 [config.toml](config.toml) 的来源分区，V2 新增的增益和频率在 [v2.toml](v2.toml) 的 assumed 中。实车电机参数尚待标定。

## V2.1 雷达定位接口

在 `plan(robot, dt)` 或 `update(robot, dt)` 中读取 `robot.wall_fix`。每辆车在新扫描到达时更新一次结果。默认控制器以 100 Hz 运行，雷达以 12 Hz 采样。若要累计统计或更新滤波器，按 `fix.time` 去重，并在控制器 reset 时清除上次处理的时间。

| 字段 | 约定 |
| --- | --- |
| `time` | 扫描时刻，单位 s，与同一观测的 `robot.lidar_time` 一致 |
| `status` | `ok` 表示本帧通过局部检查。其余值为 `too_few_returns`、`missing_wall_direction`、`short_wall_segments`、`poor_match`、`ambiguous_heading` 或 `no_convergence` |
| `pose` | 有效时为 (x mm, y mm, 航向 rad)，对应车体中心，失效时为 None |
| `wall_distances` | 有效时为 (西, 东, 南, 北) 四面墙到车体中心的垂直距离，单位 mm，失效时为 None |
| `support` | 参与墙面匹配的点数占抽样有效点数的比例，范围 0～1 |
| `rms_mm` | 匹配点到墙线的均方根残差，单位 mm，无法计算时为 None |

下面的例子可放在控制器的 update 中。雷达有效时记录扫描时刻的位姿，缺少有效解时使用当前里程计位姿，并清空本帧墙距。

~~~python
fix = robot.wall_fix
if fix is not None and fix.status == "ok":
    self.pose = fix.pose
    self.wall_distances = fix.wall_distances
else:
    self.pose = robot.estimated_pose
    self.wall_distances = None
~~~

实车适配时，把同一时刻的里程计预测与整帧扫描传给 `RectangleLocator.locate`。仿真扫描为 (车体系角度 rad, 距离 mm)，PAVO 原始包角度和以米解码的距离先经 `pavo_scan` 转换。雷达偏离车体中心时填写安装偏移。[安装坐标与实车示例](localization/README.md)

## 仿真时钟

默认物理和电机各 200 Hz，编码器、IMU、轮速 PID 和控制器 update 各 100 Hz，导航 plan 为 20 Hz，雷达为 12 Hz，窗口上限 60 FPS。可分别配置 `rate`、`motor_hz`、`encoder_hz`、`imu_hz`、`pid_hz`、`control_hz`、`navigation_hz`、`scan_rate` 和 `display.fps`。物理频率用正整数，各任务频率可为小数，上限为物理频率。

在物理步起点，核心使用最近的传感器样本依次运行到期的 plan、update 和轮速 PID，然后按电机周期更新轮速并施力。物理步结束后按时采样，再更新估计位姿。相邻采样之间保留上一份读数。非整除频率按累计仿真步编号调度；200 Hz 下前 3 次周期性雷达采样落在第 17、34、50 步。启动时另有一帧初始扫描。

## 自定义实验场景

在实验 TOML 中加入以下部分。坐标原点在场地左下，角度从 +x 逆时针计。

```toml
[scene]
poses = [[400, 400, 0], [2400, 1500, 3.14159]]
pieces = [[1, 6, 650, 400]]
obstacles = [[900, 400, 100, 160, 180]]
```

`poses` 依次给出两车的 [x, y, 航向]。`pieces` 的行是 [阵营, 棋种, x, y]，阵营 0/1 为甲/乙，棋种 0 至 6 依次为帅将、仕士、相象、马、车、炮、兵卒。空列表创建无棋实验，省略则按种子布置比赛棋子。自定义棋位用于常规局，加赛按赛题重新布棋。`obstacles` 的行是 [中心 x, 中心 y, 沿 x 宽度, 沿 y 深度, 高度]，单位均为 mm。实体障碍参与碰撞，雷达按扫描高度决定可见性。初始几何须位于场内并避免重叠。

## 日志与复现

`--record` 输出格式 3 的 JSONL。首行保存软件及 Pymunk 版本、有效配置、实验种子、场景和两份控制器的 SHA-256。每个物理步保存步号、仿真秒、两队的原始命令模式和数值，以及实际 PWM。遥测包含车辆真值、估计和棋位，末行记录结束步号。

`--replay` 读取记录的命令，在同一版本核心中重新计算轮速环并逐步核对 PWM。回放可脱离原控制器文件运行。离线位置 RMSE 可用 `experiment.py` 计算。硬件适配时须实现同样的单位和时间戳约定；本版交付仿真端接口。

## V2.2 原始传感器

`robot.lidar_sample` 提供逐束采样时刻，`robot.imu_sample` 提供传感器坐标系六轴 SI 读数，`robot.encoder_sample` 提供 TIM3/TIM8 寄存器值。第一次交付之前三个字段各自可为 None。控制器只接收已经可用的样本。[接口、单位和实车适配](V2.2手册.md)
