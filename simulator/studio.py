"""V2.1 experiment workbench. Launch from any directory with system Python."""
import copy
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from sil_core import SENSOR_REFERENCE, Plant, settings
from dynamics import vehicle_config
from imu_sensor import GYRO_SENSITIVITY


ROOT = Path(__file__).resolve().parent
LABELS = {
    "mass":"底盘含载荷质量 kg（不含另计推板）","length":"车长 mm","breadth":"车宽 mm",
    "car_height":"车高 mm","cg_x":"底盘重心向前 mm","cg_y":"底盘重心向左 mm",
    "inertia_si":"底盘惯量 kg·m²（0 自动）","track":"轮距 mm","wheel":"轮径 mm",
    "counts":"每轮编码器计数","battery_v":"电池端电压 V","zero_mode":"零 PWM 模式",
    "spec_mode":"电机参数输入方式","rated_v":"额定工作点电压 V","rated_a":"额定工作点电流 A",
    "rated_rpm":"额定电机轴转速 rpm","rated_nm":"额定电机轴输出扭矩 N·m",
    "bumper":"推板形状","bumper_mass":"推板附加质量 kg","bumper_height":"推板高度 mm",
    "bumper_depth":"推板前伸 mm","bumper_width":"推板开口宽 mm","bumper_radius":"板厚的一半 mm",
    "custom_outline":"自定义折线 [[x,y],…] mm","resistance":"电枢电阻 Ω","ke":"反电动势 V·s/rad",
    "kt":"转矩常数 N·m/A","gear":"减速比","efficiency":"传动效率","current_limit":"驱动限流 A",
    "stall_nm":"电机轴堵转扭矩上限 N·m","deadband":"PWM 死区","wheel_inertia":"轮侧等效惯量 kg·m²",
    "tire_mu":"轮胎附着系数","rolling_mu":"滚动阻力系数","long_stiffness":"纵向滑移刚度 N·s/m",
    "side_stiffness":"侧向滑移刚度 N·s/m","piece_mass":"单枚棋子质量 kg","piece_mu":"棋子滑动摩擦系数",
    "contact_friction":"实体表面接触摩擦","elasticity":"实体弹性系数","rate":"物理频率 Hz",
    "motor_hz":"驱动指令锁存 Hz","fps":"显示帧率上限","seed":"布棋种子","seconds":"实验时长 s",
    "speed_factor":"仿真倍率","scene":"实验场景 JSON","controller":"控制器文件","options":"算法参数 JSON"}
LABELS.update({"mode":"传感器误差模式","lidar_model":"雷达扫描方式","imu_hz":"IMU 采样 Hz",
               "encoder_hz":"编码器采样 Hz","lidar_dense_step_deg":"密集模式角间隔 °",
               "lidar_dense_hz":"密集模式扫描 Hz","lidar_fast_rays":"快速模式光束数",
               "lidar_mount_x_mm":"雷达前向偏移 mm","lidar_mount_y_mm":"雷达左向偏移 mm",
               "lidar_mount_yaw_deg":"雷达安装偏航 °","latency_s":"传输延迟 s"})
LABELS.update({
    "lidar_fast_fov_deg":"快速扫描视场 °", "lidar_fast_hz":"快速扫描频率 Hz",
    "lidar_fast_min_mm":"快速模式最近量程 mm", "lidar_fast_max_mm":"快速模式最远量程 mm",
    "lidar_dense_fov_deg":"密集扫描视场 °", "lidar_min_mm":"密集模式最近量程 mm",
    "lidar_max_mm":"密集模式最远量程 mm", "lidar_height_mm":"雷达离地高度 mm",
    "lidar_noise_mm":"测距标准差 mm", "lidar_bias_mm":"测距固定偏差 mm",
    "lidar_quantum_mm":"距离量化步长 mm", "lidar_angle_error_deg":"角度标准差 °",
    "lidar_dropout":"单束丢点概率", "lidar_frame_dropout":"整帧丢失概率",
    "lidar_outlier":"异常距离概率", "lidar_latency_s":"雷达传输延迟 s",
    "lidar_blind_sectors_deg":"盲区列表 [[起角, 止角], …] °",
    "lidar_clutter":"遮挡圆柱 [[x, y, 半径, 高度], …] mm",
    "lidar_stress_blind_deg":"Stress 额外盲区 °", "lidar_stress_frame_loss":"Stress 最低整帧丢失概率",
    "lidar_stress_factor":"雷达干扰倍率", "stress_factor":"IMU 干扰倍率",
    "stress_latency_s":"Stress 编码器延迟 s", "mount_rpy_deg":"安装滚转、俯仰、偏航 °",
    "gyro_bias_dps":"三轴陀螺零偏 °/s", "accel_bias_g":"三轴加速度零偏 g",
    "gyro_noise_dps":"三轴陀螺标准差 °/s", "accel_noise_g":"三轴加速度标准差 g",
    "gyro_walk_dps_sqrt_s":"陀螺零偏随机游走 (°/s)/√s", "accel_walk_g_sqrt_s":"加速度零偏随机游走 g/√s",
    "gyro_range_dps":"陀螺量程 ±°/s", "accel_range_g":"加速度量程 ±g",
    "dps_per_count":"陀螺量化步长 (°/s)/计数", "g_per_count":"加速度量化步长 g/计数",
    "gyro_scale":"三轴陀螺比例系数", "accel_scale":"三轴加速度比例系数"})
CHOICES = {"mode":("Ideal","Nominal","Stress"),"lidar_model":("fast","dense"),"zero_mode":("brake","coast"),"bumper":("flat","u","custom"),"spec_mode":("constants","rated")}
CHOICES.update(gyro_range_dps=tuple(GYRO_SENSITIVITY), motion_compensation=("odometry", "off"))
LABELS["motion_compensation"] = "密扫逐束运动补偿"
SURFACES = (("普通",0.72,0.105,0.012),("低附着",0.08,0.05,0.008),("粗糙",0.9,0.35,0.025))
SOURCES = ("假设","理论","厂商标称","实测")
VEHICLE = ("mass","length","breadth","car_height","cg_x","cg_y","inertia_si","track","wheel","counts",
           "battery_v","zero_mode","tire_mu","rolling_mu","long_stiffness","side_stiffness","bumper",
           "bumper_mass","bumper_height","bumper_depth","bumper_width","bumper_radius","custom_outline")


def encode(value):
    return value if isinstance(value,str) else json.dumps(value,ensure_ascii=False)


def validate_profile(o):
    c = o["config"]
    if "sensors" in o:
        imu = o["sensors"]["imu"]
        if imu["gyro_range_dps"] > 32768 * imu["dps_per_count"]:
            raise ValueError("陀螺量程超出 16 位计数可表示范围，请核对量化步长")
    if len(o["teams"]) != 2: raise ValueError("方案需要两辆车")
    if type(o["fps"]) is not int or o["fps"] < 1: raise ValueError("帧率须为正整数")
    if min(c[k] for k in ("piece_mass","range_mm","speed","gravity_mm_s2")) <= 0 or c["rays"] < 2:
        raise ValueError("请检查质量、速度和雷达参数")
    if min(c[k] for k in ("piece_mu","contact_friction","elasticity","laser_height")) < 0:
        raise ValueError("摩擦、弹性系数和雷达高度须非负")
    run = o.get("run",{"seed":42,"seconds":120,"speed_factor":1.})
    if min(run["seconds"],run["speed_factor"]) <= 0: raise ValueError("时长和倍率须大于零")
    vehicles = [vehicle_config(c,team,o["dynamics"],i) for i,team in enumerate(o["teams"])]
    Plant(c,run["seed"],o["scene"],vehicles,o.get("sensors"))


class Workbench:
    def __init__(self,root):
        self.root = root
        root.title("光电象棋 V2.2 · 传感器与动力学实验台")
        root.geometry("940x710")
        root.minsize(800,540)
        self.note = tk.StringVar(value="修改参数后开始实验；每次运行独立保存配置和记录。")
        ttk.Label(root,textvariable=self.note,padding=8).pack(fill="x")
        self.book = ttk.Notebook(root)
        self.book.pack(fill="both",expand=True,padx=10)
        self.book.enable_traversal()
        bar = ttk.Frame(root,padding=8)
        bar.pack(fill="x")
        for name,key,callback in (("切换地面","F4",self.surface),("手动驾驶","F5",lambda:self.launch(True)),("算法实验","F6",lambda:self.launch(False)),
                                  ("保存方案","F7",self.save),("载入方案","F8",self.load),("回放日志","F9",self.replay)):
            ttk.Button(bar,text=f"{name} [{key}]",command=callback).pack(side="left",padx=3)
            root.bind(f"<{key}>",lambda e,fn=callback:fn())
        self.populate(settings(SENSOR_REFERENCE))

    def surface(self):
        self.surface_index = (self.surface_index+1)%len(SURFACES)
        name,tire,piece,roll = SURFACES[self.surface_index]
        changes = {"piece_mu":piece,"tire_mu":tire,"rolling_mu":roll}
        for path,_,var,src in self.fields:
            if path[-1] in changes:
                var.set(str(changes[path[-1]]))
                src.set("假设")
        self.note.set(f"地面预设：{name}。两车轮胎 μ={tire}，棋子 μ={piece}，滚阻系数={roll}。")

    def page(self,name):
        frame = ttk.Frame(self.book)
        self.book.add(frame,text=name)
        canvas = tk.Canvas(frame,highlightthickness=0)
        scroll = ttk.Scrollbar(frame,command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right",fill="y")
        canvas.pack(fill="both",expand=True)
        inner = ttk.Frame(canvas,padding=10)
        window = canvas.create_window(0,0,anchor="nw",window=inner)
        inner.bind("<Configure>",lambda e:canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",lambda e:canvas.itemconfigure(window,width=e.width))
        inner.columnconfigure(1,weight=1)
        return inner

    def field(self,page,path,key,value,source=True):
        row = page.grid_size()[1]
        ttk.Label(page,text=LABELS.get(key,key)).grid(row=row,column=0,sticky="w",pady=3,padx=4)
        var = tk.StringVar(value=encode(value))
        if key in CHOICES:
            entry = ttk.Combobox(page,textvariable=var,values=CHOICES[key],state="readonly")
        else:
            entry = ttk.Entry(page,textvariable=var)
        entry.grid(row=row,column=1,sticky="ew",padx=8)
        tag = ".".join(map(str,path))
        src = tk.StringVar(value=self.base.get("sources",{}).get(tag,"假设"))
        if source:
            ttk.Combobox(page,textvariable=src,values=SOURCES,state="readonly",width=9).grid(row=row,column=2)
        self.fields.append((path,value,var,src if source else None))
        return var

    def populate(self,options):
        for tab in self.book.tabs(): self.root.nametowidget(tab).destroy()
        self.base = copy.deepcopy(options)
        if "sensors" in self.base:
            self.base["sensors"]["lidar"].setdefault("motion_compensation", "off")
        self.fields = []
        self.surface_index = 0
        run = options.get("run",{"seed":42,"seconds":120,"speed_factor":1.0})
        self.base["run"] = dict(run)
        page = self.page("实验与场地")
        for key,value in run.items(): self.field(page,("run",key),key,value,False)
        self.field(page,("fps",),"fps",options["fps"],False)
        self.field(page,("scene",),"scene",options["scene"],False)
        for key in ("piece_mass","piece_mu","contact_friction","elasticity","rate","motor_hz","pid_hz",
                    "encoder_hz","imu_hz","control_hz","navigation_hz","scan_rate","speed_kp","speed_ki","speed_kd",
                    "fov","range_mm","rays","laser_height","gyro_bias","speed","arrival","approach_gain","turn_gain"):
            if "sensors" in options and key in ("fov","range_mm","rays","laser_height","gyro_bias","scan_rate"):
                continue
            self.field(page,("config",key),key,options["config"][key])
        if "sensors" in options:
            sensors = self.base["sensors"]
            page = self.page("传感器")
            self.field(page,("sensors","mode"),"mode",sensors["mode"],False)
            for group, title in (("lidar","PAVO 雷达"),("imu","LSM6DS3TR-C"),("encoder","TIM3 / TIM8")):
                ttk.Label(page,text=title).grid(row=page.grid_size()[1],column=0,sticky="w",pady=8)
                variables = {}
                for key,value in sensors[group].items():
                    variables[key] = self.field(page,("sensors",group,key),key,value,
                                                key not in ("lidar_model", "motion_compensation"))
                if group == "imu":
                    scale, step = variables["gyro_range_dps"], variables["dps_per_count"]
                    scale.trace_add("write", lambda *_: step.set(GYRO_SENSITIVITY[float(scale.get())]))
        for i,team in enumerate(options["teams"]):
            name = "甲乙"[i]
            v,motors = vehicle_config(options["config"],team,options["dynamics"],i)
            page = self.page(name+"车")
            self.field(page,("teams",i,"controller"),"controller",team["controller"],False)
            self.field(page,("teams",i,"options"),"options",team.get("options",{}),False)
            for key in VEHICLE: self.field(page,("teams",i,"vehicle",key),key,v[key])
            for side,motor in zip(("left","right"),motors):
                page = self.page(name+("左" if side=="left" else "右")+"电机")
                for key,value in motor.items(): self.field(page,("teams",i,side,key),key,value)

    def collect(self):
        o = copy.deepcopy(self.base)
        for path,base,var,src in self.fields:
            try:
                text = var.get()
                if isinstance(base,(list,tuple,dict)): value = json.loads(text)
                elif isinstance(base,str): value = text
                elif path[0] == "sensors" and type(base) is int: value = int(text)
                elif path[-1] in ("seed","rate","fps","counts","rays"): value = int(text)
                else: value = float(text)
                if isinstance(value,float) and not math.isfinite(value): raise ValueError("finite value required")
            except (TypeError,ValueError) as exc:
                raise ValueError(f"{LABELS.get(path[-1],path[-1])}：{exc}") from exc
            dest = o
            for k in path[:-1]:
                dest = dest[k] if isinstance(dest,list) else dest.setdefault(k,{})
            dest[path[-1]] = value
            if src: o.setdefault("sources",{})[".".join(map(str,path))] = src.get()
        validate_profile(o)
        return o

    def save(self):
        try:
            o = self.collect()
            path = filedialog.asksaveasfilename(defaultextension=".json",filetypes=[("实验方案","*.json")])
            if path: Path(path).write_text(json.dumps(o,ensure_ascii=False,indent=2),encoding="utf8")
        except (ValueError,OSError,TypeError,KeyError) as exc: messagebox.showerror("保存方案",str(exc))

    def load(self):
        path = filedialog.askopenfilename(filetypes=[("实验方案","*.json *.toml")])
        if not path: return
        try:
            o = settings(path) if path.endswith(".toml") else json.loads(Path(path).read_text(encoding="utf8"))
            if "dynamics" not in o: raise ValueError("请选择 V2.1 方案")
            validate_profile(o)
            self.populate(o)
        except (ValueError,OSError,TypeError,KeyError) as exc: messagebox.showerror("载入方案",str(exc))

    def spawn(self,args):
        process = subprocess.Popen([sys.executable,str(ROOT/"sil.py"),*map(str,args)],cwd=ROOT)
        def poll():
            if process.poll() is None: self.root.after(500,poll)
            elif process.returncode: self.note.set(f"实验进程退出，错误码 {process.returncode}。请检查参数和控制器。")
        self.root.after(500,poll)

    def launch(self,manual):
        try:
            o = self.collect()
            folder = ROOT/"runs"/datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            folder.mkdir(parents=True)
            profile = folder/"profile.json"
            profile.write_text(json.dumps(o,ensure_ascii=False,indent=2),encoding="utf8")
            r = o["run"]
            args = ["--profile",profile,"--gui","--record",folder/"run.jsonl","--seconds",r["seconds"],
                    "--speed",r["speed_factor"],"--seed",r["seed"]]
            if manual: args.append("--manual")
            self.spawn(args)
            self.note.set(f"本次实验记录：{folder.name}/run.jsonl")
        except (OSError,ValueError,TypeError,KeyError) as exc: messagebox.showerror("启动实验",str(exc))

    def replay(self):
        path = filedialog.askopenfilename(initialdir=ROOT/"runs",filetypes=[("实验日志","*.jsonl")])
        if path: self.spawn(["--gui","--replay",path])


if __name__ == "__main__":
    root = tk.Tk()
    Workbench(root)
    root.mainloop()
