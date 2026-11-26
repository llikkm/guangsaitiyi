"""Week 4 蓝牙串口监视器。python pc_monitor.py COM7"""
import csv
import sys
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path

import serial


def monitor(port):
    link = serial.Serial(port, 115200, timeout=0, write_timeout=1)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    log = Path(f"amr-{stamp}.csv").open("w", newline="", encoding="utf-8")
    writer = csv.writer(log)
    root = tk.Tk()
    root.title(f"AMR Week 4 · {port}")
    status = tk.StringVar(value="等待蓝牙遥测")
    tk.Label(root, textvariable=status).pack()
    canvas = tk.Canvas(root, width=510, height=510, background="white")
    canvas.pack()
    goal = tk.StringVar(value="(0.00,1.40)")
    tk.Entry(root, textvariable=goal).pack()
    history = deque(maxlen=1500)
    pending = bytearray()
    trail = None
    previous_tick = None
    task_status = tk.StringVar()
    tk.Label(root, textvariable=task_status).pack()
    names = ("Control", "IMU", "LiDAR", "Output", "OLED", "BT TX", "BT RX", "Planner")

    def send(value):
        try:
            link.write((value + "\n").encode("ascii"))
        except (serial.SerialException, UnicodeError) as error:
            status.set(str(error))
            return
        writer.writerow(["COMMAND", value])
        log.flush()
        if value != "STOP":
            history.clear()

    buttons = tk.Frame(root)
    buttons.pack()
    for target in ("(0.00,1.40)", "(-0.70,0.70)", "(0.70,-0.70)", "(0.70,0.70)"):
        tk.Button(buttons, text=target, command=lambda target=target: send(target)).pack(side="left")
    tk.Button(buttons, text="发送", command=lambda: send(goal.get())).pack(side="left")
    tk.Button(buttons, text="STOP", command=lambda: send("STOP")).pack(side="left")

    def update():
        nonlocal trail, previous_tick
        try:
            pending.extend(link.read(min(link.in_waiting, 4096)))
        except serial.SerialException as error:
            status.set(str(error))
            return
        while b"\n" in pending:
            line, _, tail = pending.partition(b"\n")
            pending[:] = tail
            fields = line.decode("ascii", errors="replace").strip().split(",")
            try:
                if fields[0] == "POSE" and len(fields) == 11:
                    tick, x, y, heading, state, seq, points, mm, drops, result_drops = map(int, fields[1:])
                    if previous_tick is not None and tick < previous_tick:
                        history.clear()
                    previous_tick = tick
                    history.append((255 + x * .15, 255 - y * .15))
                    status.set(f"X {x} mm  Y {y} mm  H {heading / 10:.1f}°  状态 {state}  "
                               f"雷达块 {seq}  点数 {points}  最近 {mm} mm  丢块 {drops}/{result_drops}")
                elif fields[0] == "TIME" and len(fields) == 11:
                    task, period, runtime, runs, overrun, drops, rx, imu, oled, lidar = map(int, fields[1:])
                    task_status.set(f"{names[task]} · 周期 {period} µs · 执行 {runtime} µs · "
                                    f"次数 {runs} · IMU 超期 {overrun} · 错误 {drops}/{rx}/{imu}/{oled}/{lidar}")
                elif fields[0] != "LIDAR":
                    continue
                writer.writerow(fields)
            except (ValueError, IndexError):
                continue
        if len(pending) > 4096:
            pending.clear()
        if trail is not None:
            canvas.delete(trail)
        if len(history) >= 2:
            trail = canvas.create_line([value for point in history for value in point], fill="#1672c4", width=2)
        log.flush()
        root.after(40, update)

    canvas.create_line(0, 255, 510, 255, fill="#c8c8c8")
    canvas.create_line(255, 0, 255, 510, fill="#c8c8c8")
    for x, y in ((0, 1400), (-700, 700), (700, -700), (700, 700)):
        cx, cy = 255 + x * .15, 255 - y * .15
        canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#d66b19", outline="")
    root.after(40, update)
    try:
        root.mainloop()
    finally:
        link.close()
        log.close()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("用法: python pc_monitor.py COM7")
    monitor(sys.argv[1])
