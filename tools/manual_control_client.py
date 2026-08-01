#!/usr/bin/env python3
"""First-person remote takeover console for the construction zone."""

import argparse
import os
import queue
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk


class TakeoverConsole:
    def __init__(self, host, port, token):
        self.send_lock = threading.Lock()
        self.key_lock = threading.Lock()
        self.sock = socket.create_connection((host, port), timeout=5)
        with self.send_lock:
            self.sock.sendall(("AUTH:" + token + "\n").encode("utf-8"))
        self.sock.settimeout(1)
        self.alive = True
        self.keys = set()
        self.key_deadlines = {}
        self.drive_enabled = False
        self.manual_mode = False
        self.estop_latched = False
        self.last_frame_time = 0.0
        self.connected_time = time.monotonic()
        self.video_stale = True
        self.video_ready = False
        self.fresh_frame_streak = 0
        self.video_stop_sent = False
        self.last_frame = None
        self.frames = queue.Queue(maxsize=1)
        self.status_updates = queue.Queue()
        self.root = tk.Tk()
        self.root.title("小车第一视角接管 - %s:%d" % (host, port))
        self.root.geometry("1280x900")
        self.root.minsize(800, 600)
        self.status = tk.StringVar(value="已连接；等待实时画面")
        self.guides_enabled = tk.BooleanVar(value=False)
        self.video = tk.Label(self.root, bg="black")
        self.video.pack(fill=tk.BOTH, expand=True)
        tk.Label(self.root, textvariable=self.status, font=("Arial", 14)).pack()
        tk.Checkbutton(
            self.root, text="显示固定参考线（G）",
            variable=self.guides_enabled, takefocus=False
        ).pack()
        tk.Label(
            self.root,
            text="按住 Shift 才能遥控　W/S 前后　A/D 转向　失焦自动急停　空格急停",
            font=("Arial", 12),
        ).pack(pady=8)
        for key in ("w", "a", "s", "d"):
            self.root.bind("<KeyPress-%s>" % key, self.key_down)
            self.root.bind("<KeyRelease-%s>" % key, self.key_up)
        self.root.bind("<KeyPress-Shift_L>", self.enable_down)
        self.root.bind("<KeyPress-Shift_R>", self.enable_down)
        self.root.bind("<KeyRelease-Shift_L>", self.enable_up)
        self.root.bind("<KeyRelease-Shift_R>", self.enable_up)
        self.root.bind("<FocusOut>", self.focus_lost)
        self.root.bind("<space>", self.stop)
        self.root.bind("<KeyPress-c>", self.clear_stop)
        self.root.bind("<KeyPress-r>", self.return_auto)
        self.root.bind("<KeyPress-g>", self.toggle_guides)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        threading.Thread(target=self.receive_loop, daemon=True).start()
        threading.Thread(target=self.command_loop, daemon=True).start()
        self.root.after(20, self.refresh)

    def set_status(self, text):
        self.status_updates.put(text)

    def clear_keys(self):
        with self.key_lock:
            self.keys.clear()
            self.key_deadlines.clear()

    def send(self, command):
        if self.alive:
            try:
                with self.send_lock:
                    self.sock.sendall((command + "\n").encode("ascii"))
            except OSError:
                self.link_lost()

    def key_down(self, event):
        key = event.keysym.lower()
        if (self.manual_mode and self.video_ready and not self.video_stale
                and self.drive_enabled):
            with self.key_lock:
                self.keys.add(key)
                self.key_deadlines[key] = time.monotonic() + 0.25

    def key_up(self, event):
        key = event.keysym.lower()
        with self.key_lock:
            self.keys.discard(key)
            self.key_deadlines.pop(key, None)

    def enable_down(self, _event=None):
        self.drive_enabled = True

    def enable_up(self, _event=None):
        self.drive_enabled = False
        self.clear_keys()

    def focus_lost(self, _event=None):
        self.drive_enabled = False
        self.clear_keys()
        if self.manual_mode:
            self.send("STOP")
            self.set_status("窗口失去焦点；遥控已急停锁止")

    def stop(self, _event=None):
        self.clear_keys()
        self.send("STOP")
        self.set_status("急停")

    def clear_stop(self, _event=None):
        self.clear_keys()
        if not self.video_ready or self.video_stale:
            messagebox.showerror("无法解除急停", "实时画面尚未稳定恢复，禁止解除急停。")
            return
        if messagebox.askyesno("解除急停", "确认周围安全并解除锁存急停？"):
            self.send("CLEAR_STOP")
            self.set_status("急停已解除")

    def return_auto(self, _event=None):
        if not self.manual_mode:
            return
        self.clear_keys()
        self.send("STOP")
        if messagebox.askyesno(
                "切回自动",
                "确认已通过障碍，且尚未到达第一个停靠框？\n切回后车辆将自动识别停靠框。"):
            self.send("RETURN")
            self.set_status("已切回自动但仍锁存停车；确认安全后按 C 解除")

    def toggle_guides(self, _event=None):
        self.guides_enabled.set(not self.guides_enabled.get())

    def command_loop(self):
        mapping = {"w": "W", "s": "S", "a": "A", "d": "D"}
        while self.alive:
            now = time.monotonic()
            with self.key_lock:
                expired = [key for key, deadline in self.key_deadlines.items()
                           if deadline < now]
                for key in expired:
                    self.keys.discard(key)
                    self.key_deadlines.pop(key, None)
                command = "".join(mapping[k] for k in ("w", "s", "a", "d")
                                  if k in self.keys)
            can_drive = (self.manual_mode and self.video_ready and
                         not self.video_stale and self.drive_enabled)
            self.send(command if can_drive and command else "PING")
            time.sleep(0.05)

    def receive_loop(self):
        stream = self.sock.makefile("rb")
        try:
            while self.alive:
                line = stream.readline()
                if not line:
                    raise ConnectionError
                text = line.decode("ascii", errors="replace").strip()
                if text.startswith("STATE:"):
                    fields = text[6:].split(",")
                    speed, servo = fields[0], fields[1]
                    mode = fields[2] if len(fields) > 2 else "MANUAL"
                    was_manual = self.manual_mode
                    self.manual_mode = mode == "MANUAL"
                    if was_manual != self.manual_mode:
                        self.clear_keys()
                        self.drive_enabled = False
                    mode_text = "手动接管" if self.manual_mode else "自动驾驶（仅监看）"
                    estop = len(fields) > 3 and fields[3] == "ESTOP"
                    self.estop_latched = estop
                    if estop:
                        mode_text = "锁存急停（按 C 确认解除）"
                    self.set_status("%s　速度 %.2f m/s　舵机 %.0f" %
                                    (mode_text, float(speed), float(servo)))
                elif text.startswith("IMAGE:"):
                    size = int(text[6:])
                    raw = stream.read(size)
                    if len(raw) != size:
                        raise ConnectionError
                    frame = cv2.imdecode(np.frombuffer(raw, np.uint8),
                                         cv2.IMREAD_COLOR)
                    if frame is not None:
                        now = time.monotonic()
                        if self.last_frame_time and now - self.last_frame_time <= 0.4:
                            self.fresh_frame_streak += 1
                        else:
                            self.fresh_frame_streak = 1
                        self.last_frame_time = now
                        if self.fresh_frame_streak >= 3:
                            self.video_ready = True
                            self.video_stale = False
                            self.video_stop_sent = False
                        if self.frames.full():
                            self.frames.get_nowait()
                        self.frames.put_nowait(frame)
        except (OSError, ValueError, ConnectionError):
            self.link_lost()

    @staticmethod
    def add_guides(frame):
        h, w = frame.shape[:2]
        cv2.line(frame, (int(w*.12), h-1), (int(w*.43), int(h*.55)),
                 (0, 255, 0), max(2, w//320))
        cv2.line(frame, (int(w*.88), h-1), (int(w*.57), int(h*.55)),
                 (0, 255, 0), max(2, w//320))
        for ratio in (.68, .82):
            y = int(h*ratio)
            t = (ratio-.55)/.45
            cv2.line(frame, (int(w*(.43-.31*t)), y),
                     (int(w*(.57+.31*t)), y), (0, 200, 255), 2)
        return frame

    def refresh(self):
        try:
            while True:
                self.status.set(self.status_updates.get_nowait())
        except queue.Empty:
            pass
        try:
            self.last_frame = self.frames.get_nowait()
        except queue.Empty:
            pass

        now = time.monotonic()
        reference_time = self.last_frame_time or self.connected_time
        self.video_stale = now - reference_time > 0.75
        if self.video_stale:
            self.video_ready = False
            self.fresh_frame_streak = 0
            self.clear_keys()
            if self.manual_mode and not self.video_stop_sent:
                self.send("STOP")
                self.video_stop_sent = True
            self.status.set("实时画面已失效；遥控已锁止")

        if self.last_frame is not None:
            frame = self.last_frame.copy()
            if self.guides_enabled.get():
                frame = self.add_guides(frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame)
            available_w = max(320, self.video.winfo_width() - 4)
            available_h = max(240, self.video.winfo_height() - 4)
            resample = (Image.Resampling.LANCZOS if hasattr(Image, "Resampling")
                        else Image.LANCZOS)
            image.thumbnail((available_w, available_h), resample)
            photo = ImageTk.PhotoImage(image)
            self.video.configure(
                image=photo,
                text="画面已失效" if self.video_stale else "",
                foreground="red",
                compound=tk.CENTER,
                font=("Arial", 32, "bold"),
            )
            self.video.image = photo
        else:
            self.video.configure(
                image="", text="等待实时画面" if not self.video_stale else "画面已失效",
                foreground="red", font=("Arial", 32, "bold"))
        if self.alive:
            self.root.after(20, self.refresh)

    def link_lost(self):
        if self.alive:
            self.alive = False
            self.clear_keys()
            if self.estop_latched:
                message = "连接中断：车辆保持锁存急停"
            elif self.manual_mode:
                message = "接管连接中断：车端将在 500 ms 内停车"
            else:
                message = "监控连接中断：自动驾驶继续运行"
            self.set_status(message)

    def close(self):
        if self.alive:
            self.send("STOP")
        self.alive = False
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
        except OSError:
            pass
        self.root.destroy()

    def run(self):
        self.root.focus_force()
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="小车第一视角人工接管客户端")
    parser.add_argument("host", help="小车 IP 地址")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--token", default=os.getenv("ICAR_MANUAL_TOKEN"),
                        help="pre-shared token (defaults to ICAR_MANUAL_TOKEN)")
    args = parser.parse_args()
    if not args.token:
        parser.error("provide --token or set ICAR_MANUAL_TOKEN")
    TakeoverConsole(args.host, args.port, args.token).run()


if __name__ == "__main__":
    main()
