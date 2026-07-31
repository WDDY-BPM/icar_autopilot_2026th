#!/usr/bin/env python3
"""First-person remote takeover console for the construction zone."""

import argparse
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
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(1)
        self.alive = True
        self.keys = set()
        self.manual_mode = False
        self.frames = queue.Queue(maxsize=1)
        self.status_updates = queue.Queue()
        self.root = tk.Tk()
        self.root.title("小车第一视角接管 - %s:%d" % (host, port))
        self.root.geometry("980x720")
        self.status = tk.StringVar(value="已连接；车辆保持停止")
        self.video = tk.Label(self.root, bg="black")
        self.video.pack(fill=tk.BOTH, expand=True)
        tk.Label(self.root, textvariable=self.status, font=("Arial", 14)).pack()
        tk.Label(
            self.root,
            text="W/S 前进后退　A/D 转向　空格急停　R 切回自动",
            font=("Arial", 12),
        ).pack(pady=8)
        for key in ("w", "a", "s", "d"):
            self.root.bind("<KeyPress-%s>" % key, self.key_down)
            self.root.bind("<KeyRelease-%s>" % key, self.key_up)
        self.root.bind("<space>", self.stop)
        self.root.bind("<KeyPress-r>", self.return_auto)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        threading.Thread(target=self.receive_loop, daemon=True).start()
        threading.Thread(target=self.command_loop, daemon=True).start()
        self.root.after(20, self.refresh)

    def set_status(self, text):
        self.status_updates.put(text)

    def send(self, command):
        if self.alive:
            try:
                self.sock.sendall((command + "\n").encode("ascii"))
            except OSError:
                self.link_lost()

    def key_down(self, event):
        if self.manual_mode:
            self.keys.add(event.keysym.lower())

    def key_up(self, event):
        self.keys.discard(event.keysym.lower())

    def stop(self, _event=None):
        self.keys.clear()
        self.send("STOP")
        self.set_status("急停")

    def return_auto(self, _event=None):
        if not self.manual_mode:
            return
        self.keys.clear()
        self.send("STOP")
        if messagebox.askyesno("切回自动", "确认小车已安全驶出施工路况？"):
            self.send("RETURN")
            self.set_status("已请求切回自动模式")

    def command_loop(self):
        mapping = {"w": "W", "s": "S", "a": "A", "d": "D"}
        while self.alive:
            command = "".join(mapping[k] for k in ("w", "s", "a", "d")
                              if k in self.keys)
            self.send(command if self.manual_mode and command else "PING")
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
                        self.keys.clear()
                    mode_text = "手动接管" if self.manual_mode else "自动驾驶（仅监看）"
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
            frame = cv2.cvtColor(self.add_guides(self.frames.get_nowait()),
                                 cv2.COLOR_BGR2RGB)
            image = Image.fromarray(frame)
            image.thumbnail((960, 620))
            photo = ImageTk.PhotoImage(image)
            self.video.configure(image=photo)
            self.video.image = photo
        except queue.Empty:
            pass
        if self.alive:
            self.root.after(20, self.refresh)

    def link_lost(self):
        if self.alive:
            self.alive = False
            self.keys.clear()
            self.set_status("连接中断：车端将自动停车")

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
    args = parser.parse_args()
    TakeoverConsole(args.host, args.port).run()


if __name__ == "__main__":
    main()
