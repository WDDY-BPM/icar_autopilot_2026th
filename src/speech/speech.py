#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
行驶指令 → config.json 完整解析流程
"""

import json
import os
import subprocess
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from llm import LLM


def parsedToLapConfig(parsed: dict) -> dict:
    """Map a validated task list to consecutive lap configurations."""
    normalizeTasks(parsed)
    makers = {
        "park": lambda task: _makeParkLap(task["spot"]),
        "construction": lambda task: _makeBusyLap(task["stop"]),
        "fork": lambda task: _makeForkLap(task["direction"]),
    }
    return {
        f"lap{index}": makers[task["type"]](task)
        for index, task in enumerate(parsed["tasks"], start=1)
    }


def _makeDisabledLap() -> dict:
    """Safe value used to completely replace unused lap slots."""
    lap = _makeParkLap(0)
    lap["park"] = False
    return lap

def _makeParkLap(spot: int) -> dict:
    return {
        "park": True,
        "parkSpot": spot,
        "busy": False,
        "fork": False,
        "slow": True,
        "cross": True,
        "station": False,
        "stop": True,
        "curve": False,
        "fine": False,
        "yfork": False,
        "yforkLeft": False,
        "manualTakeover": False,
        "busyStopEnable": False,
        "busyStopPoint": 0,
    }


def _makeBusyLap(stop: int) -> dict:
    return {
        "park": False,
        "parkSpot": 0,
        "busy": True,
        "fork": False,
        "slow": True,
        "cross": True,
        "station": True,
        "stop": False,
        "curve": False,
        "fine": False,
        "yfork": False,
        "yforkLeft": False,
        "manualTakeover": True,
        "busyStopEnable": True,
        "busyStopPoint": stop,
    }


def _makeForkLap(direction: str) -> dict:
    return {
        "park": False,
        "parkSpot": 0,
        "busy": False,
        "fork": False,
        "slow": True,
        "cross": True,
        "station": False,
        "stop": True,
        "curve": False,
        "fine": False,
        "yfork": True,
        "yforkLeft": (direction == "left"),
        "manualTakeover": False,
        "busyStopEnable": False,
        "busyStopPoint": 0,
    }


def updateConfigJson(lapConfig: dict, totalLaps: int, configPath: str = None):
    """将每圈配置写入 config.json"""
    if configPath is None:
        configPath = os.path.join(os.path.dirname(__file__), "../../res/config.json")
    configPath = os.path.abspath(configPath)
    with open(configPath, "r", encoding="utf-8") as f:
        config = json.load(f)

    totalLaps = int(totalLaps)
    if not 1 <= totalLaps <= 3:
        raise ValueError("totalLaps must be between 1 and 3")
    expected = {f"lap{i}" for i in range(1, totalLaps + 1)}
    if set(lapConfig) != expected:
        raise ValueError("lapConfig must contain consecutive laps matching totalLaps")
    config["圈数配置"]["totalLaps"] = totalLaps
    config["每圈功能使能配置"] = {
        f"lap{i}": lapConfig.get(f"lap{i}", _makeDisabledLap())
        for i in range(1, 4)
    }

    with open(configPath, "w", encoding="utf-8") as f:
        json.dump(config, f, ensure_ascii=False, indent=2)

    print(f"已更新 {configPath}")
    print(f"  总圈数: {totalLaps}")
    for lapNum in sorted(lapConfig.keys()):
        lap = lapConfig[lapNum]
        info = _describeLap(lap)
        print(f"  {lapNum} → {info}")


def _describeLap(lap: dict) -> str:
    if lap["park"]:
        if lap.get("parkSpot") == 0:
            return "停车场(穿过)"
        return f"停车场(停车位 {lap['parkSpot']})"
    if lap["busy"]:
        pos = "中间" if lap["busyStopPoint"] == 1 else "出口"
        return f"施工区({pos})"
    if lap.get("fork") or lap.get("yfork"):
        side = "左侧" if lap.get("yforkLeft") else "右侧"
        return f"岔路口({side})"
    return "未知"


def normalizeTasks(parsed: dict):
    """Normalize and strictly validate one to three driving tasks."""
    if not isinstance(parsed, dict) or not isinstance(parsed.get("tasks"), list):
        raise ValueError("tasks must be a list")
    tasks = parsed["tasks"]
    if not 1 <= len(tasks) <= 3:
        raise ValueError("task count must be between 1 and 3")

    normalized = []
    for index, raw_task in enumerate(tasks, start=1):
        task = {"type": raw_task} if isinstance(raw_task, str) else raw_task
        if not isinstance(task, dict):
            raise ValueError(f"task {index} must be an object")
        task_type = task.get("type")
        if task_type == "park":
            spot = task.get("spot")
            if isinstance(spot, bool) or not isinstance(spot, int) or not 0 <= spot <= 4:
                raise ValueError(f"task {index}: park spot must be 0..4")
            normalized.append({"type": "park", "spot": spot})
        elif task_type == "construction":
            stop = task.get("stop")
            if isinstance(stop, bool) or not isinstance(stop, int) or stop not in (1, 2):
                raise ValueError(f"task {index}: construction stop must be 1 or 2")
            normalized.append({"type": "construction", "stop": stop})
        elif task_type == "fork":
            direction = task.get("direction")
            if direction not in ("left", "right"):
                raise ValueError(f"task {index}: fork direction must be left or right")
            normalized.append({"type": "fork", "direction": direction})
        else:
            raise ValueError(f"task {index}: unsupported type {task_type!r}")
    parsed["tasks"] = normalized
    return parsed

# ============ 使用示例 ============

if __name__ == "__main__":
    llm = LLM()

    while True:
        instruction = input("\n请输入行驶指令: ")

        print(f"原始指令: {instruction}\n")

        # 1. LLM 解析指令 → task 列表
        result = llm.parseInstruction(instruction)
        if not result:
            print("解析失败！请检查 API Key 是否有效")
            continue

        print(f"LLM 解析结果: {json.dumps(result, ensure_ascii=False)}\n")

        # 2. 修复格式
        try:
            normalizeTasks(result)
        except ValueError as exc:
            print(f"Invalid task configuration: {exc}")
            continue
        print(f"修正后: {json.dumps(result, ensure_ascii=False)}\n")

        # 3. task 列表 → 每圈配置
        lapConfig = parsedToLapConfig(result)
        totalLaps = len(result["tasks"])
        print("映射为每圈配置:")
        print(json.dumps(lapConfig, ensure_ascii=False, indent=2))
        print(f"  总圈数: {totalLaps}")
        for lapNum in sorted(lapConfig.keys()):
            lap = lapConfig[lapNum]
            info = _describeLap(lap)
            print(f"  {lapNum} → {info}")
            
        # 4. 用户确认（大小写均可）
        confirm = input("\n解析结果是否正确？(y/n): ").strip().lower()
        if confirm != "y":
            print("已取消，请重新输入指令。")
            continue

        # 5. 写入 config.json
        updateConfigJson(lapConfig, totalLaps)
        print("\n已写入 config.json")

        # 6. 启动小车程序
        buildDir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../build"))
        icarPath = os.path.join(buildDir, "icar")

        if not os.path.exists(icarPath):
            print(f"\n未找到小车程序: {icarPath}")
            break

        bootNeeded = input("\n是否需要先启动boot？(y/n): ").strip().lower()
        if bootNeeded == "y":
            bootCmd = [
                "gnome-terminal", "--working-directory", buildDir, "--", "./boot"
            ]
            bootEnv = os.environ.copy()
            bootEnv.setdefault("DISPLAY", ":0.0")
            subprocess.Popen(bootCmd, cwd=buildDir, env=bootEnv)
            print("已启动 boot，等待 3 秒...")
            time.sleep(3)

        print(f"\n正在启动小车程序: {icarPath}")
        subprocess.Popen(
            ["nohup", "./icar"],
            cwd=buildDir,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print("小车程序已启动")
        break
