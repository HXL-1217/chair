#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Orange Pi / Headless 专用手柄读取脚本
依赖: pip3 install pygame
"""

import os
import sys
import time

# 【关键1】必须在 import pygame 前设置虚拟驱动，避免尝试打开 /dev/fb0 或 X11
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import pygame

class DevicesHandle:
    def __init__(self, joystick_index=0):
        # 【关键2】在 dummy 驱动下调用 init 是安全的，不会报错，但能解决 "video system not initialized"
        # 我们只初始化需要的模块，减少资源占用
        pygame.display.init()   # 初始化显示子系统(虚拟后端)
        pygame.joystick.init()  # 初始化摇杆子系统

        count = pygame.joystick.get_count()
        print(f"🔍 系统检测到手柄数量: {count}", flush=True)
        
        if count == 0:
            print("❌ 未检测到手柄，请检查 USB 连接或尝试运行: ls -l /dev/input/js*", flush=True)
            raise RuntimeError("No joystick detected")
        
        if joystick_index >= count:
            raise RuntimeError(f"索引 {joystick_index} 超出范围 (0-{count-1})")

        self.joystick = pygame.joystick.Joystick(joystick_index)
        self.joystick.init()
        print(f"✅ 已初始化: {self.joystick.get_name()}", flush=True)

        self.done = False
        self.deadzone = 0.05
        
        # 动态初始化状态列表，适配不同手柄
        self.uAxes = [0.0] * self.joystick.get_numaxes()
        self.uKey = [0] * self.joystick.get_numbuttons()
        self.uHat = [(0, 0)] * self.joystick.get_numhats()
        
        # 打印映射参考，方便后续开发
        print(f"📊 硬件参数: {self.joystick.get_numaxes()}轴, {self.joystick.get_numbuttons()}键, {self.joystick.get_numhats()}方向键", flush=True)

    def update(self):
        """刷新事件队列 (Headless 环境必须)"""
        # 在 dummy 驱动下，pump() 是更新手柄状态的关键
        pygame.event.pump()

        # 直接读取最新状态，非阻塞
        for i in range(self.joystick.get_numaxes()):
            val = self.joystick.get_axis(i)
            self.uAxes[i] = val if abs(val) > self.deadzone else 0.0
            
        for i in range(self.joystick.get_numbuttons()):
            self.uKey[i] = self.joystick.get_button(i)
            
        for i in range(self.joystick.get_numhats()):
            self.uHat[i] = self.joystick.get_hat(i)

    def get_state(self):
        return self.uAxes.copy(), self.uKey.copy(), self.uHat.copy()

    def stop(self):
        self.done = True
        self.joystick.quit()
        pygame.joystick.quit()
        pygame.display.quit()


def main():
    handle = None
    try:
        handle = DevicesHandle()
        print("🚀 开始读取数据 (Ctrl+C 退出)...\n", flush=True)
        time.sleep(1)  # 给一点缓冲时间

        while not handle.done:
            handle.update()
            axes, keys, hats = handle.get_state()
            
            # 单行刷新输出，适合 SSH 终端
            # 仅显示前6个轴和前8个键，避免终端过宽
            status = f"\r🎮 Axes:{axes[:6]} | Keys:{keys[:8]} | Hat:{hats}"
            print(status, end="", flush=True)
            
            time.sleep(0.02)  # 50Hz 刷新率，平衡实时性与 CPU 占用

    except KeyboardInterrupt:
        print("\n👋 用户终止，退出程序。", flush=True)
    except Exception as e:
        print(f"\n❌ 错误: {type(e).__name__}: {e}", flush=True)
        # 如果是权限问题，给出具体建议
        if "Permission" in str(e) or "input" in str(e).lower():
            print("💡 提示: 可能是权限问题，尝试执行: sudo usermod -aG input $USER 然后重启", flush=True)
        sys.exit(1)
    finally:
        if handle:
            handle.stop()

if __name__ == "__main__":
    main()