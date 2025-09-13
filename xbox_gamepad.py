"""
Xbox 手柄扩展
author: human
"""
import ctypes
import math
import threading
import time

# -------------------- 常量 --------------------
DEAD_L = 7849
DEAD_R = 8689
TRIG_T = 30

# 按键掩码
BTN_A = 0x1000
BTN_B = 0x2000
BTN_X = 0x4000
BTN_Y = 0x8000
BTN_LB = 0x0100
BTN_RB = 0x0200
BTN_BACK = 0x0020
BTN_START = 0x0010
BTN_LS = 0x0040   # Left Stick Click
BTN_RS = 0x0080   # Right Stick Click
DPAD_UP = 0x0001
DPAD_DOWN = 0x0002
DPAD_LEFT = 0x0004
DPAD_RIGHT = 0x0008

# -------------------- XInput 结构 --------------------
class _GAMEPAD(ctypes.Structure):
    _fields_ = [("wButtons", ctypes.c_ushort),
                ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short),
                ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]

class _STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_uint32),
                ("Gamepad", _GAMEPAD)]

# -------------------- 加载 XInput --------------------
try:
    _xinput = ctypes.windll.xinput1_4
except OSError:
    _xinput = ctypes.windll.xinput1_3

# -------------------- 工具 --------------------
def _norm(v, dead):
    if abs(v) < dead:
        return 0.0
    return math.copysign((abs(v) - dead) / (32768 - dead), v) * 32768 / 32768

# -------------------- 主类 --------------------
class XboxGamepad:
    """线程安全、热插拔、带触发沿检测的 Xbox 手柄"""
    def __init__(self, callback=None):
        self._callback = callback or (lambda ev, btn: None)
        self._running = True
        self._state = None
        self._prev_buttons = set()
        self._lock = threading.Lock()
        threading.Thread(target=self._poll, daemon=True).start()

    def _poll(self):
        raw = _STATE()
        while self._running:
            connected = _xinput.XInputGetState(0, ctypes.byref(raw)) == 0
            if connected:
                g = raw.Gamepad
                btn = {1 << i for i in range(16) if g.wButtons & (1 << i)}
                with self._lock:
                    self._state = dict(
                        LX=_norm(g.sThumbLX, DEAD_L),
                        LY=_norm(g.sThumbLY, DEAD_L),
                        RX=_norm(g.sThumbRX, DEAD_R),
                        RY=_norm(g.sThumbRY, DEAD_R),
                        LT=g.bLeftTrigger / 255,
                        RT=g.bRightTrigger / 255,
                        buttons=btn.copy())
                    # 触发沿检测
                    for b in btn - self._prev_buttons:
                        self._callback('press', b)
                    for b in self._prev_buttons - btn:
                        self._callback('release', b)
                    self._prev_buttons = btn
            else:
                with self._lock:
                    if self._state is not None:
                        for b in self._prev_buttons:
                            self._callback('release', b)
                        self._state = None
                        self._prev_buttons.clear()
            time.sleep(0.01)

    def read(self):
        with self._lock:
            return self._state

    def close(self):
        self._running = False


# -------------------- 示例 --------------------
if __name__ == "__main__":
    def on_event(ev, btn):
        name = {BTN_A:'A', BTN_B:'B', BTN_X:'X', BTN_Y:'Y',
                BTN_LB:'LB', BTN_RB:'RB',
                DPAD_UP:'↑', DPAD_DOWN:'↓', DPAD_LEFT:'←', DPAD_RIGHT:'→'}.get(btn, f'0x{btn:04X}')
        print(f"[{ev}] {name}")

    pad = XboxGamepad(callback=on_event)
    print("手柄测试中…… 十字键/LB/RB 已支持（Ctrl-C 退出）")
    try:
        while True:
            st = pad.read()
            if st:
                print(f"\rLX:{st['LX']:+.2f} LY:{st['LY']:+.2f}  "
                      f"RX:{st['RX']:+.2f} RY:{st['RY']:+.2f}  "
                      f"LT:{st['LT']:.2f} RT:{st['RT']:.2f}", end='')
            else:
                print('\r未连接           ', end='')
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        pad.close()