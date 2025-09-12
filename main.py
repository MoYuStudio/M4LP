import pybullet as p
import pybullet_data
import time
import math

# 连接到物理服务器
p.connect(p.GUI)                 # 或 p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置重力
p.setGravity(0, 0, -9.81)

# 加载地面
p.loadURDF("plane.urdf")

# 加载机器人
robot = p.loadURDF("m4lp.urdf",
                   basePosition=[0, 0, 0.35],
                   useFixedBase=False)

# 设置轮子的摩擦系数，使其能够在地面上滚动
for j in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, j)
    joint_name = joint_info[1].decode('utf-8')
    if 'wheel' in joint_name:
        p.changeDynamics(robot, j, 
                        lateralFriction=1.0,   # 侧向摩擦
                        spinningFriction=0.1,  # 旋转摩擦
                        rollingFriction=0.01)  # 滚动摩擦

# 获取关节信息
joint_indices = {}
wheel_indices = {}
joint_names = {}
for j in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, j)
    joint_name = joint_info[1].decode('utf-8')
    joint_names[j] = joint_name
    if 'hip' in joint_name or 'knee' in joint_name:
        joint_indices[joint_name] = j
    elif 'wheel' in joint_name:
        wheel_indices[joint_name] = j

print("找到的关节:", joint_indices)
print("找到的轮子:", wheel_indices)

# 控制参数
wheel_speed = 6.0  # 轮子速度
time_step = 0.01    # 时间步长
max_speed = 24.0    # 最大速度
turn_speed = 8.0    # 转向速度

# ========================================
# 四足机器人姿态参数配置 (简化管理)
# ========================================
# 说明：每条腿使用一个伸腿数值控制（0-1范围）
# 0 = 屈膝状态，1 = 站立状态
#
# 左前腿伸腿值（FL）
FL_EXTEND = 0.3          # 左前腿伸腿程度：0=屈膝，1=站立

# 右前腿伸腿值（FR）
FR_EXTEND = 0.3          # 右前腿伸腿程度：0=屈膝，1=站立

# 左后腿伸腿值（RL）
RL_EXTEND = 0.3          # 左后腿伸腿程度：0=屈膝，1=站立

# 右后腿伸腿值（RR）
RR_EXTEND = 0.3          # 右后腿伸腿程度：0=屈膝，1=站立

# 当前移动状态
current_speed = 0.0
current_turn = 0.0

# 模式控制
current_mode = 1  # 1=默认模式, 2=踏步模式

# 姿态调整参数
height_adjustment = 0.0    # 高度调整偏移量
height_step = 0.1          # 每次高度调整的步长
roll_adjustment = 0.0      # 左右倾斜调整偏移量
roll_step = 0.1            # 每次倾斜调整的步长

# 踏步模式参数
step_amplitude = 0.3       # 踏步幅度
step_frequency = 6.0       # 踏步频率 (Hz) - 加快3倍
step_phase_offset = 0.5    # 腿之间的相位差 (0.5 = 180度)

# 踏步模式伸腿值循环
step_cycle_1 = [0.1, 0.5, 0.5, 0.1]  # 第一组：FL, FR, RL, RR
step_cycle_2 = [0.5, 0.1, 0.1, 0.5]  # 第二组：FL, FR, RL, RR
current_step_cycle = step_cycle_1     # 当前使用的循环

# 伸腿值转换为关节角度的函数
def extend_to_joint_angles(extend_value, leg_type):
    """
    将伸腿值(0-1)转换为髋关节和膝盖关节角度
    extend_value: 0=屈膝，1=站立
    leg_type: 'front' 或 'rear'
    """
    if leg_type == 'front':
        # 前腿：从屈膝到站立
        hip_angle = 1.2 - extend_value * 1.5    # 1.5 到 1.0
        knee_angle = -1.5 + extend_value * 0.6    # -1.5 到 -1.7
    else:  # rear
        # 后腿：从屈膝到站立
        hip_angle = -1.2 + extend_value * 1.5     # -1.5 到 -1.0
        knee_angle = 1.5 - extend_value * 0.6   # 1.5 到 1.7
    
    return hip_angle, knee_angle

# 初始姿态 - 屈膝稳定姿态，降低底盘
def set_stable_pose():
    # 设置关节到屈膝稳定位置
    for joint_name, joint_id in joint_indices.items():
        if 'hip' in joint_name:
            # 髋关节角度：使用伸腿值 + 高度调整 + 左右倾斜调整
            if 'FL' in joint_name:  # 左前腿
                base_hip, _ = extend_to_joint_angles(FL_EXTEND, 'front')
                hip_angle = base_hip + height_adjustment - roll_adjustment
            elif 'FR' in joint_name:  # 右前腿
                base_hip, _ = extend_to_joint_angles(FR_EXTEND, 'front')
                hip_angle = base_hip + height_adjustment + roll_adjustment
            elif 'RL' in joint_name:  # 左后腿
                base_hip, _ = extend_to_joint_angles(RL_EXTEND, 'rear')
                hip_angle = base_hip - height_adjustment - roll_adjustment
            elif 'RR' in joint_name:  # 右后腿
                base_hip, _ = extend_to_joint_angles(RR_EXTEND, 'rear')
                hip_angle = base_hip - height_adjustment + roll_adjustment
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=hip_angle, force=30)
        elif 'knee' in joint_name:
            # 膝盖关节角度：使用伸腿值 + 高度调整 + 左右倾斜调整
            if 'FL' in joint_name:  # 左前腿膝盖
                _, base_knee = extend_to_joint_angles(FL_EXTEND, 'front')
                knee_angle = base_knee - height_adjustment + roll_adjustment
            elif 'FR' in joint_name:  # 右前腿膝盖
                _, base_knee = extend_to_joint_angles(FR_EXTEND, 'front')
                knee_angle = base_knee - height_adjustment - roll_adjustment
            elif 'RL' in joint_name:  # 左后腿膝盖
                _, base_knee = extend_to_joint_angles(RL_EXTEND, 'rear')
                knee_angle = base_knee + height_adjustment + roll_adjustment
            elif 'RR' in joint_name:  # 右后腿膝盖
                _, base_knee = extend_to_joint_angles(RR_EXTEND, 'rear')
                knee_angle = base_knee + height_adjustment - roll_adjustment
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=knee_angle, force=30)
    
    # 停止所有轮子
    for wheel_name, wheel_id in wheel_indices.items():
        p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                               targetVelocity=0.0, force=50)

# 踏步模式 - 循环踏步
def step_mode_control(t):
    global current_step_cycle
    
    # 计算循环切换（每1秒切换一次）
    cycle_index = int(t * step_frequency) % 2
    if cycle_index == 0:
        current_step_cycle = step_cycle_1
    else:
        current_step_cycle = step_cycle_2
    
    # 获取当前循环的伸腿值
    fl_extend = current_step_cycle[0]  # FL
    fr_extend = current_step_cycle[1]  # FR
    rl_extend = current_step_cycle[2]  # RL
    rr_extend = current_step_cycle[3]  # RR
    
    # 应用伸腿值到关节
    for joint_name, joint_id in joint_indices.items():
        if 'hip' in joint_name:
            if 'FL' in joint_name:  # 左前腿
                base_hip, _ = extend_to_joint_angles(fl_extend, 'front')
                hip_angle = base_hip + height_adjustment - roll_adjustment
            elif 'FR' in joint_name:  # 右前腿
                base_hip, _ = extend_to_joint_angles(fr_extend, 'front')
                hip_angle = base_hip + height_adjustment + roll_adjustment
            elif 'RL' in joint_name:  # 左后腿
                base_hip, _ = extend_to_joint_angles(rl_extend, 'rear')
                hip_angle = base_hip - height_adjustment - roll_adjustment
            elif 'RR' in joint_name:  # 右后腿
                base_hip, _ = extend_to_joint_angles(rr_extend, 'rear')
                hip_angle = base_hip - height_adjustment + roll_adjustment
            
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                   targetPosition=hip_angle, force=30)
                                   
        elif 'knee' in joint_name:
            if 'FL' in joint_name:  # 左前腿膝盖
                _, base_knee = extend_to_joint_angles(fl_extend, 'front')
                knee_angle = base_knee - height_adjustment + roll_adjustment
            elif 'FR' in joint_name:  # 右前腿膝盖
                _, base_knee = extend_to_joint_angles(fr_extend, 'front')
                knee_angle = base_knee - height_adjustment - roll_adjustment
            elif 'RL' in joint_name:  # 左后腿膝盖
                _, base_knee = extend_to_joint_angles(rl_extend, 'rear')
                knee_angle = base_knee + height_adjustment + roll_adjustment
            elif 'RR' in joint_name:  # 右后腿膝盖
                _, base_knee = extend_to_joint_angles(rr_extend, 'rear')
                knee_angle = base_knee + height_adjustment - roll_adjustment
            
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                   targetPosition=knee_angle, force=30)

# PyBullet键盘事件检测 - 支持连续按键
def check_keyboard_input():
    global current_speed, current_turn, height_adjustment, roll_adjustment, current_mode
    
    # 获取键盘事件
    keys = p.getKeyboardEvents()
    
    # 重置移动状态
    current_speed = 0.0
    current_turn = 0.0
    
    # 检测按键状态（支持连续按键）
    # 前进/后退
    if ord('i') in keys:  # 前进
        current_speed = max_speed
    elif ord('k') in keys:  # 后退
        current_speed = -max_speed
    
    # 左倾/右倾
    if ord('j') in keys and keys[ord('j')] & p.KEY_WAS_TRIGGERED:  # J键 - 左倾
        roll_adjustment += roll_step
        print(f"左倾调整: +{roll_step:.2f}, 当前偏移: {roll_adjustment:.2f}")
    elif ord('l') in keys and keys[ord('l')] & p.KEY_WAS_TRIGGERED:  # L键 - 右倾
        roll_adjustment -= roll_step
        print(f"右倾调整: -{roll_step:.2f}, 当前偏移: {roll_adjustment:.2f}")
    
    # 左转/右转 - 使用方向键
    if ord('4') in keys:  # 左方向键 - 左转
        current_turn = turn_speed
    elif ord('6') in keys:  # 右方向键 - 右转
        current_turn = -turn_speed
    
    # 模式切换
    if ord('1') in keys and keys[ord('1')] & p.KEY_WAS_TRIGGERED:  # 1键 - 静止模式
        current_mode = 1
        print("切换到静止模式")
    elif ord('2') in keys and keys[ord('2')] & p.KEY_WAS_TRIGGERED:  # 2键 - 踏步模式
        current_mode = 2
        print("切换到踏步模式")
    
    # 高度调整 - 使用U和O键
    if ord('u') in keys and keys[ord('u')] & p.KEY_WAS_TRIGGERED:  # U键 - 升高
        height_adjustment += height_step
        print(f"高度调整: +{height_step:.2f}, 当前偏移: {height_adjustment:.2f}")
    elif ord('o') in keys and keys[ord('o')] & p.KEY_WAS_TRIGGERED:  # O键 - 降低
        height_adjustment -= height_step
        print(f"高度调整: -{height_step:.2f}, 当前偏移: {height_adjustment:.2f}")
    
    
    # 停止键（按下时立即停止）
    if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:  # 空格键停止
        current_speed = 0.0
        current_turn = 0.0

# 轮子驱动控制 - 像遥控汽车一样
def wheel_drive_control(t):
    global current_speed, current_turn
    
    # 检测键盘输入
    check_keyboard_input()
    
    # 根据模式执行不同的控制
    if current_mode == 1:
        # 静止模式：轮子驱动 + 稳定姿态
        # 计算每个轮子的速度
        front_left_speed = current_speed + current_turn
        front_right_speed = current_speed - current_turn
        rear_left_speed = current_speed + current_turn
        rear_right_speed = current_speed - current_turn
        
        # 应用轮子速度
        wheel_controls = {
            'FL_wheel_joint': front_left_speed,
            'FR_wheel_joint': front_right_speed,
            'RL_wheel_joint': rear_left_speed,
            'RR_wheel_joint': rear_right_speed
        }
        
        for wheel_name, wheel_id in wheel_indices.items():
            if wheel_name in wheel_controls:
                p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                                       targetVelocity=wheel_controls[wheel_name], force=50)
        
        # 保持四足屈膝稳定姿态
        for joint_name, joint_id in joint_indices.items():
            if 'hip' in joint_name:
                if 'FL' in joint_name:
                    base_hip, _ = extend_to_joint_angles(FL_EXTEND, 'front')
                    hip_angle = base_hip + height_adjustment - roll_adjustment
                elif 'FR' in joint_name:
                    base_hip, _ = extend_to_joint_angles(FR_EXTEND, 'front')
                    hip_angle = base_hip + height_adjustment + roll_adjustment
                elif 'RL' in joint_name:
                    base_hip, _ = extend_to_joint_angles(RL_EXTEND, 'rear')
                    hip_angle = base_hip - height_adjustment - roll_adjustment
                elif 'RR' in joint_name:
                    base_hip, _ = extend_to_joint_angles(RR_EXTEND, 'rear')
                    hip_angle = base_hip - height_adjustment + roll_adjustment
                p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                       targetPosition=hip_angle, force=30)
            elif 'knee' in joint_name:
                if 'FL' in joint_name:
                    _, base_knee = extend_to_joint_angles(FL_EXTEND, 'front')
                    knee_angle = base_knee - height_adjustment + roll_adjustment
                elif 'FR' in joint_name:
                    _, base_knee = extend_to_joint_angles(FR_EXTEND, 'front')
                    knee_angle = base_knee - height_adjustment - roll_adjustment
                elif 'RL' in joint_name:
                    _, base_knee = extend_to_joint_angles(RL_EXTEND, 'rear')
                    knee_angle = base_knee + height_adjustment + roll_adjustment
                elif 'RR' in joint_name:
                    _, base_knee = extend_to_joint_angles(RR_EXTEND, 'rear')
                    knee_angle = base_knee + height_adjustment - roll_adjustment
                p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                       targetPosition=knee_angle, force=30)
                                       
    elif current_mode == 2:
        # 踏步模式：小碎步动画
        # 停止所有轮子
        for wheel_name, wheel_id in wheel_indices.items():
            p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                                   targetVelocity=0.0, force=50)
        
        # 执行踏步动画
        step_mode_control(t)

# 设置初始稳定姿态
set_stable_pose()

# 等待机器人稳定
for _ in range(100):
    p.stepSimulation()
    time.sleep(time_step)

print("开始遥控汽车模式...")
print("控制说明:")
print("=== 模式切换 ===")
print("1 - 静止模式（轮子驱动）")
print("2 - 踏步模式（循环踏步）")
print("=== 移动控制 ===")
print("I - 前进")
print("K - 后退") 
print("← (左方向键) - 左转")
print("→ (右方向键) - 右转")
print("空格键 - 停止")
print("=== 姿态调整 ===")
print("U - 升高机器人")
print("O - 降低机器人")
print("J - 左倾")
print("L - 右倾")
print("=== 其他 ===")
print("按 Ctrl+C 退出")

# 主仿真循环
try:
    t = 0
    last_status_time = 0
    
    while True:
        # 执行轮子驱动控制
        wheel_drive_control(t)
        
        # 推进仿真
        p.stepSimulation()
        time.sleep(time_step)
        t += time_step
        
        # 每3秒输出一次状态
        if t - last_status_time >= 3.0:
            pos, orn = p.getBasePositionAndOrientation(robot)
            vel, ang_vel = p.getBaseVelocity(robot)
            euler = p.getEulerFromQuaternion(orn)
            
            # 显示模式
            mode_name = "静止模式" if current_mode == 1 else "踏步模式"
            
            # 显示移动状态
            status = ""
            if current_mode == 1:  # 只在静止模式下显示移动状态
                if current_speed > 0:
                    status += "前进 "
                elif current_speed < 0:
                    status += "后退 "
                if current_turn > 0:
                    status += "左转 "
                elif current_turn < 0:
                    status += "右转 "
                if not status:
                    status = "停止"
            else:  # 踏步模式
                status = "踏步中"
            
            # 显示姿态调整状态
            height_status = f"高度偏移: {height_adjustment:.2f}"
            roll_status = f"倾斜偏移: {roll_adjustment:.2f}"
            
            print(f"时间: {t:.1f}s | 模式: {mode_name} | 状态: {status}")
            print(f"位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"速度: ({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f})")
            print(f"姿态: 滚转={euler[0]:.2f}, 俯仰={euler[1]:.2f}, 偏航={euler[2]:.2f}")
            print(f"{height_status} | {roll_status}")
            print("-" * 50)
            
            last_status_time = t

except KeyboardInterrupt:
    print("\n仿真已停止")
finally:
    p.disconnect()