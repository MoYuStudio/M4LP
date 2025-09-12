import pybullet as p
import pybullet_data
import time
import math
from pid import BalanceController

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

# 陀螺仪数据存储
gyro_data = {
    'angular_velocity': [0.0, 0.0, 0.0],  # 角速度 (rad/s)
    'linear_acceleration': [0.0, 0.0, 0.0],  # 线性加速度 (m/s²)
    'orientation': [0.0, 0.0, 0.0],  # 姿态角 (roll, pitch, yaw)
    'orientation_quat': [0.0, 0.0, 0.0, 1.0]  # 四元数姿态
}

# 陀螺仪数据历史记录（用于分析）
gyro_history = {
    'time': [],
    'angular_velocity': [],
    'linear_acceleration': [],
    'orientation': []
}

# 陀螺仪数据记录参数
gyro_record_enabled = True  # 是否记录数据
gyro_max_history = 1000     # 最大历史记录数量

# 控制参数
wheel_speed = 6.0  # 轮子速度
time_step = 0.01    # 时间步长
max_speed = 24.0    # 最大速度
turn_speed = 12.0   # 转向速度（增加转向力度）
leg_lift_speed = 0.1  # 腿部抬起速度

# PID平衡控制器
balance_controller = BalanceController()
balance_enabled = True  # 平衡器启用状态

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

# 独立腿部控制变量
leg_lifts = {
    'FL': 0.0,  # 左前腿抬起量
    'FR': 0.0,  # 右前腿抬起量
    'RL': 0.0,  # 左后腿抬起量
    'RR': 0.0   # 右后腿抬起量
}

# 模式控制
current_mode = 1  # 1=默认模式, 2=踏步模式

# 姿态调整参数
height_adjustment = 0.0    # 高度调整偏移量
height_step = 0.1          # 每次高度调整的步长

# 踏步模式参数
step_amplitude = 0.3       # 踏步幅度
step_frequency = 6.0       # 踏步频率 (Hz) - 加快3倍
step_phase_offset = 0.5    # 腿之间的相位差 (0.5 = 180度)

# 踏步模式伸腿值循环
step_cycle_1 = [0.1, 0.5, 0.5, 0.1]  # 第一组：FL, FR, RL, RR
step_cycle_2 = [0.5, 0.1, 0.1, 0.5]  # 第二组：FL, FR, RL, RR
current_step_cycle = step_cycle_1     # 当前使用的循环

# 陀螺仪数据获取函数
def get_gyroscope_data(t=0.0):
    """
    获取机器人的陀螺仪数据（角速度、加速度、姿态）
    返回更新后的陀螺仪数据字典
    """
    global gyro_data, gyro_history, gyro_record_enabled
    
    # 获取机器人基座的位置和姿态
    pos, orn = p.getBasePositionAndOrientation(robot)
    
    # 获取机器人基座的速度和角速度
    vel, ang_vel = p.getBaseVelocity(robot)
    
    # 获取线性加速度（通过速度变化计算）
    # 注意：PyBullet不直接提供加速度，这里使用速度作为近似
    linear_vel = vel
    
    # 将四元数转换为欧拉角
    euler = p.getEulerFromQuaternion(orn)
    
    # 更新陀螺仪数据
    gyro_data['angular_velocity'] = list(ang_vel)  # 角速度 (rad/s)
    gyro_data['linear_acceleration'] = list(linear_vel)  # 线性速度作为加速度近似
    gyro_data['orientation'] = list(euler)  # 姿态角 (roll, pitch, yaw)
    gyro_data['orientation_quat'] = list(orn)  # 四元数姿态
    
    # 记录历史数据（如果启用）
    if gyro_record_enabled:
        gyro_history['time'].append(t)
        gyro_history['angular_velocity'].append(list(ang_vel))
        gyro_history['linear_acceleration'].append(list(linear_vel))
        gyro_history['orientation'].append(list(euler))
        
        # 限制历史记录数量
        if len(gyro_history['time']) > gyro_max_history:
            gyro_history['time'].pop(0)
            gyro_history['angular_velocity'].pop(0)
            gyro_history['linear_acceleration'].pop(0)
            gyro_history['orientation'].pop(0)
    
    return gyro_data

# 陀螺仪数据分析函数
def analyze_gyro_data():
    """
    分析陀螺仪数据，计算统计信息
    """
    if not gyro_history['time']:
        return None
    
    # 计算角速度统计
    ang_vel_data = gyro_history['angular_velocity']
    ang_vel_x = [data[0] for data in ang_vel_data]
    ang_vel_y = [data[1] for data in ang_vel_data]
    ang_vel_z = [data[2] for data in ang_vel_data]
    
    # 计算线性加速度统计
    lin_acc_data = gyro_history['linear_acceleration']
    lin_acc_x = [data[0] for data in lin_acc_data]
    lin_acc_y = [data[1] for data in lin_acc_data]
    lin_acc_z = [data[2] for data in lin_acc_data]
    
    analysis = {
        'angular_velocity': {
            'x': {'mean': sum(ang_vel_x)/len(ang_vel_x), 'max': max(ang_vel_x), 'min': min(ang_vel_x)},
            'y': {'mean': sum(ang_vel_y)/len(ang_vel_y), 'max': max(ang_vel_y), 'min': min(ang_vel_y)},
            'z': {'mean': sum(ang_vel_z)/len(ang_vel_z), 'max': max(ang_vel_z), 'min': min(ang_vel_z)}
        },
        'linear_acceleration': {
            'x': {'mean': sum(lin_acc_x)/len(lin_acc_x), 'max': max(lin_acc_x), 'min': min(lin_acc_x)},
            'y': {'mean': sum(lin_acc_y)/len(lin_acc_y), 'max': max(lin_acc_y), 'min': min(lin_acc_y)},
            'z': {'mean': sum(lin_acc_z)/len(lin_acc_z), 'max': max(lin_acc_z), 'min': min(lin_acc_z)}
        }
    }
    
    return analysis

# PID平衡补偿函数
def apply_balance_compensation(orientation, angular_velocity, dt):
    """
    应用PID平衡补偿
    orientation: 当前姿态角 [roll, pitch, yaw]
    angular_velocity: 当前角速度 [roll_vel, pitch_vel, yaw_vel]
    dt: 时间步长
    返回: 姿态补偿量 [roll_comp, pitch_comp, yaw_comp]
    """
    global balance_controller, balance_enabled
    
    if not balance_enabled:
        return [0.0, 0.0, 0.0]
    
    # 获取PID平衡补偿
    compensation = balance_controller.update(orientation, angular_velocity, dt)
    return compensation

# 简化的轮子驱动控制函数
def apply_simple_drive(speed_input):
    """
    简化的轮子驱动控制 - 只支持前进后退
    speed_input: 速度输入
    返回: (left_speed, right_speed)
    """
    # 简单的差速驱动：左右轮子速度相同
    left_speed = speed_input
    right_speed = speed_input
    
    return left_speed, right_speed

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
    # 设置关节到屈膝稳定位置 - 支持独立腿部控制
    for joint_name, joint_id in joint_indices.items():
        if 'hip' in joint_name:
            # 髋关节角度：使用伸腿值 + 高度调整 + 独立腿部抬起
            if 'FL' in joint_name:  # 左前腿
                base_hip, _ = extend_to_joint_angles(FL_EXTEND, 'front')
                hip_angle = base_hip + height_adjustment - leg_lifts['FL']
            elif 'FR' in joint_name:  # 右前腿
                base_hip, _ = extend_to_joint_angles(FR_EXTEND, 'front')
                hip_angle = base_hip + height_adjustment - leg_lifts['FR']
            elif 'RL' in joint_name:  # 左后腿
                base_hip, _ = extend_to_joint_angles(RL_EXTEND, 'rear')
                hip_angle = base_hip - height_adjustment - leg_lifts['RL']
            elif 'RR' in joint_name:  # 右后腿
                base_hip, _ = extend_to_joint_angles(RR_EXTEND, 'rear')
                hip_angle = base_hip - height_adjustment - leg_lifts['RR']
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=hip_angle, force=30)
        elif 'knee' in joint_name:
            # 膝盖关节角度：使用伸腿值 + 高度调整 + 独立腿部抬起
            if 'FL' in joint_name:  # 左前腿膝盖
                _, base_knee = extend_to_joint_angles(FL_EXTEND, 'front')
                knee_angle = base_knee - height_adjustment + leg_lifts['FL'] * 0.5
            elif 'FR' in joint_name:  # 右前腿膝盖
                _, base_knee = extend_to_joint_angles(FR_EXTEND, 'front')
                knee_angle = base_knee - height_adjustment + leg_lifts['FR'] * 0.5
            elif 'RL' in joint_name:  # 左后腿膝盖
                _, base_knee = extend_to_joint_angles(RL_EXTEND, 'rear')
                knee_angle = base_knee + height_adjustment + leg_lifts['RL'] * 0.5
            elif 'RR' in joint_name:  # 右后腿膝盖
                _, base_knee = extend_to_joint_angles(RR_EXTEND, 'rear')
                knee_angle = base_knee + height_adjustment + leg_lifts['RR'] * 0.5
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=knee_angle, force=30)
    
    # 停止所有轮子
    for wheel_name, wheel_id in wheel_indices.items():
        p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                               targetVelocity=0.0, force=50)

# 简化的姿态控制函数 - 只支持独立腿部控制
def apply_simple_pose_control():
    """
    简化的姿态控制：基础姿态 + 独立腿部控制 + PID平衡补偿
    """
    # 获取当前姿态和角速度
    pos, orn = p.getBasePositionAndOrientation(robot)
    vel, ang_vel = p.getBaseVelocity(robot)
    euler = p.getEulerFromQuaternion(orn)
    
    # 应用PID平衡补偿
    balance_compensation = apply_balance_compensation(euler, ang_vel, time_step)
    roll_comp, pitch_comp, yaw_comp = balance_compensation
    
    # 应用简化控制到所有关节
    for joint_name, joint_id in joint_indices.items():
        if 'hip' in joint_name:
            # 基础髋关节角度 + 高度调整 + 独立腿部抬起 + PID补偿
            if 'FL' in joint_name:
                base_hip, _ = extend_to_joint_angles(FL_EXTEND, 'front')
                hip_angle = base_hip + height_adjustment - leg_lifts['FL'] + roll_comp + pitch_comp
            elif 'FR' in joint_name:
                base_hip, _ = extend_to_joint_angles(FR_EXTEND, 'front')
                hip_angle = base_hip + height_adjustment - leg_lifts['FR'] + roll_comp + pitch_comp
            elif 'RL' in joint_name:
                base_hip, _ = extend_to_joint_angles(RL_EXTEND, 'rear')
                hip_angle = base_hip - height_adjustment - leg_lifts['RL'] + roll_comp + pitch_comp
            elif 'RR' in joint_name:
                base_hip, _ = extend_to_joint_angles(RR_EXTEND, 'rear')
                hip_angle = base_hip - height_adjustment - leg_lifts['RR'] + roll_comp + pitch_comp
            
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                   targetPosition=hip_angle, force=30)
                                   
        elif 'knee' in joint_name:
            # 基础膝盖关节角度 + 高度调整 + 独立腿部抬起 + PID补偿
            if 'FL' in joint_name:
                _, base_knee = extend_to_joint_angles(FL_EXTEND, 'front')
                knee_angle = base_knee - height_adjustment + leg_lifts['FL'] * 0.5 + roll_comp + pitch_comp
            elif 'FR' in joint_name:
                _, base_knee = extend_to_joint_angles(FR_EXTEND, 'front')
                knee_angle = base_knee - height_adjustment + leg_lifts['FR'] * 0.5 + roll_comp + pitch_comp
            elif 'RL' in joint_name:
                _, base_knee = extend_to_joint_angles(RL_EXTEND, 'rear')
                knee_angle = base_knee + height_adjustment + leg_lifts['RL'] * 0.5 + roll_comp + pitch_comp
            elif 'RR' in joint_name:
                _, base_knee = extend_to_joint_angles(RR_EXTEND, 'rear')
                knee_angle = base_knee + height_adjustment + leg_lifts['RR'] * 0.5 + roll_comp + pitch_comp
            
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                   targetPosition=knee_angle, force=30)

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
                hip_angle = base_hip + height_adjustment
            elif 'FR' in joint_name:  # 右前腿
                base_hip, _ = extend_to_joint_angles(fr_extend, 'front')
                hip_angle = base_hip + height_adjustment
            elif 'RL' in joint_name:  # 左后腿
                base_hip, _ = extend_to_joint_angles(rl_extend, 'rear')
                hip_angle = base_hip - height_adjustment
            elif 'RR' in joint_name:  # 右后腿
                base_hip, _ = extend_to_joint_angles(rr_extend, 'rear')
                hip_angle = base_hip - height_adjustment
            
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                   targetPosition=hip_angle, force=30)
                                   
        elif 'knee' in joint_name:
            if 'FL' in joint_name:  # 左前腿膝盖
                _, base_knee = extend_to_joint_angles(fl_extend, 'front')
                knee_angle = base_knee - height_adjustment
            elif 'FR' in joint_name:  # 右前腿膝盖
                _, base_knee = extend_to_joint_angles(fr_extend, 'front')
                knee_angle = base_knee - height_adjustment
            elif 'RL' in joint_name:  # 左后腿膝盖
                _, base_knee = extend_to_joint_angles(rl_extend, 'rear')
                knee_angle = base_knee + height_adjustment
            elif 'RR' in joint_name:  # 右后腿膝盖
                _, base_knee = extend_to_joint_angles(rr_extend, 'rear')
                knee_angle = base_knee + height_adjustment
            
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                   targetPosition=knee_angle, force=30)

# PyBullet键盘事件检测 - 支持连续按键
def check_keyboard_input():
    global current_speed, height_adjustment, current_mode, balance_enabled, leg_lifts
    
    # 获取键盘事件
    keys = p.getKeyboardEvents()
    
    # 重置移动状态
    current_speed = 0.0
    
    # 检测按键状态（支持连续按键）
    # 前进/后退
    if ord('i') in keys:  # 前进
        current_speed = max_speed
    elif ord('k') in keys:  # 后退
        current_speed = -max_speed
    
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
    
    
    # 独立腿部控制
    # 左前腿 (FL) - Q/A键
    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:  # Q键 - 左前腿抬起
        leg_lifts['FL'] += leg_lift_speed
        print(f"左前腿抬起: +{leg_lift_speed:.2f}, 当前: {leg_lifts['FL']:.2f}")
    elif ord('a') in keys and keys[ord('a')] & p.KEY_WAS_TRIGGERED:  # A键 - 左前腿放下
        leg_lifts['FL'] -= leg_lift_speed
        leg_lifts['FL'] = max(0.0, leg_lifts['FL'])  # 不能低于0
        print(f"左前腿放下: -{leg_lift_speed:.2f}, 当前: {leg_lifts['FL']:.2f}")
    
    # 右前腿 (FR) - W/S键
    if ord('w') in keys and keys[ord('w')] & p.KEY_WAS_TRIGGERED:  # W键 - 右前腿抬起
        leg_lifts['FR'] += leg_lift_speed
        print(f"右前腿抬起: +{leg_lift_speed:.2f}, 当前: {leg_lifts['FR']:.2f}")
    elif ord('s') in keys and keys[ord('s')] & p.KEY_WAS_TRIGGERED:  # S键 - 右前腿放下
        leg_lifts['FR'] -= leg_lift_speed
        leg_lifts['FR'] = max(0.0, leg_lifts['FR'])  # 不能低于0
        print(f"右前腿放下: -{leg_lift_speed:.2f}, 当前: {leg_lifts['FR']:.2f}")
    
    # 左后腿 (RL) - E/D键
    if ord('e') in keys and keys[ord('e')] & p.KEY_WAS_TRIGGERED:  # E键 - 左后腿抬起
        leg_lifts['RL'] += leg_lift_speed
        print(f"左后腿抬起: +{leg_lift_speed:.2f}, 当前: {leg_lifts['RL']:.2f}")
    elif ord('d') in keys and keys[ord('d')] & p.KEY_WAS_TRIGGERED:  # D键 - 左后腿放下
        leg_lifts['RL'] -= leg_lift_speed
        leg_lifts['RL'] = max(0.0, leg_lifts['RL'])  # 不能低于0
        print(f"左后腿放下: -{leg_lift_speed:.2f}, 当前: {leg_lifts['RL']:.2f}")
    
    # 右后腿 (RR) - R/F键
    if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:  # R键 - 右后腿抬起
        leg_lifts['RR'] += leg_lift_speed
        print(f"右后腿抬起: +{leg_lift_speed:.2f}, 当前: {leg_lifts['RR']:.2f}")
    elif ord('f') in keys and keys[ord('f')] & p.KEY_WAS_TRIGGERED:  # F键 - 右后腿放下
        leg_lifts['RR'] -= leg_lift_speed
        leg_lifts['RR'] = max(0.0, leg_lifts['RR'])  # 不能低于0
        print(f"右后腿放下: -{leg_lift_speed:.2f}, 当前: {leg_lifts['RR']:.2f}")
    
    # 平衡器控制
    if ord('b') in keys and keys[ord('b')] & p.KEY_WAS_TRIGGERED:  # B键 - 切换平衡器
        balance_enabled = not balance_enabled
        if balance_enabled:
            balance_controller.enable()
            print("PID平衡器已启用")
        else:
            balance_controller.disable()
            print("PID平衡器已禁用")
    
    # 停止键（按下时立即停止）
    if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:  # 空格键停止
        current_speed = 0.0

# 轮子驱动控制 - 简化版本
def wheel_drive_control(t):
    global current_speed
    
    # 检测键盘输入
    check_keyboard_input()
    
    # 根据模式执行不同的控制
    if current_mode == 1:
        # 静止模式：轮子驱动 + 简化姿态控制
        # 使用简化的驱动函数计算轮子速度
        left_speed, right_speed = apply_simple_drive(current_speed)
        
        # 应用轮子速度（前后轮使用相同的左右速度）
        wheel_controls = {
            'FL_wheel_joint': left_speed,   # 左前轮
            'FR_wheel_joint': right_speed,  # 右前轮
            'RL_wheel_joint': left_speed,   # 左后轮
            'RR_wheel_joint': right_speed   # 右后轮
        }
        
        for wheel_name, wheel_id in wheel_indices.items():
            if wheel_name in wheel_controls:
                p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                                       targetVelocity=wheel_controls[wheel_name], force=50)
        
        # 使用简化的姿态控制（基础姿态 + 独立腿部控制 + PID平衡补偿）
        apply_simple_pose_control()
                                       
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
print("=== 基本移动控制 ===")
print("I - 前进")
print("K - 后退") 
print("空格键 - 停止")
print("=== 姿态调整 ===")
print("U - 升高机器人")
print("O - 降低机器人")
print("B - 切换PID平衡器")
print("=== 独立腿部控制 ===")
print("Q/A - 左前腿抬起/放下")
print("W/S - 右前腿抬起/放下")
print("E/D - 左后腿抬起/放下")
print("R/F - 右后腿抬起/放下")
print("=== 独立腿部控制特性 ===")
print("- 每条腿可以独立控制抬起和放下")
print("- 髋关节主导抬起，膝盖关节配合弯曲")
print("- 抬起量可以累积，按A/S/D/F键可以放下")
print("- 支持复杂的腿部姿态组合")
print("=== PID平衡器特性 ===")
print("- 分层控制：基础姿态 + PID平衡补偿")
print("- 自动维持机器人平衡，防止倾倒")
print("- 只输出补偿角度，不直接控制关节")
print("- 可实时启用/禁用平衡器")
print("=== 传感器数据 ===")
print("陀螺仪数据每3秒显示一次，包括:")
print("- 角速度 (X, Y, Z轴)")
print("- 线性加速度 (X, Y, Z轴)")
print("- 姿态角 (滚转, 俯仰, 偏航)")
print("=== 其他 ===")
print("按 Ctrl+C 退出")

# 主仿真循环
try:
    t = 0
    last_status_time = 0
    
    while True:
        # 执行轮子驱动控制
        wheel_drive_control(t)
        
        # 获取陀螺仪数据
        gyro_data = get_gyroscope_data(t)
        
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
                    status = "前进"
                elif current_speed < 0:
                    status = "后退"
                else:
                    status = "停止"
            else:  # 踏步模式
                status = "踏步中"
            
            # 显示姿态调整状态
            height_status = f"高度偏移: {height_adjustment:.2f}"
            balance_status = f"PID平衡: {'启用' if balance_enabled else '禁用'}"
            
            # 显示独立腿部控制状态
            leg_status = f"腿部抬起: FL={leg_lifts['FL']:.2f}, FR={leg_lifts['FR']:.2f}, RL={leg_lifts['RL']:.2f}, RR={leg_lifts['RR']:.2f}"
            
            # 陀螺仪数据
            gyro_ang_vel = gyro_data['angular_velocity']
            gyro_lin_acc = gyro_data['linear_acceleration']
            gyro_orient = gyro_data['orientation']
            
            # 分析陀螺仪数据
            gyro_analysis = analyze_gyro_data()
            
            print(f"时间: {t:.1f}s | 模式: {mode_name} | 状态: {status}")
            print(f"位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"速度: ({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f})")
            print(f"姿态: 滚转={euler[0]:.2f}, 俯仰={euler[1]:.2f}, 偏航={euler[2]:.2f}")
            print(f"陀螺仪角速度: X={gyro_ang_vel[0]:.3f}, Y={gyro_ang_vel[1]:.3f}, Z={gyro_ang_vel[2]:.3f} rad/s")
            print(f"陀螺仪线性加速度: X={gyro_lin_acc[0]:.3f}, Y={gyro_lin_acc[1]:.3f}, Z={gyro_lin_acc[2]:.3f} m/s²")
            print(f"陀螺仪姿态: 滚转={gyro_orient[0]:.3f}, 俯仰={gyro_orient[1]:.3f}, 偏航={gyro_orient[2]:.3f} rad")
            
            # 显示陀螺仪数据分析（如果有足够数据）
            if gyro_analysis and len(gyro_history['time']) > 10:
                print("=== 陀螺仪数据分析 ===")
                ang_vel_stats = gyro_analysis['angular_velocity']
                lin_acc_stats = gyro_analysis['linear_acceleration']
                print(f"角速度统计 - X: 均值={ang_vel_stats['x']['mean']:.3f}, 范围=[{ang_vel_stats['x']['min']:.3f}, {ang_vel_stats['x']['max']:.3f}]")
                print(f"角速度统计 - Y: 均值={ang_vel_stats['y']['mean']:.3f}, 范围=[{ang_vel_stats['y']['min']:.3f}, {ang_vel_stats['y']['max']:.3f}]")
                print(f"角速度统计 - Z: 均值={ang_vel_stats['z']['mean']:.3f}, 范围=[{ang_vel_stats['z']['min']:.3f}, {ang_vel_stats['z']['max']:.3f}]")
                print(f"线性加速度统计 - X: 均值={lin_acc_stats['x']['mean']:.3f}, 范围=[{lin_acc_stats['x']['min']:.3f}, {lin_acc_stats['x']['max']:.3f}]")
                print(f"线性加速度统计 - Y: 均值={lin_acc_stats['y']['mean']:.3f}, 范围=[{lin_acc_stats['y']['min']:.3f}, {lin_acc_stats['y']['max']:.3f}]")
                print(f"线性加速度统计 - Z: 均值={lin_acc_stats['z']['mean']:.3f}, 范围=[{lin_acc_stats['z']['min']:.3f}, {lin_acc_stats['z']['max']:.3f}]")
            
            print(f"{height_status} | {balance_status}")
            print(leg_status)
            
            # 显示PID平衡器调试信息
            if balance_enabled:
                debug_info = balance_controller.get_debug_info()
                print(f"PID补偿: Roll={debug_info['roll_compensation']:.3f}, Pitch={debug_info['pitch_compensation']:.3f}, Yaw={debug_info['yaw_compensation']:.3f}")
                print(f"PID误差: Roll={debug_info['roll_error']:.3f}, Pitch={debug_info['pitch_error']:.3f}, Yaw={debug_info['yaw_error']:.3f}")
            
            print(f"陀螺仪数据记录: {len(gyro_history['time'])}/{gyro_max_history} 条记录")
            print("-" * 50)
            
            last_status_time = t

except KeyboardInterrupt:
    print("\n仿真已停止")
finally:
    p.disconnect()