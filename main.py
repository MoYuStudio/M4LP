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
                   basePosition=[0, 0, 0.4],
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
wheel_speed = 5.0   # 轮子速度
time_step = 0.01    # 时间步长
target_speed = 0.5  # 目标前进速度

# 初始姿态 - 站立稳定
def set_stable_pose():
    # 设置关节到稳定位置
    for joint_name, joint_id in joint_indices.items():
        if 'hip' in joint_name:
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=0.0, force=30)
        elif 'knee' in joint_name:
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=0.3, force=30)
    
    # 停止所有轮子
    for wheel_name, wheel_id in wheel_indices.items():
        p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                               targetVelocity=0.0, force=20)

# 轮子驱动和四足稳定控制
def wheel_drive_control(t):
    # 获取当前机器人状态
    base_pos, base_quat = p.getBasePositionAndOrientation(robot)
    base_vel, base_ang = p.getBaseVelocity(robot)
    euler = p.getEulerFromQuaternion(base_quat)
    
    # 轮子驱动：根据目标速度控制轮子
    wheel_vel = target_speed * 10.0  # 转换为轮子角速度
    for wheel_name, wheel_id in wheel_indices.items():
        p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                               targetVelocity=wheel_vel, force=20)
    
    # 四足稳定控制：根据姿态调整关节位置
    # 如果机器人倾斜，调整关节来保持稳定
    roll_error = euler[0]  # 左右倾斜
    pitch_error = euler[1]  # 前后倾斜
    
    # 根据倾斜调整关节位置
    for joint_name, joint_id in joint_indices.items():
        if 'hip' in joint_name:
            # 髋关节：根据左右倾斜调整
            if 'FL' in joint_name or 'RL' in joint_name:  # 左腿
                hip_angle = -roll_error * 0.5
            else:  # 右腿
                hip_angle = roll_error * 0.5
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                   targetPosition=hip_angle, force=30)
        
        elif 'knee' in joint_name:
            # 膝盖关节：根据前后倾斜调整
            if 'F' in joint_name:  # 前腿
                knee_angle = 0.3 - pitch_error * 0.3
            else:  # 后腿
                knee_angle = 0.3 + pitch_error * 0.3
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL,
                                   targetPosition=knee_angle, force=30)

# 设置初始稳定姿态
set_stable_pose()

# 等待机器人稳定
for _ in range(100):
    p.stepSimulation()
    time.sleep(time_step)

print("开始轮子驱动移动...")

# 主仿真循环
try:
    t = 0
    while True:
        # 执行轮子驱动和稳定控制
        wheel_drive_control(t)
        
        # 推进仿真
        p.stepSimulation()
        time.sleep(time_step)
        t += time_step
        
        # 每5秒输出一次状态
        if int(t) % 5 == 0 and t - int(t) < time_step:
            pos, orn = p.getBasePositionAndOrientation(robot)
            vel, ang_vel = p.getBaseVelocity(robot)
            euler = p.getEulerFromQuaternion(orn)
            print(f"时间: {t:.1f}s, 位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"速度: ({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f})")
            print(f"姿态: 滚转={euler[0]:.2f}, 俯仰={euler[1]:.2f}, 偏航={euler[2]:.2f}")

except KeyboardInterrupt:
    print("仿真已停止")
finally:
    p.disconnect()