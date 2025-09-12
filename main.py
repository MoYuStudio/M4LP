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

# 获取关节信息
joint_indices = {}
joint_names = {}
for j in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, j)
    joint_name = joint_info[1].decode('utf-8')
    joint_names[j] = joint_name
    if 'hip' in joint_name or 'knee' in joint_name:
        joint_indices[joint_name] = j

print("找到的关节:", joint_indices)

# 行走参数
step_height = 0.1  # 抬腿高度
step_length = 0.2  # 步长
walk_speed = 1.0   # 行走速度
time_step = 0.01   # 时间步长

# 初始姿态 - 站立
def set_standing_pose():
    for joint_name, joint_id in joint_indices.items():
        if 'hip' in joint_name:
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=0.0, force=50)
        elif 'knee' in joint_name:
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=0.5, force=50)

# 简单的四足行走步态
def walking_gait(t):
    # 四足步态相位：FL, FR, RL, RR
    phases = [0, math.pi, math.pi/2, 3*math.pi/2]
    
    for i, (leg, phase) in enumerate([('FL', 0), ('FR', math.pi), ('RL', math.pi/2), ('RR', 3*math.pi/2)]):
        # 计算当前腿的相位
        leg_phase = (t * walk_speed + phase) % (2 * math.pi)
        
        # 髋关节：前后摆动
        hip_angle = 0.3 * math.sin(leg_phase)
        hip_joint = joint_indices.get(f'{leg}_hip_joint')
        if hip_joint is not None:
            p.setJointMotorControl2(robot, hip_joint, p.POSITION_CONTROL, 
                                   targetPosition=hip_angle, force=30)
        
        # 膝盖关节：抬腿动作
        if leg_phase < math.pi:  # 支撑相
            knee_angle = 0.3  # 稍微弯曲
        else:  # 摆动相
            knee_angle = 0.8 + 0.3 * math.sin(leg_phase)  # 抬腿
        
        knee_joint = joint_indices.get(f'{leg}_knee_joint')
        if knee_joint is not None:
            p.setJointMotorControl2(robot, knee_joint, p.POSITION_CONTROL, 
                                   targetPosition=knee_angle, force=30)

# 设置初始站立姿态
set_standing_pose()

# 等待机器人稳定
for _ in range(100):
    p.stepSimulation()
    time.sleep(time_step)

print("开始行走...")

# 主仿真循环
try:
    t = 0
    while True:
        # 执行行走步态
        walking_gait(t)
        
        # 推进仿真
        p.stepSimulation()
        time.sleep(time_step)
        t += time_step
        
        # 每5秒输出一次状态
        if int(t) % 5 == 0 and t - int(t) < time_step:
            pos, orn = p.getBasePositionAndOrientation(robot)
            print(f"时间: {t:.1f}s, 位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

except KeyboardInterrupt:
    print("仿真已停止")
finally:
    p.disconnect()