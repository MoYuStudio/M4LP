# 核心控制程序
# 本文件纯人工编写，禁止AI修改续写

import time

import pybullet
import pybullet_data

import xbox_gamepad
from pid import BalanceController

pad = xbox_gamepad.XboxGamepad()
st = pad.read()

# 连接到物理服务器
pybullet.connect(pybullet.GUI)
# pybullet自带模型库挂载
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置重力
pybullet.setGravity(0, 0, -9.81*2)

# 加载地面
pybullet.loadURDF("plane.urdf")
# 加载机器人
robot = pybullet.loadURDF("m4lp.urdf",basePosition=[0, 0, 0.35],useFixedBase=False)

# 初始化平衡控制器
balance_controller = BalanceController()

# 关节索引
joint_indices = {pybullet.getJointInfo(robot, j)[1].decode(): j for j in range(pybullet.getNumJoints(robot))}
joint = {n: i for n, i in joint_indices.items()
          if any(k in n for k in ('hip', 'knee', 'wheel')) and 'marker' not in n}
# {'FL_hip_joint': 0, 
#  'FL_knee_joint': 3, 
#  'FL_wheel_joint': 5, 
#  'FR_hip_joint': 6, 
#  'FR_knee_joint': 9, 
#  'FR_wheel_joint': 11, 
#  'RL_hip_joint': 12, 
#  'RL_knee_joint': 15, 
#  'RL_wheel_joint': 17, 
#  'RR_hip_joint': 18, 
#  'RR_knee_joint': 21, 
#  'RR_wheel_joint': 23}

# ====== 关节默认弯曲 ======
base_pose = {
    'FL_hip_joint':  0.9, 'FL_knee_joint': -1.5,
    'FR_hip_joint':  0.9, 'FR_knee_joint': -1.5,
    'RL_hip_joint': -0.9, 'RL_knee_joint':  1.5,
    'RR_hip_joint': -0.9, 'RR_knee_joint':  1.5,
}
for name, pos in base_pose.items():
    pybullet.setJointMotorControl2(robot, joint[name],
                                  pybullet.POSITION_CONTROL,
                                  targetPosition=pos,
                                  force=30)
# 让仿真稳定几帧
for _ in range(100):
    pybullet.stepSimulation()
    time.sleep(0.02)

# -------------- 主循环 --------------
while True:
    st = pad.read()
    
    # 获取机器人当前姿态和角速度
    robot_pos, robot_orn = pybullet.getBasePositionAndOrientation(robot)
    robot_vel, robot_ang_vel = pybullet.getBaseVelocity(robot)
    
    # 将四元数转换为欧拉角 (roll, pitch, yaw)
    euler_angles = pybullet.getEulerFromQuaternion(robot_orn)
    current_orientation = [euler_angles[0], euler_angles[1], euler_angles[2]]  # [roll, pitch, yaw]
    current_angular_velocity = [robot_ang_vel[0], robot_ang_vel[1], robot_ang_vel[2]]  # [roll_vel, pitch_vel, yaw_vel]
    
    # 使用平衡控制器计算姿态补偿
    balance_compensation = balance_controller.update(current_orientation, current_angular_velocity, dt=0.02)
    roll_comp, pitch_comp, yaw_comp = balance_compensation
    
    # 调试信息（可选：打印平衡状态）
    # debug_info = balance_controller.get_debug_info()
    # print(f"Roll: {current_orientation[0]:.3f}, Pitch: {current_orientation[1]:.3f}, Yaw: {current_orientation[2]:.3f}")
    # print(f"Compensation - Roll: {roll_comp:.3f}, Pitch: {pitch_comp:.3f}, Yaw: {yaw_comp:.3f}")
    
    sync_offset = 0.0
    if st is not None:
        if 0x0001 in st['buttons']: sync_offset -= 0.3   # ↑ 更站
        if 0x0002 in st['buttons']: sync_offset += 0.9   # ↓ 更蹲
        rx = st['RX']   # -1..1
        ry = st['RY']   # -1..1
    else:
        rx = ry = 0.0

    # 对每条腿计算：同步偏移 + 右摇杆单侧偏移 + 平衡补偿
    for side in ['FL', 'FR', 'RL', 'RR']:
        lr = rx if 'L' in side else -rx      # 左/右权重
        fb = -ry if 'F' in side else ry      # 前/后权重
        # 只取最大单侧分量，让"推向哪边"哪两条腿更蹲
        side_offset = max(0, lr if abs(lr) > abs(fb) else fb) * 0.6   # 0~0.4 rad

        hip_sign = 1 if side in ['FL', 'FR'] else -1

        # 对角补偿：膝比髋多走 0.5*side_offset（稳定性）
        knee_extra = 0.5 * side_offset * hip_sign
        
        # 计算平衡补偿量
        # roll补偿：左右倾斜时，左右腿补偿相反
        roll_comp_factor = 1.0 if 'R' in side else -1.0  # 右腿为正，左腿为负
        roll_balance_comp = roll_comp * roll_comp_factor * 0.5
        
        # pitch补偿：前后倾斜时，前后腿补偿相反  
        pitch_comp_factor = 1.0 if 'F' in side else -1.0  # 前腿为正，后腿为负
        pitch_balance_comp = pitch_comp * pitch_comp_factor * 0.3
        
        # yaw补偿：旋转时，对角腿补偿
        yaw_comp_factor = 1.0 if side in ['FL', 'RR'] else -1.0  # 对角腿补偿
        yaw_balance_comp = yaw_comp * yaw_comp_factor * 0.2
        
        # 总平衡补偿量
        total_balance_comp = roll_balance_comp + pitch_balance_comp + yaw_balance_comp

        pybullet.setJointMotorControl2(
            robot, joint[side+'_hip_joint'],
            pybullet.POSITION_CONTROL,
            targetPosition=base_pose[side+'_hip_joint'] + hip_sign*(sync_offset + side_offset) + total_balance_comp,
            force=30)
        pybullet.setJointMotorControl2(
            robot, joint[side+'_knee_joint'],
            pybullet.POSITION_CONTROL,
            targetPosition=base_pose[side+'_knee_joint'] - hip_sign*(sync_offset + side_offset) - knee_extra - total_balance_comp,
            force=30)
    
    # 轮子控制：LY 前进 + LX 差速 + 平衡器 yaw 补偿
    if st is None:
        spd_l = spd_r = 0.0
    else:
        fwd  = st['LY'] * 64.0
        turn = st['LX'] * 32.0
        # **加入 yaw 补偿**，让平衡器也能“转轮”
        yaw_corr = yaw_comp * 20.0   # 缩放可调
        if abs(fwd) < 2.0:           # 原地旋转模式
            spd_l = -turn + yaw_corr
            spd_r =  turn + yaw_corr
        else:                        # 前进+差速
            spd_l = fwd - turn + yaw_corr
            spd_r = fwd + turn + yaw_corr

    # 下发到四个轮子
    for side in ['FL', 'FR', 'RL', 'RR']:
        target = spd_l if 'L' in side else spd_r
        pybullet.setJointMotorControl2(robot, joint[side+'_wheel_joint'],
                                     pybullet.VELOCITY_CONTROL,
                                     targetVelocity=target,
                                     force=20)
    
    pybullet.stepSimulation()
    time.sleep(0.02)   # 约 50 Hz