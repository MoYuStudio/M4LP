# 核心控制程序
# 本文件纯人工编写，禁止AI修改续写

import time

import pybullet
import pybullet_data

import xbox_gamepad

pad = xbox_gamepad.XboxGamepad()
st = pad.read()

# 连接到物理服务器
pybullet.connect(pybullet.GUI)
# pybullet自带模型库挂载
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置重力
pybullet.setGravity(0, 0, -9.81)

# 加载地面
pybullet.loadURDF("plane.urdf")
# 加载机器人
robot = pybullet.loadURDF("m4lp.urdf",basePosition=[0, 0, 0.35],useFixedBase=False)

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
    
    sync_offset = 0.0
    if st is not None:
        if 0x0001 in st['buttons']: sync_offset -= 0.3   # ↑ 更站
        if 0x0002 in st['buttons']: sync_offset += 0.9   # ↓ 更蹲
        rx = st['RX']   # -1..1
        ry = st['RY']   # -1..1
    else:
        rx = ry = 0.0

    # 对每条腿计算：同步偏移 + 右摇杆单侧偏移
    for side in ['FL', 'FR', 'RL', 'RR']:
        lr = -rx if 'L' in side else rx      # 左/右权重
        fb = -ry if 'F' in side else ry      # 前/后权重
        # 只取最大单侧分量，让“推向哪边”哪两条腿更蹲
        side_offset = max(0, lr if abs(lr) > abs(fb) else fb) * 0.6   # 0~0.4 rad

        hip_sign = 1 if side in ['FL', 'FR'] else -1

        # 对角补偿：膝比髋多走 0.5*side_offset（稳定性）
        knee_extra = 0.5 * side_offset * hip_sign

        pybullet.setJointMotorControl2(
            robot, joint[side+'_hip_joint'],
            pybullet.POSITION_CONTROL,
            targetPosition=base_pose[side+'_hip_joint'] + hip_sign*(sync_offset + side_offset),
            force=30)
        pybullet.setJointMotorControl2(
            robot, joint[side+'_knee_joint'],
            pybullet.POSITION_CONTROL,
            targetPosition=base_pose[side+'_knee_joint'] - hip_sign*(sync_offset + side_offset) - knee_extra,
            force=30)
    
    # 轮子控制
    spd = 0.0 if st is None else st['LY'] * 64.0   # 满推 64 rad/s
    for side in ['FL', 'FR', 'RL', 'RR']:
        pybullet.setJointMotorControl2(robot, joint[side+'_wheel_joint'],
                                     pybullet.VELOCITY_CONTROL,
                                     targetVelocity=spd,
                                     force=20)
    pybullet.stepSimulation()
    time.sleep(0.02)   # 约 50 Hz