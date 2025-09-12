#!/usr/bin/env python3
"""
轮子滚动演示
展示M4LP机器人的轮子驱动和四足稳定功能
"""

import pybullet as p
import pybullet_data
import time
import numpy as np

def demo_wheel_rolling():
    # 连接到物理服务器
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 设置重力
    p.setGravity(0, 0, -9.81)
    
    # 加载地面
    p.loadURDF("plane.urdf")
    
    # 加载机器人
    robot = p.loadURDF("m4lp.urdf", basePosition=[0, 0, 0.4])
    
    # 设置轮子的摩擦系数
    wheel_indices = []
    joint_indices = []
    for j in range(p.getNumJoints(robot)):
        joint_info = p.getJointInfo(robot, j)
        joint_name = joint_info[1].decode('utf-8')
        if 'wheel' in joint_name:
            wheel_indices.append(j)
            p.changeDynamics(robot, j, 
                            lateralFriction=1.0,   # 侧向摩擦
                            spinningFriction=0.1,  # 旋转摩擦
                            rollingFriction=0.01)  # 滚动摩擦
        elif 'hip' in joint_name or 'knee' in joint_name:
            joint_indices.append(j)
    
    print("M4LP 轮子驱动机器人演示")
    print("=" * 40)
    
    # 设置初始稳定姿态
    for joint_id in joint_indices:
        joint_info = p.getJointInfo(robot, joint_id)
        joint_name = joint_info[1].decode('utf-8')
        if 'hip' in joint_name:
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=0.0, force=30)
        elif 'knee' in joint_name:
            p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 
                                   targetPosition=0.3, force=30)
    
    # 等待稳定
    print("机器人正在稳定...")
    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.01)
    
    print("开始演示...")
    
    # 演示1：直线前进
    print("\n演示1：直线前进")
    for wheel_id in wheel_indices:
        p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                               targetVelocity=3.0, force=20)
    
    for i in range(200):
        p.stepSimulation()
        time.sleep(0.01)
        if i % 50 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot)
            print(f"位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
    
    # 停止
    for wheel_id in wheel_indices:
        p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                               targetVelocity=0.0, force=20)
    
    # 等待停止
    for _ in range(50):
        p.stepSimulation()
        time.sleep(0.01)
    
    # 演示2：原地转向
    print("\n演示2：原地转向")
    for i, wheel_id in enumerate(wheel_indices):
        if i % 2 == 0:  # 左轮
            p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                                   targetVelocity=-2.0, force=20)
        else:  # 右轮
            p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                                   targetVelocity=2.0, force=20)
    
    for i in range(150):
        p.stepSimulation()
        time.sleep(0.01)
        if i % 30 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot)
            euler = p.getEulerFromQuaternion(orn)
            print(f"位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), 偏航: {euler[2]:.2f}")
    
    # 停止
    for wheel_id in wheel_indices:
        p.setJointMotorControl2(robot, wheel_id, p.VELOCITY_CONTROL,
                               targetVelocity=0.0, force=20)
    
    print("\n演示完成！")
    print("特点：")
    print("- 轮子在地面上滚动，推动机器人移动")
    print("- 四足关节保持机器人稳定")
    print("- 支持直线移动和转向")
    
    # 保持窗口打开
    input("按回车键退出...")
    p.disconnect()

if __name__ == "__main__":
    demo_wheel_rolling()
