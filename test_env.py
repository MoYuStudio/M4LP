#!/usr/bin/env python3
"""
测试 M4LP 环境是否正常工作
"""
import time
from m4lp_env import M4LPEnv

def test_env():
    print("正在测试 M4LP 环境...")
    
    # 创建环境
    env = M4LPEnv(render_mode="human")
    print("✓ 环境创建成功")
    
    # 重置环境
    obs, info = env.reset()
    print(f"✓ 环境重置成功，观测维度: {obs.shape}")
    
    # 运行几步随机动作
    print("运行 100 步随机动作...")
    for i in range(100):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        if i % 20 == 0:
            print(f"  步骤 {i}: 奖励 = {reward:.3f}, 终止 = {terminated}")
        
        if terminated or truncated:
            print("  环境重置...")
            obs, info = env.reset()
    
    # 关闭环境
    env.close()
    print("✓ 环境测试完成")

if __name__ == "__main__":
    test_env()
