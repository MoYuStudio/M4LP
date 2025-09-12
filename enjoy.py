import time
import pybullet as p
from stable_baselines3 import PPO
from m4lp_env import M4LPEnv

model = PPO.load("m4lp_ppo_final")
env = M4LPEnv(render_mode="human")
obs, _ = env.reset()
while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        obs, _ = env.reset()
    time.sleep(0.02)
