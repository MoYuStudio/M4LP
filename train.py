from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback
from m4lp_env import M4LPEnv

def train():
    n_envs = 8
    env = make_vec_env(M4LPEnv, n_envs=n_envs, seed=0)

    model = PPO("MlpPolicy", env, verbose=1,
                tensorboard_log="tb",
                n_steps=4096,
                batch_size=256,
                gamma=0.99,
                gae_lambda=0.95,
                clip_range=0.2,
                ent_coef=0.0,
                learning_rate=3e-4)

    checkpoint_callback = CheckpointCallback(save_freq=50_000,
                                             save_path="./ckpt/",
                                             name_prefix="m4lp")
    model.learn(total_timesteps=5_000_000, callback=checkpoint_callback)
    model.save("m4lp_ppo_final")

if __name__ == "__main__":
    train()
