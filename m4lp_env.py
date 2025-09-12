import numpy as np
import pybullet as p
import pybullet_data
import gymnasium as gym
from gymnasium import spaces
import time

class M4LPEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None):
        super().__init__()
        self.render_mode = render_mode
        self.dt = 1/500          # 仿真步长
        self.ctrl_decimation = 10
        self.action_repeat = self.ctrl_decimation
        self.np_random, _ = gym.utils.seeding.np_random()

        # 观测与动作空间
        # 观测：基础状态(3) + 姿态(6) + 关节位置(8) + 关节速度(8) + 轮子速度(4) + 目标速度(3) = 32维
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(32,), dtype=np.float32)
        # 动作：髋关节(4) + 膝盖关节(4) + 轮子速度(4) = 12维
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(12,), dtype=np.float32)

        self.physics_client_id = None
        self.robot = None
        self.joint_indices = []
        self.wheel_indices = []
        self.target_speed = np.array([0.5, 0, 0])   # 默认前进 0.5 m/s

    def _build_joint_list(self):
        joint_ids = []
        wheel_ids = []
        for j in range(p.getNumJoints(self.robot)):
            name = p.getJointInfo(self.robot, j)[1].decode("utf-8")
            if "hip_joint" in name or "knee_joint" in name:
                joint_ids.append(j)
            elif "wheel_joint" in name:
                wheel_ids.append(j)
        return sorted(joint_ids), sorted(wheel_ids)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.physics_client_id is not None:
            p.disconnect(self.physics_client_id)

        if self.render_mode == "human":
            self.physics_client_id = p.connect(p.GUI)
        else:
            self.physics_client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.dt)
        p.loadURDF("plane.urdf")

        self.robot = p.loadURDF("m4lp.urdf", basePosition=[0, 0, 0.4])
        self.joint_indices, self.wheel_indices = self._build_joint_list()
        
        # 设置轮子的摩擦系数，使其能够在地面上滚动
        for wheel_id in self.wheel_indices:
            p.changeDynamics(self.robot, wheel_id, 
                           lateralFriction=1.0,  # 侧向摩擦
                           spinningFriction=0.1,  # 旋转摩擦
                           rollingFriction=0.01)  # 滚动摩擦
        
        self._stand_still(2.0)   # 先站稳

        obs = self._get_obs()
        info = {}
        return obs, info

    def _stand_still(self, duration):
        for _ in range(int(duration / self.dt)):
            p.stepSimulation()

    def _get_obs(self):
        base_pos, base_quat = p.getBasePositionAndOrientation(self.robot)
        base_vel, base_ang = p.getBaseVelocity(self.robot)
        euler = p.getEulerFromQuaternion(base_quat)

        # 关节位置和速度 (8个关节：4个髋关节 + 4个膝盖关节)
        jpos = np.zeros(8)
        jvel = np.zeros(8)
        for i, jid in enumerate(self.joint_indices):
            jpos[i], jvel[i], *_ = p.getJointState(self.robot, jid)

        # 轮子速度 (4个轮子)
        wheel_vel = np.zeros(4)
        for i, jid in enumerate(self.wheel_indices):
            _, wheel_vel[i], *_ = p.getJointState(self.robot, jid)

        obs = np.concatenate([
            base_vel[:2],                # vx, vy (2)
            [base_ang[2]],               # yaw rate (1)
            np.sin(euler), np.cos(euler), # 姿态 (6)
            jpos, jvel,                  # 关节状态 (16)
            wheel_vel,                   # 轮子速度 (4)
            self.target_speed            # 目标速度 (3)
        ])
        return obs.astype(np.float32)

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        
        # 分解动作：前8个是关节控制，后4个是轮子速度
        joint_actions = action[:8]  # 髋关节(4) + 膝盖关节(4)
        wheel_actions = action[8:]  # 轮子速度(4)
        
        for _ in range(self.action_repeat):
            # 关节PD控制 - 用于稳定姿态
            for i, jid in enumerate(self.joint_indices):
                q = p.getJointState(self.robot, jid)[0]
                q_des = q + joint_actions[i] * 0.1  # 小幅调整
                p.setJointMotorControl2(self.robot, jid, p.POSITION_CONTROL,
                                      targetPosition=q_des, force=30)
            
            # 轮子速度控制 - 用于驱动移动
            for i, jid in enumerate(self.wheel_indices):
                wheel_vel = wheel_actions[i] * 10.0  # 缩放轮子速度
                p.setJointMotorControl2(self.robot, jid, p.VELOCITY_CONTROL,
                                      targetVelocity=wheel_vel, force=20)
            
            p.stepSimulation()
            if self.render_mode == "human":
                time.sleep(self.dt)

        obs = self._get_obs()
        reward, terminated = self._compute_reward()
        truncated = False
        return obs, reward, terminated, truncated, {}

    def _compute_reward(self):
        base_pos, base_quat = p.getBasePositionAndOrientation(self.robot)
        base_vel, _ = p.getBaseVelocity(self.robot)
        euler = p.getEulerFromQuaternion(base_quat)

        # 移动奖励：鼓励按目标速度移动
        vxy = np.array(base_vel[:2])
        v_target = self.target_speed[:2]
        v_match = -np.linalg.norm(vxy - v_target)

        # 稳定性奖励：鼓励保持水平姿态
        stability = 1.0 if abs(euler[0]) < 0.2 and abs(euler[1]) < 0.2 else 0.0

        # 关节能耗：鼓励节能的关节控制
        joint_energy = np.sum(np.abs([p.getJointState(self.robot, j)[3]
                                     for j in self.joint_indices]))

        # 轮子能耗：鼓励合理的轮子使用
        wheel_energy = np.sum(np.abs([p.getJointState(self.robot, j)[3]
                                     for j in self.wheel_indices]))

        # 关节限制惩罚
        joint_limit = 0
        for j in self.joint_indices:
            lim_low, lim_high = p.getJointInfo(self.robot, j)[8:10]
            q = p.getJointState(self.robot, j)[0]
            if q < lim_low + 0.1 or q > lim_high - 0.1:
                joint_limit += 1

        # 摔倒检测
        fall = base_pos[2] < 0.15 or abs(euler[0]) > 1.0 or abs(euler[1]) > 1.0

        # 综合奖励函数
        reward = (2.0 * v_match                    # 移动匹配 (主要)
                  + 0.3 * stability                # 稳定性
                  - 0.1 * joint_energy * 1e-3     # 关节能耗
                  - 0.05 * wheel_energy * 1e-3    # 轮子能耗
                  - 0.2 * joint_limit * 0.1       # 关节限制
                  - 2.0 * fall)                   # 摔倒惩罚
        return reward, fall

    def close(self):
        if self.physics_client_id is not None:
            p.disconnect(self.physics_client_id)
            self.physics_client_id = None


if __name__ == "__main__":
    # 快速自检
    import time
    env = M4LPEnv(render_mode="human")
    env.reset()
    for _ in range(200):
        env.step(env.action_space.sample())
    env.close()
