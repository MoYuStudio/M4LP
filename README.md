# M4LP 四足机器人强化学习

基于 PyBullet 和 Stable-Baselines3 的四足机器人强化学习训练方案。

## 文件说明

- `m4lp.urdf` - 四足机器人 URDF 模型文件
- `m4lp_env.py` - 自定义 Gym 环境
- `train.py` - 训练脚本
- `enjoy.py` - 可视化回放脚本
- `test_env.py` - 环境测试脚本
- `requirements.txt` - 依赖包列表

## 安装依赖

```bash
pip install -r requirements.txt
```

或者手动安装：

```bash
pip install stable-baselines3[extra] pybullet gymnasium numpy tensorboard
```

## 使用方法

### 1. 测试环境

```bash
python test_env.py
```

### 2. 开始训练

```bash
python train.py
```

训练过程会：
- 创建 8 个并行环境
- 使用 PPO 算法训练 500 万步
- 每 5 万步保存一次检查点到 `./ckpt/` 目录
- 训练日志保存到 `./tb/` 目录

### 3. 查看训练曲线

```bash
tensorboard --logdir tb
```

然后在浏览器中打开 `http://localhost:6006`

### 4. 观看训练后的策略

```bash
python enjoy.py
```

## 环境参数

- **观测空间**: 33 维
  - 基座速度 (vx, vy): 2 维
  - 偏航角速度: 1 维
  - 基座姿态 (sin/cos): 6 维
  - 关节位置: 12 维
  - 关节速度: 12 维
  - 目标速度命令: 3 维

- **动作空间**: 12 维关节位置增量 [-0.3, 0.3]

- **奖励函数**:
  - 速度匹配奖励: 1.2 × 速度匹配度
  - 姿态奖励: 0.05 × 躯干直立奖励
  - 能耗惩罚: -0.8 × 关节能耗
  - 关节限制惩罚: -0.5 × 关节限制违反
  - 跌倒惩罚: -1.0 × 跌倒

## 训练建议

1. **快速收敛**: 1M 步左右即可看到机器人小跑
2. **稳定训练**: 5M 步速度 0.5 m/s 基本稳定
3. **调参建议**:
   - 如果震荡明显，降低动作幅度到 0.2
   - 或增加 gamma 到 0.995
   - 想提速可把 n_steps=4096→2048，batch_size=256→128

## 扩展方向

1. **扭矩控制**: 将 PD 控制换成直接扭矩控制
2. **地形适应**: 加入高度图进行地形适应训练
3. **鲁棒性**: 加入外力扰动训练
4. **真实机器人**: 使用 `pybullet.setRealTimeSimulation(1)` 连接真实硬件

## 故障排除

如果遇到问题：

1. 确保 PyBullet 正确安装
2. 检查 URDF 文件路径
3. 确保有足够的 GPU 内存（训练时）
4. 如果训练不稳定，尝试降低学习率或调整奖励函数权重
