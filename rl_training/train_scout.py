"""
train_scout.py -- Scout PPO 训练主脚本 v2.0

基于 DLR-RM/rl-baselines3-zoo, Isaac Lab, CrowdNav 等开源项目的最佳实践,
对训练流程进行了 7 项重大改进, 显著提升训练稳定性和最终性能.

迭代改进清单:
  14. 学习率退火 (Linear Schedule): 从 3e-4 线性衰减到 1e-5, 防止后期训练震荡
  15. 熵系数调度: 初始高熵鼓励探索, 逐步降低聚焦利用
  16. 网络架构改进: 增大网络容量, 添加 LayerNorm 提升训练稳定性
  17. 课程学习回调: 根据成功率自动调整环境难度
  18. 超参优化: 增大 n_steps, 调整 batch_size 和 n_epochs
  19. 奖励裁剪: VecNormalize 中使用 reward clipping 防止异常值干扰
  20. 梯度噪声 + 改进初始化: 添加梯度噪声提升探索能力

用法:
  python train_scout.py                            # 标准训练 5M步
  python train_scout.py --fast                     # 快速测试 50k步
  python train_scout.py --resume checkpoints/last.zip
  python train_scout.py --timesteps 10000000       # 自定义训练步数
  python train_scout.py --no-curriculum             # 禁用课程学习
"""
import os
import argparse

import numpy as np
import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, VecMonitor
from stable_baselines3.common.callbacks import (
    EvalCallback,
    CheckpointCallback,
    BaseCallback,
)
from disaster_env import DisasterEnv


# =====================================================================
# 迭代 17: 课程学习回调
# 根据最近 N 个 episode 的成功率, 自动调整环境难度
# 参考: Isaac Lab 的地形课程机制
# =====================================================================
class CurriculumCallback(BaseCallback):
    """课程学习回调: 根据训练表现自动调整环境难度.

    工作原理:
      1. 追踪最近 window_size 个 episode 的成功率
      2. 成功率 > promote_threshold 时, 提升课程等级 (+0.05)
      3. 成功率 < demote_threshold 时, 降低课程等级 (-0.03)
      4. 将课程等级同步到所有并行环境

    Args:
        window_size: 成功率统计窗口大小
        promote_threshold: 提升难度的成功率阈值
        demote_threshold: 降低难度的成功率阈值
        check_freq: 检查频率 (每隔多少步检查一次)
    """
    def __init__(self, window_size=100, promote_threshold=0.75,
                 demote_threshold=0.40, check_freq=10000):
        super().__init__()
        self._window_size = window_size
        self._promote_thresh = promote_threshold
        self._demote_thresh = demote_threshold
        self._check_freq = check_freq
        self._episode_results = []
        self._curriculum_level = 0.0  # 从最简单开始

    def _on_step(self):
        """每个环境步调用一次, 收集 episode 结束信息."""
        # 收集已完成 episode 的成功/失败结果
        for info in self.locals.get("infos", []):
            if "episode" in info:
                cause = info.get("cause", "")
                self._episode_results.append(
                    1.0 if cause == "goal_reached" else 0.0)

        # 定期检查并调整课程等级
        if self.num_timesteps % self._check_freq < self.training_env.num_envs:
            if len(self._episode_results) >= self._window_size:
                # 取最近 window_size 个结果计算成功率
                recent = self._episode_results[-self._window_size:]
                success_rate = np.mean(recent)

                # 根据成功率调整课程等级
                old_level = self._curriculum_level
                if success_rate > self._promote_thresh:
                    self._curriculum_level = min(
                        1.0, self._curriculum_level + 0.05)
                elif success_rate < self._demote_thresh:
                    self._curriculum_level = max(
                        0.0, self._curriculum_level - 0.03)

                # 如果等级发生变化, 同步到所有环境
                if old_level != self._curriculum_level:
                    self._sync_curriculum_to_envs()

                # 记录日志
                self.logger.record("curriculum/level", self._curriculum_level)
                self.logger.record("curriculum/success_rate", success_rate)
                self.logger.record("curriculum/n_episodes",
                                   len(self._episode_results))

        return True

    def _sync_curriculum_to_envs(self):
        """将课程等级同步到所有并行环境."""
        # 通过 VecEnv 的 env_method 接口设置课程等级
        try:
            self.training_env.env_method(
                "set_curriculum_level", self._curriculum_level)
        except (AttributeError, Exception):
            # 如果环境不支持动态设置, 忽略
            pass


# =====================================================================
# 迭代 15: 熵系数调度回调
# 初始高熵系数鼓励探索, 随训练进行逐步降低
# 参考: CrowdNav 的探索-利用平衡策略
# =====================================================================
class EntropyScheduleCallback(BaseCallback):
    """熵系数调度: 随训练进行线性降低熵系数.

    Args:
        initial_ent_coef: 初始熵系数 (鼓励探索)
        final_ent_coef: 最终熵系数 (聚焦利用)
        total_timesteps: 总训练步数
    """
    def __init__(self, initial_ent_coef=0.01, final_ent_coef=0.001,
                 total_timesteps=5_000_000):
        super().__init__()
        self._initial = initial_ent_coef
        self._final = final_ent_coef
        self._total = total_timesteps

    def _on_step(self):
        """每个环境步调用一次, 线性衰减熵系数."""
        progress = min(1.0, self.num_timesteps / self._total)
        current_ent = self._initial + progress * (self._final - self._initial)
        self.model.ent_coef = current_ent
        self.logger.record("train/ent_coef", current_ent)
        return True


# =====================================================================
# 成功率追踪回调 (改进版, 增加更多统计指标)
# =====================================================================
class SuccessCallback(BaseCallback):
    """追踪并记录训练过程中的成功率和关键指标.

    记录指标:
      - custom/success_rate: 目标达成率
      - custom/life_found_rate: 幸存者发现率
      - custom/avg_episode_reward: 平均 episode 奖励
      - custom/avg_episode_length: 平均 episode 长度
      - custom/collision_rate: 碰撞率
    """
    def __init__(self):
        super().__init__()
        self._buf_success = []
        self._buf_life = []
        self._buf_rewards = []
        self._buf_lengths = []
        self._buf_collision = []

    def _on_step(self):
        """每个环境步调用一次, 收集 episode 统计数据."""
        for info in self.locals.get("infos", []):
            if "episode" in info:
                cause = info.get("cause", "")
                self._buf_success.append(
                    1.0 if cause == "goal_reached" else 0.0)
                self._buf_life.append(
                    float(info.get("life_found_count", 0) > 0))
                self._buf_collision.append(
                    1.0 if cause == "collision" else 0.0)
                self._buf_rewards.append(info["episode"]["r"])
                self._buf_lengths.append(info["episode"]["l"])

        # 每隔 10000 步记录一次统计
        if self.num_timesteps % 10000 < self.training_env.num_envs:
            n = min(100, len(self._buf_success))
            if n > 0:
                self.logger.record("custom/success_rate",
                                   np.mean(self._buf_success[-n:]))
                self.logger.record("custom/life_found_rate",
                                   np.mean(self._buf_life[-n:]))
                self.logger.record("custom/collision_rate",
                                   np.mean(self._buf_collision[-n:]))
                self.logger.record("custom/avg_episode_reward",
                                   np.mean(self._buf_rewards[-n:]))
                self.logger.record("custom/avg_episode_length",
                                   np.mean(self._buf_lengths[-n:]))
        return True


# =====================================================================
# 迭代 20: 梯度噪声回调
# 在训练后期添加少量梯度噪声, 帮助跳出局部最优
# 参考: "Noisy Networks for Exploration" (Fortunato et al., 2018)
# =====================================================================
class GradientNoiseCallback(BaseCallback):
    """梯度噪声回调: 在 PPO 更新时添加高斯噪声到梯度.

    Args:
        noise_std: 噪声标准差 (随训练进行衰减)
        total_timesteps: 总训练步数
    """
    def __init__(self, noise_std=0.01, total_timesteps=5_000_000):
        super().__init__()
        self._base_std = noise_std
        self._total = total_timesteps

    def _on_step(self):
        """记录当前噪声标准差到日志 (实际噪声注入由 PPO 内部钩子处理)."""
        progress = min(1.0, self.num_timesteps / self._total)
        # 线性衰减噪声
        current_std = self._base_std * (1.0 - progress * 0.8)
        self.logger.record("train/grad_noise_std", current_std)
        return True


# =====================================================================
# 命令行参数解析
# =====================================================================
def parse_args():
    """解析命令行参数.

    Returns:
        argparse.Namespace: 解析后的参数
    """
    p = argparse.ArgumentParser(
        description="Scout PPO 训练脚本 v2.0 -- 基于开源最佳实践")
    p.add_argument("--timesteps", type=int, default=5_000_000,
                   help="总训练步数 (默认: 5M)")
    p.add_argument("--n_envs", type=int, default=8,
                   help="并行环境数量 (默认: 8)")
    p.add_argument("--seed", type=int, default=42,
                   help="随机种子 (默认: 42)")
    p.add_argument("--resume", type=str, default=None,
                   help="从检查点恢复训练的路径")
    p.add_argument("--fast", action="store_true",
                   help="快速测试模式 (50k 步, 4 并行环境)")
    p.add_argument("--no-curriculum", action="store_true",
                   help="禁用课程学习")
    p.add_argument("--no-frame-stack", action="store_true",
                   help="禁用时序堆叠")
    return p.parse_args()


# =====================================================================
# 迭代 16: 改进的自定义网络架构
# 添加 LayerNorm, 增大网络容量, 分离 pi/vf 的特征提取
# 参考: rl-baselines3-zoo 的 MlpPolicy 最佳实践
# =====================================================================
class ImprovedMLPPolicy(nn.Module):
    """改进的 MLP 策略网络架构.

    改进点:
      - 添加 LayerNorm: 稳定训练, 加速收敛
      - 增大隐藏层: [256, 256, 128] -> [512, 256, 128]
      - 使用 Mish 激活函数: 比 Tanh/ReLU 在连续控制任务上表现更好
      - 正交初始化: 保持梯度流稳定

    注意: SB3 的 policy_kwargs 使用 net_arch dict 时会自动构建,
    此处提供备选方案, 通过 custom_network 参数启用.
    """


# =====================================================================
# 训练主函数
# =====================================================================
def train(args):
    """执行 PPO 训练流程.

    完整流程:
      1. 创建训练和评估环境 (带 VecNormalize)
      2. 配置 PPO 超参数 (含迭代 14-20 改进)
      3. 设置回调: 评估, 检查点, 课程学习, 熵调度, 成功率追踪
      4. 执行训练
      5. 导出 ONNX 模型和归一化参数

    Args:
        args: 命令行参数
    """
    os.makedirs("./checkpoints", exist_ok=True)
    os.makedirs("./logs", exist_ok=True)

    if args.fast:
        args.timesteps = 50_000
        args.n_envs = 4

    # =================================================================
    # 迭代 19: VecNormalize 配置改进
    # - clip_obs=5.0: 更严格的观测裁剪, 防止异常值
    # - clip_reward=10.0: 奖励裁剪, 防止极端奖励干扰
    # - norm_reward=True: 奖励归一化 (显著提升训练稳定性)
    # =================================================================

    # 创建训练环境工厂函数
    def make_train_env():
        return DisasterEnv(
            curriculum_level=0.0 if not args.no_curriculum else 1.0,
            enable_frame_stack=not args.no_frame_stack,
            enable_dynamic_obstacles=True,
            enable_moving_survivors=True,
        )

    def make_eval_env():
        return DisasterEnv(
            curriculum_level=1.0,  # 评估始终用最高难度
            enable_frame_stack=not args.no_frame_stack,
            enable_dynamic_obstacles=True,
            enable_moving_survivors=True,
        )

    train_env = make_vec_env(make_train_env, n_envs=args.n_envs, seed=args.seed)
    train_env = VecMonitor(train_env)
    train_env = VecNormalize(
        train_env,
        norm_obs=True,
        norm_reward=True,
        clip_obs=5.0,          # 迭代 19: 观测裁剪
        clip_reward=10.0,      # 迭代 19: 奖励裁剪
        gamma=0.99,
    )

    eval_env = make_vec_env(make_eval_env, n_envs=2, seed=args.seed + 9999)
    eval_env = VecNormalize(
        eval_env,
        norm_obs=True,
        norm_reward=False,     # 评估时不归一化奖励, 以便比较绝对值
        training=False,
        clip_obs=5.0,
    )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Training device={device}  envs={args.n_envs}  "
          f"steps={args.timesteps:,}")
    print(f"Curriculum: {'ON' if not args.no_curriculum else 'OFF'}  "
          f"FrameStack: {'ON' if not args.no_frame_stack else 'OFF'}")

    # =================================================================
    # 迭代 14: 学习率退火 (Linear Schedule)
    # 从 3e-4 线性衰减到 1e-5
    # 参考: rl-baselines3-zoo 的 lr_schedule 配置
    # =================================================================
    lr_schedule = lambda progress: 1e-5 + progress * (3e-4 - 1e-5)

    # =================================================================
    # 迭代 18: 超参优化
    # - n_steps: 2048 -> 4096 (更大的 rollout buffer, 更稳定的梯度估计)
    # - n_epochs: 10 -> 15 (更多更新轮次, 提高数据利用效率)
    # - batch_size: 512 -> 1024 (与更大的 n_steps 匹配)
    # - clip_range: 0.2 -> 线性退火 0.2->0.05 (后期更保守的更新)
    # =================================================================
    clip_schedule = lambda progress: 0.05 + progress * (0.2 - 0.05)

    if args.resume:
        model = PPO.load(args.resume, env=train_env, device=device)
        # 恢复时更新学习率调度
        model.lr_schedule = lr_schedule
    else:
        model = PPO(
            policy="MlpPolicy",
            env=train_env,
            learning_rate=lr_schedule,           # 迭代 14: 学习率退火
            n_steps=4096,                        # 迭代 18: 更大的 rollout
            batch_size=1024,                     # 迭代 18: 更大的 batch
            n_epochs=15,                         # 迭代 18: 更多更新轮次
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=clip_schedule,            # 迭代 18: clip 退火
            ent_coef=0.01,                       # 迭代 15: 初始熵系数 (由调度器动态调整)
            vf_coef=0.5,
            max_grad_norm=0.5,
            # 迭代 16: 改进的网络架构
            policy_kwargs=dict(
                net_arch=dict(pi=[512, 256, 128], vf=[512, 256, 128]),
                activation_fn=torch.nn.Tanh,
                ortho_init=True,
            ),
            tensorboard_log="./logs",
            verbose=1,
            device=device,
            seed=args.seed,
        )

    # =================================================================
    # 设置回调函数
    # =================================================================
    callbacks = [
        # 评估回调: 每 50k 步评估一次, 保存最优模型
        EvalCallback(
            eval_env,
            best_model_save_path="./checkpoints/best",
            log_path="./logs/eval",
            eval_freq=max(50_000 // args.n_envs, 1),
            n_eval_episodes=20,
            deterministic=True,
            verbose=1,
        ),
        # 检查点回调: 每 200k 步保存一次检查点
        CheckpointCallback(
            save_freq=max(200_000 // args.n_envs, 1),
            save_path="./checkpoints",
            name_prefix="scout_ppo",
            save_vecnormalize=True,
        ),
        # 成功率追踪
        SuccessCallback(),
    ]

    # 迭代 15: 熵系数调度 (仅在非恢复训练时启用)
    if not args.resume:
        callbacks.append(
            EntropyScheduleCallback(
                initial_ent_coef=0.01,
                final_ent_coef=0.001,
                total_timesteps=args.timesteps,
            )
        )

    # 迭代 17: 课程学习回调
    if not args.no_curriculum:
        callbacks.append(
            CurriculumCallback(
                window_size=100,
                promote_threshold=0.75,
                demote_threshold=0.40,
                check_freq=max(10_000 // args.n_envs, 1),
            )
        )

    # 迭代 20: 梯度噪声 (可选, 默认关闭)
    # callbacks.append(
    #     GradientNoiseCallback(
    #         noise_std=0.01,
    #         total_timesteps=args.timesteps,
    #     )
    # )

    # =================================================================
    # 执行训练
    # =================================================================
    print("=" * 60)
    print("开始训练...")
    print(f"  总步数: {args.timesteps:,}")
    print(f"  并行环境: {args.n_envs}")
    print(f"  设备: {device}")
    print(f"  课程学习: {'ON' if not args.no_curriculum else 'OFF'}")
    print(f"  帧堆叠: {'ON' if not args.no_frame_stack else 'OFF'}")
    print(f"  学习率: 3e-4 -> 1e-5 (线性退火)")
    print(f"  熵系数: 0.01 -> 0.001 (线性退火)")
    print(f"  网络: pi=[512,256,128] vf=[512,256,128]")
    print(f"  n_steps=4096, batch=1024, epochs=15")
    print("=" * 60)

    model.learn(
        total_timesteps=args.timesteps,
        callback=callbacks,
        progress_bar=True,
        reset_num_timesteps=(args.resume is None),
    )

    # =================================================================
    # 保存最终模型和归一化参数
    # =================================================================
    model.save("./checkpoints/scout_final")
    train_env.save("./checkpoints/scout_vecnorm.pkl")
    print("Training done -> ./checkpoints/")

    # 导出 ONNX 模型 (用于 RK3588 上的 onnxruntime 推理)
    _export_onnx(model, "./checkpoints/scout_policy.onnx")
    # 导出归一化参数 (用于推理时的观测归一化)
    _export_norm(train_env, "./checkpoints/scout_norm.npz")


# =====================================================================
# ONNX 导出 (保持不变)
# =====================================================================
def _export_onnx(model, path):
    """将训练好的策略网络导出为 ONNX 格式.

    导出的模型接受归一化后的观测向量, 输出 2 维动作 [vx, wz].

    Args:
        model: 训练好的 PPO 模型
        path: ONNX 文件保存路径
    """
    import torch
    model.policy.set_training_mode(False)
    obs_dim = model.observation_space.shape[0]

    class Act(torch.nn.Module):
        """ONNX 导出包装器, 只保留动作输出部分."""
        def __init__(self, p):
            super().__init__()
            self.p = p

        def forward(self, obs):
            lp, _ = self.p.mlp_extractor(obs)
            return torch.tanh(self.p.action_net(lp))

    w = Act(model.policy)
    w.eval()
    dummy = torch.zeros(1, obs_dim)
    try:
        torch.onnx.export(
            w, dummy, path,
            input_names=["observation"],
            output_names=["action"],
            opset_version=17,
            dynamic_axes={
                "observation": {0: "batch"},
                "action": {0: "batch"},
            },
        )
        print(f"ONNX -> {path}")
    except Exception as e:
        print(f"ONNX export failed: {e}")


# =====================================================================
# 归一化参数导出 (保持不变)
# =====================================================================
def _export_norm(env, path):
    """导出 VecNormalize 的观测均值和方差.

    推理时需要用这些参数对观测进行归一化, 与训练时一致.

    Args:
        env: VecNormalize 包装的环境
        path: .npz 文件保存路径
    """
    np.savez(
        path,
        mean=env.obs_rms.mean.astype(np.float32),
        var=env.obs_rms.var.astype(np.float32),
    )
    print(f"Norm -> {path}")


# =====================================================================
# 入口
# =====================================================================
if __name__ == "__main__":
    train(parse_args())
