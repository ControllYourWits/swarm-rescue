"""
eval_scout.py -- Scout PPO 策略评估脚本

加载训练好的模型, 运行 N 个 episode 统计:
  - 成功率 (goal_reached)
  - 碰撞率 (collision)
  - 幸存者发现率
  - 平均 episode 奖励
  - 平均 episode 长度
  - 平均完成时间

用法:
  python eval_scout.py --model checkpoints/best/best_model.zip --episodes 50
  python eval_scout.py --model checkpoints/scout_final.zip --episodes 100 --curriculum 1.0
  python eval_scout.py --onnx checkpoints/scout_policy.onnx --episodes 50
"""
import argparse
import csv
import os
import time

import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description="Scout PPO evaluation")
    p.add_argument("--model", type=str, default=None,
                   help="Path to SB3 .zip model")
    p.add_argument("--onnx", type=str, default=None,
                   help="Path to ONNX model (uses onnxruntime)")
    p.add_argument("--norm", type=str, default="checkpoints/scout_norm.npz",
                   help="Path to normalization .npz file")
    p.add_argument("--episodes", type=int, default=50,
                   help="Number of evaluation episodes")
    p.add_argument("--curriculum", type=float, default=1.0,
                   help="Curriculum level for evaluation (0.0-1.0)")
    p.add_argument("--seed", type=int, default=12345,
                   help="Random seed")
    p.add_argument("--output", type=str, default="eval_results.csv",
                   help="Output CSV file")
    p.add_argument("--render", action="store_true",
                   help="Render episodes")
    return p.parse_args()


def eval_sb3_model(model_path, episodes, curriculum, seed, render):
    """Evaluate using Stable-Baselines3 model."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import VecNormalize
    from disaster_env import DisasterEnv

    print(f"Loading SB3 model: {model_path}")
    model = PPO.load(model_path)

    render_mode = "human" if render else None
    env = DisasterEnv(
        curriculum_level=curriculum,
        enable_frame_stack=False,
        render_mode=render_mode,
    )

    results = []
    for ep in range(episodes):
        obs, info = env.reset(seed=seed + ep)
        total_reward = 0.0
        steps = 0
        cause = "timeout"
        life_found = 0

        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            steps += 1
            life_found = info.get("life_found_count", 0)

            if terminated or truncated:
                cause = info.get("cause", "timeout")
                break

        results.append({
            "episode": ep,
            "cause": cause,
            "reward": total_reward,
            "steps": steps,
            "life_found": life_found,
            "curriculum": curriculum,
        })

        if render:
            env.close()

    return results


def eval_onnx_model(onnx_path, norm_path, episodes, curriculum, seed, render):
    """Evaluate using ONNX model with manual inference loop."""
    import onnxruntime as ort
    from disaster_env import DisasterEnv

    print(f"Loading ONNX model: {onnx_path}")
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    inp_name = sess.get_inputs()[0].name
    out_name = sess.get_outputs()[0].name

    # Load normalization params
    if os.path.exists(norm_path):
        data = np.load(norm_path)
        mean = data["mean"].astype(np.float32)
        var = data["var"].astype(np.float32)
        print(f"Loaded normalization: mean.shape={mean.shape}, var.shape={var.shape}")
    else:
        print(f"Warning: norm file {norm_path} not found, using identity normalization")
        obs_dim = sess.get_inputs()[0].shape[1] if len(sess.get_inputs()[0].shape) > 1 else 43
        mean = np.zeros(obs_dim, np.float32)
        var = np.ones(obs_dim, np.float32)

    render_mode = "human" if render else None
    env = DisasterEnv(
        curriculum_level=curriculum,
        enable_frame_stack=False,
        render_mode=render_mode,
    )

    results = []
    for ep in range(episodes):
        obs, info = env.reset(seed=seed + ep)
        total_reward = 0.0
        steps = 0
        cause = "timeout"
        life_found = 0

        while True:
            # Normalize observation
            obs_norm = np.clip(
                (obs - mean) / np.sqrt(var + 1e-8), -10.0, 10.0
            ).astype(np.float32)
            action = sess.run([out_name], {inp_name: obs_norm.reshape(1, -1)})[0][0]
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            steps += 1
            life_found = info.get("life_found_count", 0)

            if terminated or truncated:
                cause = info.get("cause", "timeout")
                break

        results.append({
            "episode": ep,
            "cause": cause,
            "reward": total_reward,
            "steps": steps,
            "life_found": life_found,
            "curriculum": curriculum,
        })

        if render:
            env.close()

    return results


def print_summary(results):
    """Print evaluation summary."""
    n = len(results)
    if n == 0:
        print("No results")
        return

    causes = [r["cause"] for r in results]
    rewards = [r["reward"] for r in results]
    steps = [r["steps"] for r in results]
    lifes = [r["life_found"] for r in results]

    n_goal = causes.count("goal_reached")
    n_collision = causes.count("collision")
    n_boundary = causes.count("boundary")
    n_timeout = causes.count("timeout")

    print("\n" + "=" * 50)
    print(f"  Evaluation Results ({n} episodes)")
    print("=" * 50)
    print(f"  Success rate:    {n_goal/n*100:.1f}% ({n_goal}/{n})")
    print(f"  Collision rate:  {n_collision/n*100:.1f}% ({n_collision}/{n})")
    print(f"  Boundary rate:   {n_boundary/n*100:.1f}% ({n_boundary}/{n})")
    print(f"  Timeout rate:    {n_timeout/n*100:.1f}% ({n_timeout}/{n})")
    print(f"  Life found rate: {sum(1 for l in lifes if l > 0)/n*100:.1f}%")
    print(f"  Avg reward:      {np.mean(rewards):.1f} ± {np.std(rewards):.1f}")
    print(f"  Avg steps:       {np.mean(steps):.0f} ± {np.std(steps):.0f}")

    success_rewards = [r["reward"] for r in results if r["cause"] == "goal_reached"]
    if success_rewards:
        success_steps = [r["steps"] for r in results if r["cause"] == "goal_reached"]
        print(f"  Success avg reward: {np.mean(success_rewards):.1f}")
        print(f"  Success avg steps:  {np.mean(success_steps):.0f}")
    print("=" * 50)


def save_csv(results, path):
    """Save results to CSV."""
    if not results:
        return
    keys = results[0].keys()
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(results)
    print(f"Results saved to {path}")


def main():
    args = parse_args()

    if args.model:
        results = eval_sb3_model(args.model, args.episodes,
                                  args.curriculum, args.seed, args.render)
    elif args.onnx:
        results = eval_onnx_model(args.onnx, args.norm, args.episodes,
                                   args.curriculum, args.seed, args.render)
    else:
        print("Error: specify --model or --onnx")
        return

    print_summary(results)
    save_csv(results, args.output)


if __name__ == "__main__":
    main()
