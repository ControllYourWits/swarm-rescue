"""
verify_onnx.py -- ONNX 模型导出精度验证

对比 PyTorch 原始模型和 ONNX 导出模型的输出,
确保导出精度误差在可接受范围内 (< 1e-4).

用法:
  python verify_onnx.py --model checkpoints/scout_final.zip --onnx checkpoints/scout_policy.onnx
  python verify_onnx.py --onnx checkpoints/scout_policy.onnx --norm checkpoints/scout_norm.npz --samples 200
"""
import argparse
import os

import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description="Verify ONNX export accuracy")
    p.add_argument("--model", type=str, default=None,
                   help="Path to SB3 .zip model (for PyTorch reference)")
    p.add_argument("--onnx", type=str, required=True,
                   help="Path to ONNX model")
    p.add_argument("--norm", type=str, default="checkpoints/scout_norm.npz",
                   help="Path to normalization .npz file")
    p.add_argument("--samples", type=int, default=200,
                   help="Number of random test samples")
    p.add_argument("--tolerance", type=float, default=1e-4,
                   help="Max allowed absolute error")
    p.add_argument("--seed", type=int, default=42,
                   help="Random seed")
    return p.parse_args()


def verify_with_sb3(model_path, onnx_path, norm_path, n_samples, tolerance, seed):
    """Compare SB3 PyTorch model output with ONNX output."""
    import torch
    from stable_baselines3 import PPO
    import onnxruntime as ort

    print(f"PyTorch model: {model_path}")
    print(f"ONNX model:    {onnx_path}")

    # Load SB3 model
    model = PPO.load(model_path)
    model.policy.set_training_mode(False)

    # Load ONNX model
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    inp_name = sess.get_inputs()[0].name
    out_name = sess.get_outputs()[0].name

    obs_dim = model.observation_space.shape[0]
    print(f"Observation dim: {obs_dim}")

    # Load normalization
    if os.path.exists(norm_path):
        data = np.load(norm_path)
        mean = data["mean"].astype(np.float32)
        var = data["var"].astype(np.float32)
    else:
        mean = np.zeros(obs_dim, np.float32)
        var = np.ones(obs_dim, np.float32)

    rng = np.random.default_rng(seed)
    max_err = 0.0
    n_fail = 0

    for i in range(n_samples):
        # Random observation
        obs = rng.uniform(-1.0, 1.0, (1, obs_dim)).astype(np.float32)

        # Normalize
        obs_norm = np.clip(
            (obs - mean) / np.sqrt(var + 1e-8), -10.0, 10.0
        ).astype(np.float32)

        # PyTorch inference
        with torch.no_grad():
            obs_t = torch.from_numpy(obs_norm)
            pytorch_out = model.policy.actor(obs_t).numpy()

        # ONNX inference
        onnx_out = sess.run([out_name], {inp_name: obs_norm})[0]

        # Compare
        err = np.max(np.abs(pytorch_out - onnx_out))
        max_err = max(max_err, err)
        if err > tolerance:
            n_fail += 1
            if n_fail <= 5:
                print(f"  Sample {i}: err={err:.6f} > {tolerance}")
                print(f"    PyTorch: {pytorch_out}")
                print(f"    ONNX:    {onnx_out}")

    print(f"\nVerification: {n_samples} samples, max_err={max_err:.2e}, "
          f"tolerance={tolerance:.2e}")
    if n_fail == 0:
        print("✓ PASSED — ONNX output matches PyTorch within tolerance")
        return True
    else:
        print(f"✗ FAILED — {n_fail}/{n_samples} samples exceeded tolerance")
        return False


def verify_standalone(onnx_path, norm_path, n_samples, seed):
    """Standalone ONNX verification (no SB3 model needed).

    Checks that the model:
    1. Loads correctly
    2. Accepts the expected input shape
    3. Produces finite output in [-1, 1] range
    4. Deterministic (same input → same output)
    """
    import onnxruntime as ort

    print(f"ONNX model: {onnx_path}")

    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    out = sess.get_outputs()[0]

    print(f"Input:  name={inp.name}, shape={inp.shape}, dtype={inp.type}")
    print(f"Output: name={out.name}, shape={out.shape}, dtype={out.type}")

    # Determine obs_dim from input shape
    inp_shape = inp.shape
    if isinstance(inp_shape, list) and len(inp_shape) >= 2:
        obs_dim = inp_shape[1]
        if isinstance(obs_dim, str):
            # Dynamic dimension, try norm file
            if os.path.exists(norm_path):
                data = np.load(norm_path)
                obs_dim = data["mean"].shape[0]
            else:
                obs_dim = 43
    else:
        obs_dim = 43

    print(f"Using obs_dim={obs_dim}")

    # Load normalization
    if os.path.exists(norm_path):
        data = np.load(norm_path)
        mean = data["mean"].astype(np.float32)
        var = data["var"].astype(np.float32)
        print(f"Normalization loaded: mean.shape={mean.shape}")
    else:
        print(f"Warning: {norm_path} not found, using identity normalization")
        mean = np.zeros(obs_dim, np.float32)
        var = np.ones(obs_dim, np.float32)

    rng = np.random.default_rng(seed)
    all_ok = True

    # Test 1: Basic inference
    print("\n--- Test 1: Basic inference ---")
    obs = rng.uniform(-1.0, 1.0, (1, obs_dim)).astype(np.float32)
    obs_norm = np.clip((obs - mean) / np.sqrt(var + 1e-8), -10.0, 10.0)
    result = sess.run([out.name], {inp.name: obs_norm})[0]
    print(f"  Output shape: {result.shape}")
    print(f"  Output range: [{result.min():.4f}, {result.max():.4f}]")
    print(f"  Finite: {np.all(np.isfinite(result))}")
    if not np.all(np.isfinite(result)):
        print("  ✗ FAIL: Non-finite output")
        all_ok = False
    else:
        print("  ✓ PASS")

    # Test 2: Output in valid range
    print("\n--- Test 2: Output range [-1, 1] ---")
    in_range = np.all(result >= -1.01) and np.all(result <= 1.01)
    if in_range:
        print("  ✓ PASS")
    else:
        print(f"  ✗ FAIL: Output out of range: {result}")
        all_ok = False

    # Test 3: Deterministic
    print("\n--- Test 3: Deterministic output ---")
    result2 = sess.run([out.name], {inp.name: obs_norm})[0]
    det = np.allclose(result, result2, atol=1e-7)
    if det:
        print("  ✓ PASS")
    else:
        print(f"  ✗ FAIL: max diff = {np.max(np.abs(result - result2)):.2e}")
        all_ok = False

    # Test 4: Multiple random inputs
    print(f"\n--- Test 4: {n_samples} random inputs ---")
    n_nan = 0
    n_oob = 0
    for _ in range(n_samples):
        obs = rng.uniform(-2.0, 2.0, (1, obs_dim)).astype(np.float32)
        obs_norm = np.clip((obs - mean) / np.sqrt(var + 1e-8), -10.0, 10.0)
        r = sess.run([out.name], {inp.name: obs_norm})[0]
        if not np.all(np.isfinite(r)):
            n_nan += 1
        if np.any(r < -1.1) or np.any(r > 1.1):
            n_oob += 1

    if n_nan == 0 and n_oob == 0:
        print(f"  ✓ PASS — all {n_samples} outputs finite and in range")
    else:
        if n_nan: print(f"  ✗ {n_nan} outputs had non-finite values")
        if n_oob: print(f"  ✗ {n_oob} outputs out of [-1.1, 1.1] range")
        all_ok = False

    print(f"\n{'='*40}")
    if all_ok:
        print("All ONNX verification tests PASSED ✓")
    else:
        print("Some ONNX verification tests FAILED ✗")
    return all_ok


def main():
    args = parse_args()

    if args.model:
        ok = verify_with_sb3(args.model, args.onnx, args.norm,
                              args.samples, args.tolerance, args.seed)
    else:
        ok = verify_standalone(args.onnx, args.norm, args.samples, args.seed)

    exit(0 if ok else 1)


if __name__ == "__main__":
    main()
