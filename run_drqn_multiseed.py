import argparse
import csv
import json
import os
import random
import shutil
from types import SimpleNamespace

import numpy as np
import torch

from drqn_minimal import GridPOMDPEnv, TorchDRQN, train


def _train_one_seed(base_args, seed, root_dir):
    out_dir = os.path.join(root_dir, f"seed_{seed}")
    os.makedirs(out_dir, exist_ok=True)
    args = SimpleNamespace(**vars(base_args))
    args.seed = seed
    args.output_dir = out_dir
    print(f"[multiseed] training seed={seed} output={out_dir}", flush=True)
    train(args)
    return {
        "seed": seed,
        "output_dir": out_dir,
        "history_csv": os.path.join(out_dir, "drqn_torch_history.csv"),
        "best_ckpt": os.path.join(out_dir, "drqn_torch_best.pt"),
    }


def _evaluate_checkpoint(ckpt_path, eval_episodes, max_steps, eval_seed, device):
    random.seed(eval_seed)
    np.random.seed(eval_seed)
    torch.manual_seed(eval_seed)

    payload = torch.load(ckpt_path, map_location=device)
    obs_dim = int(payload["obs_dim"])
    hidden_dim = int(payload["hidden_dim"])
    num_actions = int(payload.get("num_actions", 5))
    max_neighbors = int(payload.get("max_neighbors", max(1, num_actions - 1)))
    snow_dynamic = bool(payload.get("snow_dynamic", True))
    snow_start_zero = bool(payload.get("snow_start_zero", True))
    block_init_prob = float(payload.get("block_init_prob", 0.0))
    block_from_snow_threshold = float(payload.get("block_from_snow_threshold", 0.72))
    block_from_snow_prob = float(payload.get("block_from_snow_prob", 0.003))
    model = TorchDRQN(obs_dim=obs_dim, hidden_dim=hidden_dim, num_actions=num_actions).to(device)
    model.load_state_dict(payload["model_state_dict"])
    model.eval()

    env = GridPOMDPEnv(
        max_steps=max_steps,
        max_neighbors=max_neighbors,
        snow_dynamic=snow_dynamic,
        snow_start_zero=snow_start_zero,
        block_init_prob=block_init_prob,
        block_from_snow_threshold=block_from_snow_threshold,
        block_from_snow_prob=block_from_snow_prob,
    )
    returns = []
    reached = []
    steps = []

    with torch.no_grad():
        for _ in range(eval_episodes):
            obs_np = env.reset()
            h = model.init_hidden(device=device)
            done = False
            ep_ret = 0.0
            ep_steps = 0
            ep_reached = 0
            while not done:
                ep_steps += 1
                obs_t = torch.tensor(obs_np, dtype=torch.float32, device=device).unsqueeze(0)
                q_t, h = model(obs_t, h)
                mask = env.get_action_mask()
                q_np = q_t.detach().cpu().numpy().squeeze(0)
                q_np[mask <= 0.5] = -1e9
                action = int(np.argmax(q_np))
                obs_np, reward, done, _ = env.step(action)
                ep_ret += reward
                if done and env.node == env.goal_node:
                    ep_reached = 1
            returns.append(ep_ret)
            reached.append(ep_reached)
            steps.append(ep_steps)

    return {
        "eval_return_mean": float(np.mean(returns)),
        "eval_return_std": float(np.std(returns)),
        "eval_reached_rate": float(np.mean(reached)),
        "eval_steps_mean": float(np.mean(steps)),
    }


def _evaluate_checkpoint_detailed(ckpt_path, eval_episodes, max_steps, eval_seed, device):
    random.seed(eval_seed)
    np.random.seed(eval_seed)
    torch.manual_seed(eval_seed)

    payload = torch.load(ckpt_path, map_location=device)
    obs_dim = int(payload["obs_dim"])
    hidden_dim = int(payload["hidden_dim"])
    num_actions = int(payload.get("num_actions", 5))
    max_neighbors = int(payload.get("max_neighbors", max(1, num_actions - 1)))
    snow_dynamic = bool(payload.get("snow_dynamic", True))
    snow_start_zero = bool(payload.get("snow_start_zero", True))
    block_init_prob = float(payload.get("block_init_prob", 0.0))
    block_from_snow_threshold = float(payload.get("block_from_snow_threshold", 0.72))
    block_from_snow_prob = float(payload.get("block_from_snow_prob", 0.003))
    model = TorchDRQN(obs_dim=obs_dim, hidden_dim=hidden_dim, num_actions=num_actions).to(device)
    model.load_state_dict(payload["model_state_dict"])
    model.eval()

    env = GridPOMDPEnv(
        max_steps=max_steps,
        max_neighbors=max_neighbors,
        snow_dynamic=snow_dynamic,
        snow_start_zero=snow_start_zero,
        block_init_prob=block_init_prob,
        block_from_snow_threshold=block_from_snow_threshold,
        block_from_snow_prob=block_from_snow_prob,
    )
    rows = []
    returns = []
    reached = []
    steps = []
    with torch.no_grad():
        for ep in range(1, eval_episodes + 1):
            obs_np = env.reset()
            h = model.init_hidden(device=device)
            done = False
            ep_ret = 0.0
            ep_steps = 0
            ep_reached = 0
            while not done:
                ep_steps += 1
                obs_t = torch.tensor(obs_np, dtype=torch.float32, device=device).unsqueeze(0)
                q_t, h = model(obs_t, h)
                mask = env.get_action_mask()
                q_np = q_t.detach().cpu().numpy().squeeze(0)
                q_np[mask <= 0.5] = -1e9
                action = int(np.argmax(q_np))
                obs_np, reward, done, _ = env.step(action)
                ep_ret += reward
                if done and env.node == env.goal_node:
                    ep_reached = 1
            rows.append(
                {
                    "episode": ep,
                    "return": float(ep_ret),
                    "reached": ep_reached,
                    "steps": ep_steps,
                }
            )
            returns.append(ep_ret)
            reached.append(ep_reached)
            steps.append(ep_steps)

    summary = {
        "episodes": int(eval_episodes),
        "seed": int(eval_seed),
        "checkpoint": ckpt_path,
        "return_mean": float(np.mean(returns)),
        "return_std": float(np.std(returns)),
        "reached_rate": float(np.mean(reached)),
        "steps_mean": float(np.mean(steps)),
        "steps_std": float(np.std(steps)),
    }
    return rows, summary


def _load_history(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(
                {
                    "episode": int(r["episode"]),
                    "return": float(r["return"]),
                    "moving_avg_return": float(r["moving_avg_return"]),
                }
            )
    return rows


def _plot_histories(seed_infos, out_png, show_plot=False):
    import matplotlib
    if not show_plot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.figure(figsize=(10, 6))
    for info in seed_infos:
        rows = _load_history(info["history_csv"])
        x = [r["episode"] for r in rows]
        y = [r["moving_avg_return"] for r in rows]
        plt.plot(x, y, linewidth=1.6, alpha=0.8, label=f"seed {info['seed']}")

    min_len = min(len(_load_history(info["history_csv"])) for info in seed_infos)
    xs = np.array([_load_history(seed_infos[0]["history_csv"])[i]["episode"] for i in range(min_len)])
    ys = []
    for info in seed_infos:
        rows = _load_history(info["history_csv"])[:min_len]
        ys.append([r["moving_avg_return"] for r in rows])
    ys = np.array(ys)
    y_mean = np.mean(ys, axis=0)
    y_std = np.std(ys, axis=0)
    plt.plot(xs, y_mean, color="black", linewidth=2.5, label="mean")
    plt.fill_between(xs, y_mean - y_std, y_mean + y_std, color="gray", alpha=0.2, label="mean ± std")

    plt.title("DRQN Training Curves (Moving Avg Return)")
    plt.xlabel("Episode")
    plt.ylabel("Moving Avg Return")
    plt.legend()
    plt.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    if show_plot:
        plt.show(block=True)
    else:
        plt.close()


def run(args):
    os.makedirs(args.output_root, exist_ok=True)
    seeds = [int(s.strip()) for s in args.seeds.split(",") if s.strip()]

    train_args = SimpleNamespace(
        episodes=args.episodes,
        max_steps=args.max_steps,
        max_neighbors=args.max_neighbors,
        goal_reward_base=args.goal_reward_base,
        goal_reward_time_bonus=args.goal_reward_time_bonus,
        w_reached=args.w_reached,
        w_alive=args.w_alive,
        w_exposure=args.w_exposure,
        w_time=args.w_time,
        w_progress=args.w_progress,
        snow_dynamic=args.snow_dynamic,
        snow_start_zero=args.snow_start_zero,
        block_init_prob=args.block_init_prob,
        block_from_snow_threshold=args.block_from_snow_threshold,
        block_from_snow_prob=args.block_from_snow_prob,
        hidden_dim=args.hidden_dim,
        lr=args.lr,
        gamma=args.gamma,
        eps_start=args.eps_start,
        eps_end=args.eps_end,
        eps_decay=args.eps_decay,
        target_update=args.target_update,
        grad_clip=args.grad_clip,
        replay_episodes=args.replay_episodes,
        batch_size=args.batch_size,
        warmup_steps=args.warmup_steps,
        updates_per_step=args.updates_per_step,
        seq_len=args.seq_len,
        burn_in=args.burn_in,
        best_window=args.best_window,
        curriculum_start_dist=args.curriculum_start_dist,
        curriculum_end_dist=args.curriculum_end_dist,
        curriculum_full_at_episode=args.curriculum_full_at_episode,
        curriculum_freeze_episode=args.curriculum_freeze_episode,
        log_every=args.log_every,
        seed=42,
        output_dir=args.output_root,
        device=args.device,
        init_checkpoint=None,
    )

    seed_infos = []
    for seed in seeds:
        info = _train_one_seed(train_args, seed, args.output_root)
        seed_infos.append(info)

    eval_rows = []
    for info in seed_infos:
        eval_seed = args.eval_seed_base + info["seed"]
        stats = _evaluate_checkpoint(
            ckpt_path=info["best_ckpt"],
            eval_episodes=args.eval_episodes,
            max_steps=args.max_steps,
            eval_seed=eval_seed,
            device=torch.device(args.device if args.device != "auto" else "cpu"),
        )
        row = {"seed": info["seed"], **stats}
        eval_rows.append(row)
        print(f"[multiseed] eval seed={info['seed']} -> {stats}", flush=True)

    summary_csv = os.path.join(args.output_root, "multiseed_eval_summary.csv")
    with open(summary_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(eval_rows[0].keys()))
        w.writeheader()
        w.writerows(eval_rows)

    agg = {
        "return_mean_mean": float(np.mean([r["eval_return_mean"] for r in eval_rows])),
        "return_mean_std": float(np.std([r["eval_return_mean"] for r in eval_rows])),
        "reached_rate_mean": float(np.mean([r["eval_reached_rate"] for r in eval_rows])),
        "reached_rate_std": float(np.std([r["eval_reached_rate"] for r in eval_rows])),
        "steps_mean_mean": float(np.mean([r["eval_steps_mean"] for r in eval_rows])),
        "steps_mean_std": float(np.std([r["eval_steps_mean"] for r in eval_rows])),
    }

    agg_txt = os.path.join(args.output_root, "multiseed_eval_aggregate.txt")
    with open(agg_txt, "w", encoding="utf-8") as f:
        f.write("Multi-seed DRQN Evaluation Aggregate\n")
        for k, v in agg.items():
            f.write(f"{k}: {v:.6f}\n")

    best_row = max(eval_rows, key=lambda r: (r["eval_return_mean"], r["eval_reached_rate"]))
    best_seed = int(best_row["seed"])
    best_seed_info = next(info for info in seed_infos if info["seed"] == best_seed)
    best_cfg = {
        "selection_rule": "max(eval_return_mean), tie-break by eval_reached_rate",
        "best_seed": best_seed,
        "best_checkpoint": best_seed_info["best_ckpt"],
        "best_eval_metrics": best_row,
        "aggregate_metrics": agg,
        "training_hyperparameters": {
            "episodes": args.episodes,
            "max_steps": args.max_steps,
            "hidden_dim": args.hidden_dim,
            "max_neighbors": args.max_neighbors,
            "goal_reward_base": args.goal_reward_base,
            "goal_reward_time_bonus": args.goal_reward_time_bonus,
            "w_reached": args.w_reached,
            "w_alive": args.w_alive,
            "w_exposure": args.w_exposure,
            "w_time": args.w_time,
            "w_progress": args.w_progress,
            "snow_dynamic": args.snow_dynamic,
            "snow_start_zero": args.snow_start_zero,
            "block_init_prob": args.block_init_prob,
            "block_from_snow_threshold": args.block_from_snow_threshold,
            "block_from_snow_prob": args.block_from_snow_prob,
            "lr": args.lr,
            "gamma": args.gamma,
            "eps_start": args.eps_start,
            "eps_end": args.eps_end,
            "eps_decay": args.eps_decay,
            "target_update": args.target_update,
            "grad_clip": args.grad_clip,
            "replay_episodes": args.replay_episodes,
            "batch_size": args.batch_size,
            "warmup_steps": args.warmup_steps,
            "updates_per_step": args.updates_per_step,
            "seq_len": args.seq_len,
            "burn_in": args.burn_in,
            "best_window": args.best_window,
            "curriculum_start_dist": args.curriculum_start_dist,
            "curriculum_end_dist": args.curriculum_end_dist,
            "curriculum_full_at_episode": args.curriculum_full_at_episode,
            "curriculum_freeze_episode": args.curriculum_freeze_episode,
            "device": args.device,
        },
        "evaluation_setup": {
            "eval_episodes": args.eval_episodes,
            "eval_seed_base": args.eval_seed_base,
        },
        "all_seed_outputs": seed_infos,
    }
    best_cfg_path = os.path.join(args.output_root, "best_run_config.json")
    with open(best_cfg_path, "w", encoding="utf-8") as f:
        json.dump(best_cfg, f, indent=2)

    best_model_path = os.path.join(args.output_root, "best_model.pt")
    shutil.copy2(best_seed_info["best_ckpt"], best_model_path)

    final_eval_dir = os.path.join(args.output_root, "best_model_eval")
    os.makedirs(final_eval_dir, exist_ok=True)
    final_eval_seed = args.final_eval_seed
    rows, summary = _evaluate_checkpoint_detailed(
        ckpt_path=best_model_path,
        eval_episodes=args.final_eval_episodes,
        max_steps=args.max_steps,
        eval_seed=final_eval_seed,
        device=torch.device(args.device if args.device != "auto" else "cpu"),
    )
    final_details_csv = os.path.join(final_eval_dir, "best_model_eval_details.csv")
    with open(final_details_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["episode", "return", "reached", "steps"])
        w.writeheader()
        w.writerows(rows)
    final_summary_json = os.path.join(final_eval_dir, "best_model_eval_summary.json")
    with open(final_summary_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    final_summary_txt = os.path.join(final_eval_dir, "best_model_eval_summary.txt")
    with open(final_summary_txt, "w", encoding="utf-8") as f:
        f.write("Best Model Evaluation KPI\n")
        for k, v in summary.items():
            f.write(f"{k}: {v}\n")

    rerun_cmd = (
        "venv/bin/python drqn_minimal.py "
        f"--episodes {args.episodes} --max-steps {args.max_steps} "
        f"--max-neighbors {args.max_neighbors} "
        f"--goal-reward-base {args.goal_reward_base} "
        f"--goal-reward-time-bonus {args.goal_reward_time_bonus} "
        f"--w-reached {args.w_reached} --w-alive {args.w_alive} "
        f"--w-exposure {args.w_exposure} --w-time {args.w_time} --w-progress {args.w_progress} "
        f"--snow-dynamic {args.snow_dynamic} --snow-start-zero {args.snow_start_zero} "
        f"--block-init-prob {args.block_init_prob} "
        f"--block-from-snow-threshold {args.block_from_snow_threshold} "
        f"--block-from-snow-prob {args.block_from_snow_prob} "
        f"--hidden-dim {args.hidden_dim} --lr {args.lr} --gamma {args.gamma} "
        f"--eps-start {args.eps_start} --eps-end {args.eps_end} --eps-decay {args.eps_decay} "
        f"--target-update {args.target_update} --grad-clip {args.grad_clip} "
        f"--replay-episodes {args.replay_episodes} --batch-size {args.batch_size} "
        f"--warmup-steps {args.warmup_steps} --updates-per-step {args.updates_per_step} "
        f"--seq-len {args.seq_len} --burn-in {args.burn_in} --best-window {args.best_window} "
        f"--curriculum-start-dist {args.curriculum_start_dist} "
        f"--curriculum-end-dist {args.curriculum_end_dist} "
        f"--curriculum-full-at-episode {args.curriculum_full_at_episode} "
        f"--curriculum-freeze-episode {args.curriculum_freeze_episode} "
        f"--seed {best_seed} --output-dir {best_seed_info['output_dir']} --device {args.device}"
    )
    rerun_cmd_path = os.path.join(args.output_root, "best_run_command.txt")
    with open(rerun_cmd_path, "w", encoding="utf-8") as f:
        f.write(rerun_cmd + "\n")

    finetune_cmd = (
        "venv/bin/python drqn_minimal.py "
        f"--episodes {args.episodes} --max-steps {args.max_steps} "
        f"--max-neighbors {args.max_neighbors} "
        f"--goal-reward-base {args.goal_reward_base} "
        f"--goal-reward-time-bonus {args.goal_reward_time_bonus} "
        f"--w-reached {args.w_reached} --w-alive {args.w_alive} "
        f"--w-exposure {args.w_exposure} --w-time {args.w_time} --w-progress {args.w_progress} "
        f"--snow-dynamic {args.snow_dynamic} --snow-start-zero {args.snow_start_zero} "
        f"--block-init-prob {args.block_init_prob} "
        f"--block-from-snow-threshold {args.block_from_snow_threshold} "
        f"--block-from-snow-prob {args.block_from_snow_prob} "
        f"--hidden-dim {args.hidden_dim} --lr {args.lr} --gamma {args.gamma} "
        f"--eps-start {args.eps_start} --eps-end {args.eps_end} --eps-decay {args.eps_decay} "
        f"--target-update {args.target_update} --grad-clip {args.grad_clip} "
        f"--replay-episodes {args.replay_episodes} --batch-size {args.batch_size} "
        f"--warmup-steps {args.warmup_steps} --updates-per-step {args.updates_per_step} "
        f"--seq-len {args.seq_len} --burn-in {args.burn_in} --best-window {args.best_window} "
        f"--curriculum-start-dist {args.curriculum_start_dist} "
        f"--curriculum-end-dist {args.curriculum_end_dist} "
        f"--curriculum-full-at-episode {args.curriculum_full_at_episode} "
        f"--curriculum-freeze-episode {args.curriculum_freeze_episode} "
        f"--seed {best_seed} --init-checkpoint {best_model_path} "
        f"--output-dir {os.path.join(args.output_root, 'finetune_from_best')} --device {args.device}"
    )
    finetune_cmd_path = os.path.join(args.output_root, "best_model_finetune_command.txt")
    with open(finetune_cmd_path, "w", encoding="utf-8") as f:
        f.write(finetune_cmd + "\n")

    out_png = os.path.join(args.output_root, "drqn_training_curve_multiseed.png")
    _plot_histories(seed_infos, out_png, show_plot=args.show_plot)

    print(f"[multiseed] summary csv: {summary_csv}")
    print(f"[multiseed] aggregate txt: {agg_txt}")
    print(f"[multiseed] best config: {best_cfg_path}")
    print(f"[multiseed] best model: {best_model_path}")
    print(f"[multiseed] best command: {rerun_cmd_path}")
    print(f"[multiseed] best finetune command: {finetune_cmd_path}")
    print(f"[multiseed] final best-eval csv: {final_details_csv}")
    print(f"[multiseed] final best-eval summary: {final_summary_json}")
    print(f"[multiseed] training curve: {out_png}")
    print(f"[multiseed] aggregate: {agg}")


def main():
    parser = argparse.ArgumentParser(description="Run fixed-parameter DRQN over 3 seeds with eval and plot.")
    parser.add_argument("--preset", default="none", choices=["none", "stable_v1", "stable_v2"])
    parser.add_argument("--output-root", default="logs/drqn_multiseed")
    parser.add_argument("--seeds", default="42,52,62")
    parser.add_argument("--episodes", type=int, default=600)
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--max-neighbors", type=int, default=8)
    parser.add_argument("--goal-reward-base", type=float, default=100.0)
    parser.add_argument("--goal-reward-time-bonus", type=float, default=0.5)
    parser.add_argument("--w-reached", type=float, default=0.0)
    parser.add_argument("--w-alive", type=float, default=0.0)
    parser.add_argument("--w-exposure", type=float, default=2.0)
    parser.add_argument("--w-time", type=float, default=1.0)
    parser.add_argument("--w-progress", type=float, default=0.01)
    parser.add_argument("--snow-dynamic", type=lambda x: str(x).lower() in {"1", "true", "yes"}, default=True)
    parser.add_argument("--snow-start-zero", type=lambda x: str(x).lower() in {"1", "true", "yes"}, default=True)
    parser.add_argument("--block-init-prob", type=float, default=0.0)
    parser.add_argument("--block-from-snow-threshold", type=float, default=0.72)
    parser.add_argument("--block-from-snow-prob", type=float, default=0.003)
    parser.add_argument("--hidden-dim", type=int, default=64)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--eps-start", type=float, default=1.0)
    parser.add_argument("--eps-end", type=float, default=0.05)
    parser.add_argument("--eps-decay", type=float, default=0.995)
    parser.add_argument("--target-update", type=int, default=300)
    parser.add_argument("--grad-clip", type=float, default=1.0)
    parser.add_argument("--replay-episodes", type=int, default=600)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--warmup-steps", type=int, default=500)
    parser.add_argument("--updates-per-step", type=int, default=1)
    parser.add_argument("--seq-len", type=int, default=8)
    parser.add_argument("--burn-in", type=int, default=4)
    parser.add_argument("--best-window", type=int, default=20)
    parser.add_argument("--curriculum-start-dist", type=float, default=2000.0)
    parser.add_argument("--curriculum-end-dist", type=float, default=12000.0)
    parser.add_argument("--curriculum-full-at-episode", type=int, default=300)
    parser.add_argument("--curriculum-freeze-episode", type=int, default=-1)
    parser.add_argument("--log-every", type=int, default=20)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda", "mps"])
    parser.add_argument("--eval-episodes", type=int, default=200)
    parser.add_argument("--eval-seed-base", type=int, default=1000)
    parser.add_argument("--final-eval-episodes", type=int, default=300)
    parser.add_argument("--final-eval-seed", type=int, default=2026)
    parser.add_argument("--show-plot", action="store_true")
    args = parser.parse_args()

    if args.preset == "stable_v1":
        args.episodes = 600
        args.max_steps = 120
        args.max_neighbors = 8
        args.lr = 3e-4
        args.eps_start = 1.0
        args.eps_end = 0.05
        args.eps_decay = 0.998
        args.replay_episodes = 1200
        args.warmup_steps = 500
        args.target_update = 500
        args.batch_size = 64
        args.seq_len = 8
        args.burn_in = 4
        args.curriculum_start_dist = 300.0
        args.curriculum_end_dist = 2500.0
        args.curriculum_full_at_episode = 400
        args.curriculum_freeze_episode = -1
        args.w_reached = 120.0
        args.w_alive = 0.0
        args.w_exposure = 2.5
        args.w_time = 1.0
        args.w_progress = 0.02
        args.goal_reward_base = 0.0
        args.goal_reward_time_bonus = 0.0
        args.snow_dynamic = True
        args.snow_start_zero = True
        args.block_init_prob = 0.0
        args.block_from_snow_threshold = 0.72
        args.block_from_snow_prob = 0.003
    elif args.preset == "stable_v2":
        args.episodes = 800
        args.max_steps = 100
        args.max_neighbors = 8
        args.lr = 3e-4
        args.eps_start = 1.0
        args.eps_end = 0.05
        args.eps_decay = 0.999
        args.replay_episodes = 1500
        args.warmup_steps = 500
        args.target_update = 500
        args.batch_size = 64
        args.seq_len = 8
        args.burn_in = 4
        args.curriculum_start_dist = 150.0
        args.curriculum_end_dist = 600.0
        args.curriculum_full_at_episode = 700
        args.curriculum_freeze_episode = 300
        args.w_reached = 220.0
        args.w_alive = 0.0
        args.w_exposure = 1.0
        args.w_time = 0.8
        args.w_progress = 0.05
        args.goal_reward_base = 0.0
        args.goal_reward_time_bonus = 0.0
        args.snow_dynamic = True
        args.snow_start_zero = True
        args.block_init_prob = 0.0
        args.block_from_snow_threshold = 0.85
        args.block_from_snow_prob = 0.0005

    run(args)


if __name__ == "__main__":
    main()
