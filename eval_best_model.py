import argparse
import csv
import json
import os
import random

import numpy as np
import torch

from drqn_minimal import GridPOMDPEnv, TorchDRQN


def evaluate_checkpoint(ckpt_path, episodes, max_steps, seed, device):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

    payload = torch.load(ckpt_path, map_location=device)
    obs_dim = int(payload["obs_dim"])
    hidden_dim = int(payload["hidden_dim"])
    num_actions = int(payload.get("num_actions", 5))
    max_neighbors = int(payload.get("max_neighbors", max(1, num_actions - 1)))
    if max_steps is None:
        max_steps = int(payload.get("max_steps", 80))
    snow_dynamic = bool(payload.get("snow_dynamic", True))
    snow_start_zero = bool(payload.get("snow_start_zero", True))
    block_init_prob = float(payload.get("block_init_prob", 0.0))
    block_from_snow_threshold = float(payload.get("block_from_snow_threshold", 0.72))
    block_from_snow_prob = float(payload.get("block_from_snow_prob", 0.003))
    failure_penalty_steps = int(payload.get("failure_penalty_steps", 25))
    failure_block_on_repeat = int(payload.get("failure_block_on_repeat", 2))
    allow_stay_when_move_available = bool(payload.get("allow_stay_when_move_available", True))
    use_subgoals = bool(payload.get("use_subgoals", False))
    checkpoint_interval = int(payload.get("checkpoint_interval", 5))
    w_checkpoint = float(payload.get("w_checkpoint", 30.0))
    dynamic_step_budget = bool(payload.get("dynamic_step_budget", False))
    step_budget_scale = float(payload.get("step_budget_scale", 0.08))
    step_budget_min = int(payload.get("step_budget_min", max_steps))
    step_budget_max = int(payload.get("step_budget_max", max_steps))
    step_budget_slack = float(payload.get("step_budget_slack", 20.0))
    frontier_bonus = float(payload.get("frontier_bonus", 0.0))
    revisit_penalty = float(payload.get("revisit_penalty", 0.0))
    replan_on_block = bool(payload.get("replan_on_block", True))
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
        failure_penalty_steps=failure_penalty_steps,
        failure_block_on_repeat=failure_block_on_repeat,
        allow_stay_when_move_available=allow_stay_when_move_available,
        use_subgoals=use_subgoals,
        checkpoint_interval=checkpoint_interval,
        w_checkpoint=w_checkpoint,
        dynamic_step_budget=dynamic_step_budget,
        step_budget_scale=step_budget_scale,
        step_budget_min=step_budget_min,
        step_budget_max=step_budget_max,
        step_budget_slack=step_budget_slack,
        frontier_bonus=frontier_bonus,
        revisit_penalty=revisit_penalty,
        replan_on_block=replan_on_block,
    )
    returns = []
    reached = []
    steps = []
    rows = []

    with torch.no_grad():
        for ep in range(1, episodes + 1):
            obs = env.reset()
            h = model.init_hidden(device=device)
            done = False
            ep_return = 0.0
            ep_steps = 0
            ep_reached = 0
            while not done:
                ep_steps += 1
                obs_t = torch.tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
                q_t, h = model(obs_t, h)
                mask = env.get_action_mask()
                q_np = q_t.detach().cpu().numpy().squeeze(0)
                q_np[mask <= 0.5] = -1e9
                action = int(np.argmax(q_np))
                obs, reward, done, _ = env.step(action)
                ep_return += reward
                if done and env.node == env.goal_node:
                    ep_reached = 1

            returns.append(ep_return)
            reached.append(ep_reached)
            steps.append(ep_steps)
            rows.append(
                {
                    "episode": ep,
                    "return": ep_return,
                    "reached": ep_reached,
                    "steps": ep_steps,
                }
            )

    summary = {
        "episodes": episodes,
        "seed": seed,
        "checkpoint": ckpt_path,
        "return_mean": float(np.mean(returns)),
        "return_std": float(np.std(returns)),
        "reached_rate": float(np.mean(reached)),
        "steps_mean": float(np.mean(steps)),
        "steps_std": float(np.std(steps)),
    }
    return rows, summary


def main():
    parser = argparse.ArgumentParser(description="Evaluate best DRQN model checkpoint and output KPI.")
    parser.add_argument("--checkpoint", required=True, help="Path to best_model.pt or drqn_torch_best.pt")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--device", default="cpu", choices=["cpu", "cuda", "mps"])
    parser.add_argument("--output-dir", default="logs")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rows, summary = evaluate_checkpoint(
        ckpt_path=args.checkpoint,
        episodes=args.episodes,
        max_steps=args.max_steps,
        seed=args.seed,
        device=torch.device(args.device),
    )

    details_csv = os.path.join(args.output_dir, "best_model_eval_details.csv")
    with open(details_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["episode", "return", "reached", "steps"])
        w.writeheader()
        w.writerows(rows)

    summary_json = os.path.join(args.output_dir, "best_model_eval_summary.json")
    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    summary_txt = os.path.join(args.output_dir, "best_model_eval_summary.txt")
    with open(summary_txt, "w", encoding="utf-8") as f:
        f.write("Best Model Evaluation KPI\n")
        for k, v in summary.items():
            f.write(f"{k}: {v}\n")

    print(f"[eval] details csv: {details_csv}")
    print(f"[eval] summary json: {summary_json}")
    print(f"[eval] summary txt: {summary_txt}")
    print(f"[eval] KPI: {summary}")


if __name__ == "__main__":
    main()
