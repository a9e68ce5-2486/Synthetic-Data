import argparse
import csv
import os
import random

import numpy as np

from evac_env import EvacEnv
from scenario_loader import temporary_config

try:
    import torch
    import torch.nn as nn
except Exception as e:  # pragma: no cover - runtime dependency guard
    raise SystemExit(
        "PyTorch is required for drqn_minimal.py.\n"
        f"Import error: {e}\n"
        "Install in this project venv, for example:\n"
        "  venv/bin/python -m pip install torch"
    )


ACTION_STAY = 0
ACTION_NORTH = 1
ACTION_SOUTH = 2
ACTION_WEST = 3
ACTION_EAST = 4
NUM_ACTIONS = 5


class GridPOMDPEnv:
    def __init__(self, max_steps=80):
        self.max_steps = max_steps
        self.env = None
        self.start_node = None
        self.goal_node = None
        self.node = None
        self.steps = 0

    def reset(self):
        with temporary_config(
            {
                "EVAC_USE_OSM": False,
                "EVAC_SNOW_DYNAMIC": True,
                "EVAC_SNOW_START_ZERO": True,
                "EVAC_BLOCK_INIT_PROB": 0.0,
                "EVAC_BLOCK_FROM_SNOW_THRESHOLD": 0.72,
                "EVAC_BLOCK_FROM_SNOW_PROB": 0.003,
            }
        ):
            self.env = EvacEnv()

        nodes = list(self.env.G_walk.nodes())
        self.goal_node = random.choice(list(self.env.shelters)) if self.env.shelters else random.choice(nodes)
        valid_starts = [n for n in nodes if n != self.goal_node]
        self.start_node = random.choice(valid_starts) if valid_starts else self.goal_node
        self.node = self.start_node
        self.steps = 0
        return self._obs()

    def _neighbor_for_action(self, node, action):
        if not isinstance(node, tuple) or len(node) != 2:
            return None
        i, j = node
        if action == ACTION_NORTH:
            return (i, j + 1)
        if action == ACTION_SOUTH:
            return (i, j - 1)
        if action == ACTION_WEST:
            return (i - 1, j)
        if action == ACTION_EAST:
            return (i + 1, j)
        return node

    def _obs(self):
        i, j = self.node
        gi, gj = self.goal_node
        dx = (gi - i) / 6.0
        dy = (gj - j) / 6.0
        local = self.env.observe(self.node, mode="walk")
        blocked_ratio = 0.0
        snow_avg = 0.0
        if local:
            blocked_ratio = sum(1.0 for v in local.values() if v["blocked"]) / len(local)
            snow_avg = sum(float(v["snow"]) for v in local.values()) / len(local)
        t = self.steps / float(self.max_steps)
        return np.array([dx, dy, blocked_ratio, snow_avg, t], dtype=np.float32)

    def step(self, action):
        self.steps += 1
        self.env.step_hazards()

        reward = -1.0
        done = False

        nxt = self._neighbor_for_action(self.node, action)
        if nxt is None or nxt == self.node:
            reward -= 0.2
        elif not self.env.G_walk.has_edge(self.node, nxt):
            reward -= 1.0
        elif self.env.is_blocked(self.node, nxt, mode="walk", belief=None):
            reward -= 5.0
        else:
            snow = self.env.snow_depth_walk.get((self.node, nxt), 0.0)
            reward -= 2.0 * float(snow)
            self.node = nxt

        if self.node == self.goal_node:
            reward += 100.0
            done = True
        elif self.steps >= self.max_steps:
            done = True

        return self._obs(), reward, done, {}


class TorchDRQN(nn.Module):
    def __init__(self, obs_dim, hidden_dim):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.gru = nn.GRUCell(obs_dim, hidden_dim)
        self.q_head = nn.Linear(hidden_dim, NUM_ACTIONS)

    def init_hidden(self, batch_size=1, device="cpu"):
        return torch.zeros(batch_size, self.hidden_dim, device=device)

    def forward(self, obs_t, h_prev):
        h = self.gru(obs_t, h_prev)
        q = self.q_head(h)
        return q, h


def _pick_device(device_arg):
    if device_arg != "auto":
        return torch.device(device_arg)
    if torch.cuda.is_available():
        return torch.device("cuda")
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


def train(args):
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    device = _pick_device(args.device)
    print(f"[drqn] device={device}")

    env = GridPOMDPEnv(max_steps=args.max_steps)
    obs0 = env.reset()
    obs_dim = obs0.shape[0]

    policy_net = TorchDRQN(obs_dim=obs_dim, hidden_dim=args.hidden_dim).to(device)
    target_net = TorchDRQN(obs_dim=obs_dim, hidden_dim=args.hidden_dim).to(device)
    target_net.load_state_dict(policy_net.state_dict())
    target_net.eval()

    optimizer = torch.optim.Adam(policy_net.parameters(), lr=args.lr)
    loss_fn = nn.SmoothL1Loss()

    os.makedirs(args.output_dir, exist_ok=True)
    history_path = os.path.join(args.output_dir, "drqn_torch_history.csv")
    ckpt_path = os.path.join(args.output_dir, "drqn_torch_weights.pt")

    eps = args.eps_start
    global_step = 0
    with open(history_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["episode", "return", "steps", "reached", "avg_loss", "eps"])

        for ep in range(1, args.episodes + 1):
            obs_np = env.reset()
            h = policy_net.init_hidden(device=device)
            done = False
            ep_return = 0.0
            losses = []
            reached = 0
            step = 0

            while not done:
                step += 1
                global_step += 1

                obs_t = torch.tensor(obs_np, dtype=torch.float32, device=device).unsqueeze(0)
                q_t, h_next = policy_net(obs_t, h)
                if random.random() < eps:
                    action = random.randrange(NUM_ACTIONS)
                else:
                    action = int(torch.argmax(q_t, dim=1).item())

                next_obs_np, reward, done, _ = env.step(action)
                if done and env.node == env.goal_node:
                    reached = 1

                q_selected = q_t[0, action]

                with torch.no_grad():
                    next_obs_t = torch.tensor(next_obs_np, dtype=torch.float32, device=device).unsqueeze(0)
                    q_next_target, _ = target_net(next_obs_t, h_next.detach())
                    max_next = torch.max(q_next_target, dim=1).values[0]
                    td_target = torch.tensor(
                        reward + (0.0 if done else args.gamma * float(max_next.item())),
                        dtype=torch.float32,
                        device=device,
                    )

                loss = loss_fn(q_selected, td_target)
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(policy_net.parameters(), args.grad_clip)
                optimizer.step()

                if global_step % args.target_update == 0:
                    target_net.load_state_dict(policy_net.state_dict())

                losses.append(float(loss.item()))
                ep_return += reward
                obs_np = next_obs_np
                h = h_next.detach()

            avg_loss = float(np.mean(losses)) if losses else 0.0
            w.writerow([ep, f"{ep_return:.4f}", step, reached, f"{avg_loss:.6f}", f"{eps:.4f}"])

            if ep % args.log_every == 0 or ep == 1:
                print(
                    f"[drqn] ep={ep:4d} return={ep_return:8.3f} steps={step:3d} "
                    f"reached={reached} avg_loss={avg_loss:.6f} eps={eps:.3f}",
                    flush=True,
                )

            eps = max(args.eps_end, eps * args.eps_decay)

    torch.save(
        {
            "model_state_dict": policy_net.state_dict(),
            "obs_dim": obs_dim,
            "hidden_dim": args.hidden_dim,
            "num_actions": NUM_ACTIONS,
        },
        ckpt_path,
    )
    print(f"[drqn] history: {history_path}")
    print(f"[drqn] weights: {ckpt_path}")


def main():
    parser = argparse.ArgumentParser(description="Minimal runnable DRQN baseline (PyTorch)")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--hidden-dim", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--eps-start", type=float, default=1.0)
    parser.add_argument("--eps-end", type=float, default=0.05)
    parser.add_argument("--eps-decay", type=float, default=0.995)
    parser.add_argument("--target-update", type=int, default=100)
    parser.add_argument("--grad-clip", type=float, default=1.0)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output-dir", default="logs")
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda", "mps"])
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
