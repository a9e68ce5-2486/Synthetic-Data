import argparse
import csv
import os
import random
from collections import deque

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


class EpisodeReplayBuffer:
    def __init__(self, capacity_episodes):
        self.capacity_episodes = int(capacity_episodes)
        self.episodes = deque(maxlen=self.capacity_episodes)
        self.total_transitions = 0

    def __len__(self):
        return len(self.episodes)

    def add_episode(self, transitions):
        if not transitions:
            return
        if len(self.episodes) == self.capacity_episodes:
            oldest = self.episodes.popleft()
            self.total_transitions -= len(oldest)
        self.episodes.append(transitions)
        self.total_transitions += len(transitions)

    def _eligible_episodes(self, min_len):
        return [ep for ep in self.episodes if len(ep) >= min_len]

    def sample_sequences(self, batch_size, burn_in, seq_len):
        total_len = burn_in + seq_len
        eligible = self._eligible_episodes(total_len)
        if len(eligible) < batch_size:
            raise ValueError("Not enough eligible episodes for sequence sampling.")

        chosen = random.sample(eligible, batch_size)
        obs_batch = []
        act_batch = []
        rew_batch = []
        nxt_batch = []
        done_batch = []
        for ep in chosen:
            start = random.randint(0, len(ep) - total_len)
            chunk = ep[start : start + total_len]
            obs, act, rew, nxt, done = zip(*chunk)
            obs_batch.append(np.stack(obs).astype(np.float32))
            act_batch.append(np.array(act, dtype=np.int64))
            rew_batch.append(np.array(rew, dtype=np.float32))
            nxt_batch.append(np.stack(nxt).astype(np.float32))
            done_batch.append(np.array(done, dtype=np.float32))

        return (
            np.stack(obs_batch),   # [B, T, obs]
            np.stack(act_batch),   # [B, T]
            np.stack(rew_batch),   # [B, T]
            np.stack(nxt_batch),   # [B, T, obs]
            np.stack(done_batch),  # [B, T]
        )


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
    best_ckpt_path = os.path.join(args.output_dir, "drqn_torch_best.pt")
    replay = EpisodeReplayBuffer(args.replay_episodes)

    eps = args.eps_start
    global_step = 0
    train_updates = 0
    returns = []
    best_ma = -float("inf")
    with open(history_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["episode", "return", "steps", "reached", "avg_loss", "eps", "moving_avg_return", "buffer_size"])

        for ep in range(1, args.episodes + 1):
            obs_np = env.reset()
            h = policy_net.init_hidden(device=device)
            done = False
            ep_return = 0.0
            losses = []
            reached = 0
            step = 0
            episode_transitions = []

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

                episode_transitions.append((obs_np, action, reward, next_obs_np, float(done)))

                if replay.total_transitions >= args.warmup_steps and len(replay) >= args.batch_size:
                    for _ in range(args.updates_per_step):
                        obs_seq, act_seq, rew_seq, nxt_seq, done_seq = replay.sample_sequences(
                            batch_size=args.batch_size,
                            burn_in=args.burn_in,
                            seq_len=args.seq_len,
                        )
                        obs_t = torch.tensor(obs_seq, dtype=torch.float32, device=device)   # [B,T,O]
                        act_t = torch.tensor(act_seq, dtype=torch.int64, device=device)      # [B,T]
                        rew_t = torch.tensor(rew_seq, dtype=torch.float32, device=device)    # [B,T]
                        nxt_t = torch.tensor(nxt_seq, dtype=torch.float32, device=device)    # [B,T,O]
                        done_t = torch.tensor(done_seq, dtype=torch.float32, device=device)  # [B,T]

                        h_online = policy_net.init_hidden(batch_size=args.batch_size, device=device)
                        h_target = target_net.init_hidden(batch_size=args.batch_size, device=device)
                        h_online_for_next = policy_net.init_hidden(batch_size=args.batch_size, device=device)

                        # Burn-in: warm hidden states without contributing to loss.
                        if args.burn_in > 0:
                            with torch.no_grad():
                                for t in range(args.burn_in):
                                    _, h_online = policy_net(obs_t[:, t, :], h_online)
                                    _, h_target = target_net(obs_t[:, t, :], h_target)
                                    _, h_online_for_next = policy_net(obs_t[:, t, :], h_online_for_next)

                        seq_losses = []
                        for u in range(args.seq_len):
                            t_idx = args.burn_in + u
                            q_online, h_online = policy_net(obs_t[:, t_idx, :], h_online)
                            a_t = act_t[:, t_idx].unsqueeze(1)
                            q_selected = q_online.gather(1, a_t).squeeze(1)

                            with torch.no_grad():
                                # Double DQN: action from online net, value from target net.
                                q_next_online, h_online_for_next = policy_net(nxt_t[:, t_idx, :], h_online_for_next)
                                next_action = q_next_online.argmax(dim=1, keepdim=True)
                                q_next_target, h_target = target_net(nxt_t[:, t_idx, :], h_target)
                                next_q = q_next_target.gather(1, next_action).squeeze(1)
                                td_target = rew_t[:, t_idx] + args.gamma * (1.0 - done_t[:, t_idx]) * next_q

                            seq_losses.append(loss_fn(q_selected, td_target))

                        loss = torch.stack(seq_losses).mean()
                        optimizer.zero_grad()
                        loss.backward()
                        nn.utils.clip_grad_norm_(policy_net.parameters(), args.grad_clip)
                        optimizer.step()

                        train_updates += 1
                        if train_updates % args.target_update == 0:
                            target_net.load_state_dict(policy_net.state_dict())

                        losses.append(float(loss.item()))

                ep_return += reward
                obs_np = next_obs_np
                h = h_next.detach()

            replay.add_episode(episode_transitions)

            avg_loss = float(np.mean(losses)) if losses else 0.0
            returns.append(ep_return)
            window = max(1, int(args.best_window))
            moving_avg = float(np.mean(returns[-window:]))
            w.writerow(
                [
                    ep,
                    f"{ep_return:.4f}",
                    step,
                    reached,
                    f"{avg_loss:.6f}",
                    f"{eps:.4f}",
                    f"{moving_avg:.4f}",
                    replay.total_transitions,
                ]
            )

            if moving_avg > best_ma:
                best_ma = moving_avg
                torch.save(
                    {
                        "model_state_dict": policy_net.state_dict(),
                        "obs_dim": obs_dim,
                        "hidden_dim": args.hidden_dim,
                        "num_actions": NUM_ACTIONS,
                        "episode": ep,
                        "moving_avg_return": moving_avg,
                    },
                    best_ckpt_path,
                )

            if ep % args.log_every == 0 or ep == 1:
                print(
                    f"[drqn] ep={ep:4d} return={ep_return:8.3f} steps={step:3d} "
                    f"reached={reached} avg_loss={avg_loss:.6f} eps={eps:.3f} "
                    f"ma_return={moving_avg:8.3f} best_ma={best_ma:8.3f} "
                    f"trans={replay.total_transitions:6d} eps_buf={len(replay):4d}",
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
    print(f"[drqn] best weights: {best_ckpt_path} (best moving_avg_return={best_ma:.3f})")


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
    parser.add_argument("--replay-episodes", type=int, default=600)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--warmup-steps", type=int, default=500)
    parser.add_argument("--updates-per-step", type=int, default=1)
    parser.add_argument("--seq-len", type=int, default=8)
    parser.add_argument("--burn-in", type=int, default=4)
    parser.add_argument("--best-window", type=int, default=20)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output-dir", default="logs")
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda", "mps"])
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
