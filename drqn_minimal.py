import argparse
import csv
import json
import os
import random
from collections import deque

import numpy as np
import networkx as nx

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


class GridPOMDPEnv:
    def __init__(
        self,
        max_steps=80,
        max_neighbors=8,
        goal_reward_base=100.0,
        goal_reward_time_bonus=0.0,
        w_reached=0.0,
        w_alive=0.0,
        w_exposure=2.0,
        w_time=1.0,
        w_progress=0.01,
        snow_dynamic=True,
        snow_start_zero=True,
        block_init_prob=0.0,
        block_from_snow_threshold=0.72,
        block_from_snow_prob=0.003,
        failure_penalty_steps=25,
        failure_block_on_repeat=2,
        allow_stay_when_move_available=True,
        use_subgoals=False,
        checkpoint_interval=5,
        w_checkpoint=30.0,
        dynamic_step_budget=False,
        step_budget_scale=0.08,
        step_budget_min=100,
        step_budget_max=400,
        step_budget_slack=20,
        frontier_bonus=0.0,
        revisit_penalty=0.0,
        replan_on_block=True,
        domain_rand=False,
        domain_rand_block_init_max=0.12,
        domain_rand_snow_threshold_min=0.60,
        domain_rand_snow_threshold_max=0.92,
        domain_rand_snow_prob_max=0.007,
    ):
        self.base_max_steps = int(max_steps)
        self.max_steps = int(max_steps)
        self.max_neighbors = int(max_neighbors)
        self.num_actions = 1 + self.max_neighbors  # stay + k neighbors
        self.goal_reward_base = float(goal_reward_base)
        self.goal_reward_time_bonus = float(goal_reward_time_bonus)
        self.w_reached = float(w_reached)
        self.w_alive = float(w_alive)
        self.w_exposure = float(w_exposure)
        self.w_time = float(w_time)
        self.w_progress = float(w_progress)
        self.snow_dynamic = bool(snow_dynamic)
        self.snow_start_zero = bool(snow_start_zero)
        self.block_init_prob = float(block_init_prob)
        self.block_from_snow_threshold = float(block_from_snow_threshold)
        self.block_from_snow_prob = float(block_from_snow_prob)
        self.failure_penalty_steps = int(failure_penalty_steps)
        self.failure_block_on_repeat = int(failure_block_on_repeat)
        self.allow_stay_when_move_available = bool(allow_stay_when_move_available)
        self.use_subgoals = bool(use_subgoals)
        self.checkpoint_interval = int(checkpoint_interval)
        self.w_checkpoint = float(w_checkpoint)
        self.dynamic_step_budget = bool(dynamic_step_budget)
        self.step_budget_scale = float(step_budget_scale)
        self.step_budget_min = int(step_budget_min)
        self.step_budget_max = int(step_budget_max)
        self.step_budget_slack = int(step_budget_slack)
        self.frontier_bonus = float(frontier_bonus)
        self.revisit_penalty = float(revisit_penalty)
        self.replan_on_block = bool(replan_on_block)
        self.domain_rand = bool(domain_rand)
        self.domain_rand_block_init_max = float(domain_rand_block_init_max)
        self.domain_rand_snow_threshold_min = float(domain_rand_snow_threshold_min)
        self.domain_rand_snow_threshold_max = float(domain_rand_snow_threshold_max)
        self.domain_rand_snow_prob_max = float(domain_rand_snow_prob_max)
        self.env = None
        self.start_node = None
        self.goal_node = None
        self.node = None
        self.steps = 0
        self._coord_scale = 1.0
        self._prev_goal_dist = None
        self._curr_action_mask = np.zeros(self.num_actions, dtype=np.float32)
        self._curr_candidates = []
        self.obs_dim = 12 + 5 * self.max_neighbors  # 7 base + 5 persona + 5*k neighbors
        self._persona_features = np.zeros(5, dtype=np.float32)  # [speed, panic, familiarity, compliance, obs_err]
        self._failure_memory = {}
        self._checkpoint_nodes = []
        self._checkpoint_idx = 0
        self.current_target = None
        self.start_goal_dist = 0.0
        self.raw_step_budget = float(self.max_steps)
        self._visit_counts = {}

    def reset(self, max_start_goal_dist=None, start_coverage_ratio=None):
        if self.domain_rand:
            ep_block_init = random.uniform(0.0, self.domain_rand_block_init_max)
            ep_threshold = random.uniform(
                self.domain_rand_snow_threshold_min,
                self.domain_rand_snow_threshold_max,
            )
            ep_snow_prob = random.uniform(
                self.block_from_snow_prob, self.domain_rand_snow_prob_max
            )
        else:
            ep_block_init = self.block_init_prob
            ep_threshold = self.block_from_snow_threshold
            ep_snow_prob = self.block_from_snow_prob
        with temporary_config(
            {
                "EVAC_USE_OSM": True,
                "EVAC_SNOW_DYNAMIC": self.snow_dynamic,
                "EVAC_SNOW_START_ZERO": self.snow_start_zero,
                "EVAC_BLOCK_INIT_PROB": ep_block_init,
                "EVAC_BLOCK_FROM_SNOW_THRESHOLD": ep_threshold,
                "EVAC_BLOCK_FROM_SNOW_PROB": ep_snow_prob,
            }
        ):
            self.env = EvacEnv()

        nodes = list(self.env.G_walk.nodes())
        if not nodes:
            raise RuntimeError("OSM graph build failed: no nodes available.")

        # Enforce no fallback grid for DRQN runs.
        n0 = nodes[0]
        if isinstance(n0, tuple) and len(n0) == 2 and all(isinstance(v, int) for v in n0):
            raise RuntimeError(
                "Fallback grid was detected, but DRQN is configured to require OSM. "
                "Ensure OSM build works and EVAC_USE_OSM remains enabled."
            )

        xs = [self.env.pos[n][0] for n in nodes if n in self.env.pos]
        ys = [self.env.pos[n][1] for n in nodes if n in self.env.pos]
        span_x = (max(xs) - min(xs)) if xs else 1.0
        span_y = (max(ys) - min(ys)) if ys else 1.0
        self._coord_scale = max(1.0, span_x, span_y)

        self.goal_node = random.choice(list(self.env.shelters)) if self.env.shelters else random.choice(nodes)
        walk_graph = self._walk_graph_view()
        reachable_starts = []
        for n in nodes:
            if n == self.goal_node:
                continue
            try:
                if nx.has_path(walk_graph, n, self.goal_node):
                    d = float(nx.shortest_path_length(walk_graph, n, self.goal_node, weight="weight"))
                    reachable_starts.append((d, n))
            except Exception:
                continue
        reachable_starts.sort(key=lambda item: item[0])

        valid_starts = []
        if start_coverage_ratio is not None and reachable_starts:
            frac = max(0.0, min(1.0, float(start_coverage_ratio)))
            keep_n = max(1, int(np.ceil(frac * len(reachable_starts))))
            valid_starts = [n for _, n in reachable_starts[:keep_n]]
        elif max_start_goal_dist is not None and reachable_starts:
            valid_starts = [n for d, n in reachable_starts if d <= max_start_goal_dist]
        else:
            valid_starts = [n for _, n in reachable_starts]

        if not valid_starts:
            valid_starts = [n for n in nodes if n != self.goal_node]
        self.start_node = random.choice(valid_starts) if valid_starts else self.goal_node
        self.node = self.start_node
        self.steps = 0
        try:
            self.start_goal_dist = float(
                nx.shortest_path_length(walk_graph, self.start_node, self.goal_node, weight="weight")
            )
        except Exception:
            self.start_goal_dist = 0.0
        if self.dynamic_step_budget:
            self.raw_step_budget = self.step_budget_scale * self.start_goal_dist + self.step_budget_slack
            budget = int(self.raw_step_budget)
            budget = max(self.step_budget_min, min(self.step_budget_max, budget))
            self.max_steps = budget
        else:
            self.raw_step_budget = float(self.base_max_steps)
            self.max_steps = self.base_max_steps
        self._failure_memory = {}
        self._visit_counts = {self.start_node: 1}
        self._checkpoint_nodes = self._build_checkpoints(self.start_node, self.goal_node)
        self._checkpoint_idx = 0
        self.current_target = self._checkpoint_nodes[0] if self._checkpoint_nodes else self.goal_node
        self._prev_goal_dist = self._goal_dist()
        self._curr_candidates, self._curr_action_mask = self._action_context(self.node)
        return self._obs()

    def _walk_graph_view(self):
        blocked = set(getattr(self.env, "blocked_edges_walk", set()))
        if not blocked:
            return self.env.G_walk
        keep_edges = [(u, v) for u, v in self.env.G_walk.edges() if (u, v) not in blocked]
        return self.env.G_walk.edge_subgraph(keep_edges).copy()

    def _build_checkpoints(self, start, goal):
        if not self.use_subgoals or self.checkpoint_interval <= 0:
            return [goal]
        try:
            path = nx.shortest_path(self._walk_graph_view(), start, goal, weight="weight")
        except Exception:
            return [goal]
        if len(path) <= 2:
            return [goal]
        checkpoints = []
        for idx in range(self.checkpoint_interval, len(path) - 1, self.checkpoint_interval):
            checkpoints.append(path[idx])
        if not checkpoints or checkpoints[-1] != goal:
            checkpoints.append(goal)
        return checkpoints

    def _target_node(self):
        return self.current_target if self.current_target is not None else self.goal_node

    def _refresh_targets(self):
        replanned = self._build_checkpoints(self.node, self.goal_node)
        self._checkpoint_nodes = replanned
        self._checkpoint_idx = 0
        self.current_target = replanned[0] if replanned else self.goal_node
        self._prev_goal_dist = self._goal_dist()

    def _advance_target(self):
        if self._checkpoint_idx + 1 < len(self._checkpoint_nodes):
            self._checkpoint_idx += 1
            self.current_target = self._checkpoint_nodes[self._checkpoint_idx]
            self._prev_goal_dist = self._goal_dist()
            return True
        self.current_target = self.goal_node
        return False

    def _candidate_neighbors(self, node):
        if node not in self.env.G_walk:
            return []
        succ = list(self.env.G_walk.successors(node))
        if not succ:
            return []

        # Rank successors by geometric progress toward goal, then shorter edge.
        target = self._target_node()
        gx, gy = self.env.pos.get(target, (0.0, 0.0))
        x0, y0 = self.env.pos.get(node, (0.0, 0.0))
        d0 = float(np.hypot(gx - x0, gy - y0))
        graph_view = self._walk_graph_view()
        try:
            base_target_dist = float(nx.shortest_path_length(graph_view, node, target, weight="weight"))
        except Exception:
            base_target_dist = float("inf")
        ranked = []
        for nxt in succ:
            if self.env.is_blocked(node, nxt, mode="walk", belief=None):
                continue
            x1, y1 = self.env.pos.get(nxt, (x0, y0))
            d1 = float(np.hypot(gx - x1, gy - y1))
            geo_progress = d0 - d1
            try:
                nxt_target_dist = float(nx.shortest_path_length(graph_view, nxt, target, weight="weight"))
            except Exception:
                nxt_target_dist = float("inf")
            if np.isfinite(base_target_dist) and np.isfinite(nxt_target_dist):
                graph_progress = base_target_dist - nxt_target_dist
            else:
                graph_progress = -float("inf")
            edge_w = float(self.env.G_walk[node][nxt].get("weight", 1.0))
            visit_count = self._visit_counts.get(nxt, 0)
            ranked.append((graph_progress, geo_progress, -visit_count, -edge_w, nxt))
        ranked.sort(reverse=True)
        return [n for *_rest, n in ranked[: self.max_neighbors]]

    def _action_context(self, node):
        cand = self._candidate_neighbors(node)
        mask = np.zeros(self.num_actions, dtype=np.float32)
        if cand:
            for idx, nxt in enumerate(cand, start=1):
                info = self._failure_memory.get((node, nxt))
                if info is None:
                    mask[idx] = 1.0
                    continue
                fail_count, expires_at = info
                if self.steps >= expires_at or fail_count < self.failure_block_on_repeat:
                    mask[idx] = 1.0
        if self.allow_stay_when_move_available:
            mask[ACTION_STAY] = 1.0
        elif np.any(mask[1:] > 0.5):
            mask[ACTION_STAY] = 0.0
        else:
            mask[ACTION_STAY] = 1.0
        return cand, mask

    def _record_failure(self, u, v):
        if u is None or v is None:
            return
        fail_count, expires_at = self._failure_memory.get((u, v), (0, -1))
        fail_count += 1
        self._failure_memory[(u, v)] = (fail_count, self.steps + self.failure_penalty_steps)

    def _clear_failure(self, u, v):
        if u is None or v is None:
            return
        self._failure_memory.pop((u, v), None)

    def set_persona(self, walk_speed_multiplier=1.0, panic_level=0.0,
                    shelter_familiarity=0.5, compliance_rate=0.8,
                    observation_error_multiplier=1.0):
        """Set normalized persona features for the current episode."""
        eff_compliance = compliance_rate * (1.0 - 0.5 * panic_level)
        eff_obs_err = observation_error_multiplier * (1.0 + panic_level) / 3.0
        self._persona_features = np.array([
            min(1.0, walk_speed_multiplier / 1.5),
            float(panic_level),
            float(shelter_familiarity),
            min(1.0, max(0.0, eff_compliance)),
            min(1.0, max(0.0, eff_obs_err)),
        ], dtype=np.float32)

    def _obs(self):
        target = self._target_node()
        if self.node not in self.env.pos or target not in self.env.pos:
            dx = 0.0
            dy = 0.0
        else:
            x, y = self.env.pos[self.node]
            gx, gy = self.env.pos[target]
            dx = (gx - x) / self._coord_scale
            dy = (gy - y) / self._coord_scale
        if self.node not in self.env.pos or self.goal_node not in self.env.pos:
            final_dx = 0.0
            final_dy = 0.0
        else:
            x, y = self.env.pos[self.node]
            gx, gy = self.env.pos[self.goal_node]
            final_dx = (gx - x) / self._coord_scale
            final_dy = (gy - y) / self._coord_scale
        local = self.env.observe(self.node, mode="walk")
        blocked_ratio = 0.0
        snow_avg = 0.0
        if local:
            blocked_ratio = sum(1.0 for v in local.values() if v["blocked"]) / len(local)
            snow_avg = sum(float(v["snow"]) for v in local.values()) / len(local)
        t = self.steps / float(self.max_steps)
        features = [dx, dy, final_dx, final_dy, blocked_ratio, snow_avg, t]

        # Persona features (5 dims): normalized so values stay in ~[0, 1]
        features.extend(self._persona_features.tolist())

        base_goal_dist = self._goal_dist()
        for idx in range(self.max_neighbors):
            if idx < len(self._curr_candidates):
                nxt = self._curr_candidates[idx]
                edge_w = float(self.env.G_walk[self.node][nxt].get("weight", 1.0))
                edge_len_norm = min(1.0, edge_w / max(1.0, self._coord_scale))
                edge_snow = float(self.env.snow_depth_walk.get((self.node, nxt), 0.0))
                edge_blocked = 1.0 if self.env.is_blocked(self.node, nxt, mode="walk", belief=None) else 0.0
                try:
                    nxt_goal_dist = float(
                        nx.shortest_path_length(self.env.G_walk, nxt, self._target_node(), weight="weight")
                    )
                except Exception:
                    nxt_goal_dist = float("inf")
                if np.isfinite(base_goal_dist) and np.isfinite(nxt_goal_dist):
                    progress = (base_goal_dist - nxt_goal_dist) / max(1.0, self._coord_scale)
                else:
                    progress = -1.0
                features.extend(
                    [
                        1.0,
                        max(-1.0, min(1.0, progress)),
                        edge_len_norm,
                        max(0.0, min(1.0, edge_snow)),
                        edge_blocked,
                    ]
                )
            else:
                features.extend([0.0, 0.0, 0.0, 0.0, 0.0])
        return np.array(features, dtype=np.float32)

    def get_action_mask(self):
        return self._curr_action_mask.copy()

    def _goal_dist(self):
        try:
            return float(nx.shortest_path_length(self._walk_graph_view(), self.node, self._target_node(), weight="weight"))
        except Exception:
            return float("inf")

    def step(self, action):
        self.steps += 1
        self.env.step_hazards()

        reward = -self.w_time
        done = False
        alive = 1

        nxt = None
        if action == ACTION_STAY:
            nxt = self.node
        elif 1 <= action <= len(self._curr_candidates):
            nxt = self._curr_candidates[action - 1]
        exposure = 0.0
        if nxt is None or nxt == self.node:
            reward -= 0.2
            local = self.env.observe(self.node, mode="walk")
            if local:
                exposure = sum(float(v["snow"]) for v in local.values()) / len(local)
            if nxt is not None and nxt == self.node:
                self._record_failure(self.node, nxt)
        elif not self.env.G_walk.has_edge(self.node, nxt):
            reward -= 1.0
            local = self.env.observe(self.node, mode="walk")
            if local:
                exposure = sum(float(v["snow"]) for v in local.values()) / len(local)
            self._record_failure(self.node, nxt)
        elif self.env.is_blocked(self.node, nxt, mode="walk", belief=None):
            reward -= 5.0
            local = self.env.observe(self.node, mode="walk")
            if local:
                exposure = sum(float(v["snow"]) for v in local.values()) / len(local)
            self._record_failure(self.node, nxt)
            if self.replan_on_block:
                self._refresh_targets()
        else:
            exposure = float(self.env.snow_depth_walk.get((self.node, nxt), 0.0))
            prev_node = self.node
            self.node = nxt
            self._clear_failure(prev_node, nxt)
            visit_count = self._visit_counts.get(self.node, 0) + 1
            self._visit_counts[self.node] = visit_count
            if visit_count == 1:
                reward += self.frontier_bonus
            elif visit_count > 1:
                reward -= self.revisit_penalty * float(visit_count - 1)

        reward -= self.w_exposure * exposure

        if self.node == self._target_node():
            if self._target_node() == self.goal_node:
                remaining_steps = max(0, self.max_steps - self.steps)
                reward += self.w_reached
                reward += self.goal_reward_base + self.goal_reward_time_bonus * remaining_steps
                done = True
            else:
                reward += self.w_checkpoint
                self._advance_target()
        elif self.steps >= self.max_steps:
            done = True

        if done and alive:
            reward += self.w_alive

        curr_dist = self._goal_dist()
        if np.isfinite(self._prev_goal_dist) and np.isfinite(curr_dist):
            # Reward progress toward shelter; penalize moving away.
            reward += self.w_progress * (self._prev_goal_dist - curr_dist)
        self._prev_goal_dist = curr_dist
        self._curr_candidates, self._curr_action_mask = self._action_context(self.node)
        next_obs = self._obs()

        return next_obs, reward, done, {"action_mask": self.get_action_mask()}


class TorchDRQN(nn.Module):
    def __init__(self, obs_dim, hidden_dim, num_actions):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.gru = nn.GRUCell(obs_dim, hidden_dim)
        self.q_head = nn.Linear(hidden_dim, num_actions)

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
        if not eligible:
            raise ValueError("Not enough eligible episodes for sequence sampling.")

        # If eligible episodes are fewer than batch size, sample with replacement
        # so training can continue when many episodes terminate early.
        if len(eligible) >= batch_size:
            chosen = random.sample(eligible, batch_size)
        else:
            chosen = random.choices(eligible, k=batch_size)
        obs_batch = []
        mask_batch = []
        act_batch = []
        rew_batch = []
        nxt_batch = []
        nxt_mask_batch = []
        done_batch = []
        for ep in chosen:
            start = random.randint(0, len(ep) - total_len)
            chunk = ep[start : start + total_len]
            obs, mask, act, rew, nxt, nxt_mask, done = zip(*chunk)
            obs_batch.append(np.stack(obs).astype(np.float32))
            mask_batch.append(np.stack(mask).astype(np.float32))
            act_batch.append(np.array(act, dtype=np.int64))
            rew_batch.append(np.array(rew, dtype=np.float32))
            nxt_batch.append(np.stack(nxt).astype(np.float32))
            nxt_mask_batch.append(np.stack(nxt_mask).astype(np.float32))
            done_batch.append(np.array(done, dtype=np.float32))

        return (
            np.stack(obs_batch),   # [B, T, obs]
            np.stack(mask_batch),  # [B, T, A]
            np.stack(act_batch),   # [B, T]
            np.stack(rew_batch),   # [B, T]
            np.stack(nxt_batch),   # [B, T, obs]
            np.stack(nxt_mask_batch),  # [B, T, A]
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


def _load_persona_pool():
    """Load all persona definitions from agent_profiles.json for training diversification."""
    profiles_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "agent_profiles.json")
    try:
        with open(profiles_path, "r", encoding="utf-8") as f:
            profiles = json.load(f)
        return list(profiles.values())
    except Exception:
        return []


_PERSONA_POOL = _load_persona_pool()


def train(args):
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    device = _pick_device(args.device)
    print(f"[drqn] device={device}")

    env = GridPOMDPEnv(
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
        failure_penalty_steps=args.failure_penalty_steps,
        failure_block_on_repeat=args.failure_block_on_repeat,
        allow_stay_when_move_available=args.allow_stay_when_move_available,
        use_subgoals=args.use_subgoals,
        checkpoint_interval=args.checkpoint_interval,
        w_checkpoint=args.w_checkpoint,
        dynamic_step_budget=args.dynamic_step_budget,
        step_budget_scale=args.step_budget_scale,
        step_budget_min=args.step_budget_min,
        step_budget_max=args.step_budget_max,
        step_budget_slack=args.step_budget_slack,
        frontier_bonus=args.frontier_bonus,
        revisit_penalty=args.revisit_penalty,
        replan_on_block=args.replan_on_block,
        domain_rand=args.domain_rand,
        domain_rand_block_init_max=args.domain_rand_block_init_max,
        domain_rand_snow_threshold_min=args.domain_rand_snow_threshold_min,
        domain_rand_snow_threshold_max=args.domain_rand_snow_threshold_max,
        domain_rand_snow_prob_max=args.domain_rand_snow_prob_max,
    )
    obs0 = env.reset()
    obs_dim = obs0.shape[0]
    num_actions = env.num_actions

    policy_net = TorchDRQN(obs_dim=obs_dim, hidden_dim=args.hidden_dim, num_actions=num_actions).to(device)
    target_net = TorchDRQN(obs_dim=obs_dim, hidden_dim=args.hidden_dim, num_actions=num_actions).to(device)

    if args.init_checkpoint:
        payload = torch.load(args.init_checkpoint, map_location=device)
        policy_net.load_state_dict(payload["model_state_dict"])
        print(f"[drqn] loaded init checkpoint: {args.init_checkpoint}", flush=True)

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
    reached_hist = []
    best_ma = -float("inf")
    best_ma_reached = -float("inf")
    adaptive_max_dist = float(args.curriculum_start_dist)
    with open(history_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "episode",
                "return",
                "steps",
                "reached",
                "avg_loss",
                "eps",
                "moving_avg_return",
                "moving_avg_reached_rate",
                "buffer_size",
            ]
        )

        for ep in range(1, args.episodes + 1):
            curriculum_value = -1.0
            coverage_ratio = None
            max_dist = None
            if args.curriculum_mode == "coverage":
                effective_ep = ep
                if args.curriculum_freeze_episode > 0:
                    effective_ep = min(ep, int(args.curriculum_freeze_episode))
                frac = min(1.0, float(effective_ep) / max(1.0, float(args.curriculum_full_at_episode)))
                coverage_ratio = args.curriculum_start_coverage + frac * (
                    args.curriculum_end_coverage - args.curriculum_start_coverage
                )
                curriculum_value = coverage_ratio
            elif args.curriculum_mode == "adaptive_distance":
                max_dist = adaptive_max_dist
                curriculum_value = adaptive_max_dist
            elif args.curriculum_start_dist > 0:
                effective_ep = ep
                if args.curriculum_freeze_episode > 0:
                    effective_ep = min(ep, int(args.curriculum_freeze_episode))
                frac = min(1.0, float(effective_ep) / max(1.0, float(args.curriculum_full_at_episode)))
                max_dist = args.curriculum_start_dist + frac * (args.curriculum_end_dist - args.curriculum_start_dist)
                curriculum_value = max_dist
            obs_np = env.reset(max_start_goal_dist=max_dist, start_coverage_ratio=coverage_ratio)
            # Sample random persona each episode for generalization
            if hasattr(env, "set_persona") and _PERSONA_POOL:
                p = random.choice(_PERSONA_POOL)
                env.set_persona(
                    walk_speed_multiplier=p.get("walk_speed_multiplier", 1.0),
                    panic_level=p.get("panic_level", 0.0),
                    shelter_familiarity=p.get("shelter_familiarity", 0.5),
                    compliance_rate=p.get("compliance_rate", 0.8),
                    observation_error_multiplier=p.get("observation_error_multiplier", 1.0),
                )
                obs_np = env._obs()  # rebuild obs with fresh persona
            action_mask_np = env.get_action_mask()
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
                valid_actions = np.flatnonzero(action_mask_np > 0.5)
                if len(valid_actions) == 0:
                    valid_actions = np.array([ACTION_STAY], dtype=np.int64)
                if random.random() < eps:
                    action = int(random.choice(valid_actions.tolist()))
                else:
                    q_np = q_t.detach().cpu().numpy().squeeze(0)
                    invalid = action_mask_np <= 0.5
                    q_np[invalid] = -1e9
                    action = int(np.argmax(q_np))

                next_obs_np, reward, done, info = env.step(action)
                next_action_mask_np = info.get("action_mask", env.get_action_mask())
                if done and env.node == env.goal_node:
                    reached = 1

                episode_transitions.append(
                    (obs_np, action_mask_np, action, reward, next_obs_np, next_action_mask_np, float(done))
                )

                if replay.total_transitions >= args.warmup_steps:
                    for _ in range(args.updates_per_step):
                        try:
                            obs_seq, _mask_seq, act_seq, rew_seq, nxt_seq, nxt_mask_seq, done_seq = replay.sample_sequences(
                                batch_size=args.batch_size,
                                burn_in=args.burn_in,
                                seq_len=args.seq_len,
                            )
                        except ValueError:
                            continue
                        obs_t = torch.tensor(obs_seq, dtype=torch.float32, device=device)   # [B,T,O]
                        act_t = torch.tensor(act_seq, dtype=torch.int64, device=device)      # [B,T]
                        rew_t = torch.tensor(rew_seq, dtype=torch.float32, device=device)    # [B,T]
                        nxt_t = torch.tensor(nxt_seq, dtype=torch.float32, device=device)    # [B,T,O]
                        nxt_mask_t = torch.tensor(nxt_mask_seq, dtype=torch.float32, device=device)  # [B,T,A]
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
                                q_next_online_masked = q_next_online.masked_fill(nxt_mask_t[:, t_idx, :] < 0.5, -1e9)
                                next_action = q_next_online_masked.argmax(dim=1, keepdim=True)
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
                action_mask_np = next_action_mask_np
                h = h_next.detach()

            replay.add_episode(episode_transitions)

            avg_loss = float(np.mean(losses)) if losses else 0.0
            returns.append(ep_return)
            reached_hist.append(reached)
            window = max(1, int(args.best_window))
            moving_avg = float(np.mean(returns[-window:]))
            moving_avg_reached = float(np.mean(reached_hist[-window:]))

            if args.curriculum_mode == "adaptive_distance":
                adaptive_window = max(1, int(args.curriculum_adaptive_window))
                if len(reached_hist) >= adaptive_window:
                    recent_reached = float(np.mean(reached_hist[-adaptive_window:]))
                    if recent_reached >= float(args.curriculum_advance_threshold):
                        adaptive_max_dist = min(
                            float(args.curriculum_end_dist),
                            adaptive_max_dist + float(args.curriculum_adaptive_step),
                        )
                    elif recent_reached <= float(args.curriculum_regress_threshold):
                        adaptive_max_dist = max(
                            float(args.curriculum_start_dist),
                            adaptive_max_dist - 0.5 * float(args.curriculum_adaptive_step),
                        )
                    curriculum_value = adaptive_max_dist

            w.writerow(
                [
                    ep,
                    f"{ep_return:.4f}",
                    step,
                    reached,
                    f"{avg_loss:.6f}",
                    f"{eps:.4f}",
                    f"{moving_avg:.4f}",
                    f"{moving_avg_reached:.4f}",
                    replay.total_transitions,
                ]
            )

            better_ckpt = (
                ep >= int(args.best_min_episode)
                and (
                    moving_avg_reached > best_ma_reached
                    or (
                        abs(moving_avg_reached - best_ma_reached) <= 1e-9
                        and moving_avg > best_ma
                    )
                )
            )
            if better_ckpt:
                best_ma = moving_avg
                best_ma_reached = moving_avg_reached
                torch.save(
                    {
                        "model_state_dict": policy_net.state_dict(),
                        "obs_dim": obs_dim,
                        "hidden_dim": args.hidden_dim,
                        "num_actions": num_actions,
                        "max_neighbors": args.max_neighbors,
                        "max_steps": args.max_steps,
                        "episode": ep,
                        "moving_avg_return": moving_avg,
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
                        "failure_penalty_steps": args.failure_penalty_steps,
                        "failure_block_on_repeat": args.failure_block_on_repeat,
                        "allow_stay_when_move_available": args.allow_stay_when_move_available,
                        "use_subgoals": args.use_subgoals,
                        "checkpoint_interval": args.checkpoint_interval,
                        "w_checkpoint": args.w_checkpoint,
                        "dynamic_step_budget": args.dynamic_step_budget,
                        "step_budget_scale": args.step_budget_scale,
                        "step_budget_min": args.step_budget_min,
                        "step_budget_max": args.step_budget_max,
                        "step_budget_slack": args.step_budget_slack,
                        "frontier_bonus": args.frontier_bonus,
                        "revisit_penalty": args.revisit_penalty,
                        "replan_on_block": args.replan_on_block,
                        "domain_rand": args.domain_rand,
                        "domain_rand_block_init_max": args.domain_rand_block_init_max,
                        "domain_rand_snow_threshold_min": args.domain_rand_snow_threshold_min,
                        "domain_rand_snow_threshold_max": args.domain_rand_snow_threshold_max,
                        "domain_rand_snow_prob_max": args.domain_rand_snow_prob_max,
                        "moving_avg_reached_rate": moving_avg_reached,
                        "best_min_episode": int(args.best_min_episode),
                        "curriculum_mode": args.curriculum_mode,
                        "curriculum_start_dist": args.curriculum_start_dist,
                        "curriculum_end_dist": args.curriculum_end_dist,
                        "curriculum_adaptive_window": int(args.curriculum_adaptive_window),
                        "curriculum_advance_threshold": args.curriculum_advance_threshold,
                        "curriculum_regress_threshold": args.curriculum_regress_threshold,
                        "curriculum_adaptive_step": args.curriculum_adaptive_step,
                    },
                    best_ckpt_path,
                )

            if ep % args.log_every == 0 or ep == 1:
                print(
                    f"[drqn] ep={ep:4d} return={ep_return:8.3f} steps={step:3d} "
                    f"reached={reached} avg_loss={avg_loss:.6f} eps={eps:.3f} "
                    f"ma_return={moving_avg:8.3f} ma_reached={moving_avg_reached:.3f} "
                    f"best_ma={best_ma:8.3f} best_ma_reached={best_ma_reached:.3f} "
                    f"trans={replay.total_transitions:6d} eps_buf={len(replay):4d} "
                    f"raw_step_budget={env.raw_step_budget:6.1f} max_steps={env.max_steps:3d} "
                    f"curriculum_{args.curriculum_mode}={curriculum_value:.3f}",
                    flush=True,
                )

            eps = max(args.eps_end, eps * args.eps_decay)

    torch.save(
        {
            "model_state_dict": policy_net.state_dict(),
            "obs_dim": obs_dim,
            "hidden_dim": args.hidden_dim,
            "num_actions": num_actions,
            "max_neighbors": args.max_neighbors,
            "max_steps": args.max_steps,
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
            "failure_penalty_steps": args.failure_penalty_steps,
            "failure_block_on_repeat": args.failure_block_on_repeat,
            "allow_stay_when_move_available": args.allow_stay_when_move_available,
            "use_subgoals": args.use_subgoals,
            "checkpoint_interval": args.checkpoint_interval,
            "w_checkpoint": args.w_checkpoint,
            "dynamic_step_budget": args.dynamic_step_budget,
            "step_budget_scale": args.step_budget_scale,
            "step_budget_min": args.step_budget_min,
            "step_budget_max": args.step_budget_max,
            "step_budget_slack": args.step_budget_slack,
            "frontier_bonus": args.frontier_bonus,
            "revisit_penalty": args.revisit_penalty,
            "replan_on_block": args.replan_on_block,
            "domain_rand": args.domain_rand,
            "domain_rand_block_init_max": args.domain_rand_block_init_max,
            "domain_rand_snow_threshold_min": args.domain_rand_snow_threshold_min,
            "domain_rand_snow_threshold_max": args.domain_rand_snow_threshold_max,
            "domain_rand_snow_prob_max": args.domain_rand_snow_prob_max,
            "curriculum_mode": args.curriculum_mode,
            "curriculum_start_dist": args.curriculum_start_dist,
            "curriculum_end_dist": args.curriculum_end_dist,
            "curriculum_adaptive_window": int(args.curriculum_adaptive_window),
            "curriculum_advance_threshold": args.curriculum_advance_threshold,
            "curriculum_regress_threshold": args.curriculum_regress_threshold,
            "curriculum_adaptive_step": args.curriculum_adaptive_step,
        },
        ckpt_path,
    )
    if not os.path.exists(best_ckpt_path):
        torch.save(
            {
                "model_state_dict": policy_net.state_dict(),
                "obs_dim": obs_dim,
                "hidden_dim": args.hidden_dim,
                "num_actions": num_actions,
                "max_neighbors": args.max_neighbors,
                "max_steps": args.max_steps,
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
                "failure_penalty_steps": args.failure_penalty_steps,
                "failure_block_on_repeat": args.failure_block_on_repeat,
                "allow_stay_when_move_available": args.allow_stay_when_move_available,
                "use_subgoals": args.use_subgoals,
                "checkpoint_interval": args.checkpoint_interval,
                "w_checkpoint": args.w_checkpoint,
                "dynamic_step_budget": args.dynamic_step_budget,
                "step_budget_scale": args.step_budget_scale,
                "step_budget_min": args.step_budget_min,
                "step_budget_max": args.step_budget_max,
                "step_budget_slack": args.step_budget_slack,
                "frontier_bonus": args.frontier_bonus,
                "revisit_penalty": args.revisit_penalty,
                "replan_on_block": args.replan_on_block,
                "domain_rand": args.domain_rand,
                "domain_rand_block_init_max": args.domain_rand_block_init_max,
                "domain_rand_snow_threshold_min": args.domain_rand_snow_threshold_min,
                "domain_rand_snow_threshold_max": args.domain_rand_snow_threshold_max,
                "domain_rand_snow_prob_max": args.domain_rand_snow_prob_max,
                "moving_avg_reached_rate": float(np.mean(reached_hist[-max(1, int(args.best_window)) :])) if reached_hist else 0.0,
                "best_min_episode": int(args.best_min_episode),
                "curriculum_mode": args.curriculum_mode,
                "curriculum_start_dist": args.curriculum_start_dist,
                "curriculum_end_dist": args.curriculum_end_dist,
                "curriculum_adaptive_window": int(args.curriculum_adaptive_window),
                "curriculum_advance_threshold": args.curriculum_advance_threshold,
                "curriculum_regress_threshold": args.curriculum_regress_threshold,
                "curriculum_adaptive_step": args.curriculum_adaptive_step,
            },
            best_ckpt_path,
        )
    print(f"[drqn] history: {history_path}")
    print(f"[drqn] weights: {ckpt_path}")
    print(f"[drqn] best weights: {best_ckpt_path} (best moving_avg_return={best_ma:.3f})")


def main():
    parser = argparse.ArgumentParser(description="Minimal runnable DRQN baseline (PyTorch)")
    parser.add_argument("--preset", default="none", choices=["none", "stable_v1", "stable_v2", "stable_train", "stable_demo", "stable_train_easymax"])
    parser.add_argument("--episodes", type=int, default=200)
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
    parser.add_argument("--failure-penalty-steps", type=int, default=25)
    parser.add_argument("--failure-block-on-repeat", type=int, default=2)
    parser.add_argument("--allow-stay-when-move-available", type=lambda x: str(x).lower() in {"1", "true", "yes"}, default=True)
    parser.add_argument("--use-subgoals", type=lambda x: str(x).lower() in {"1", "true", "yes"}, default=False)
    parser.add_argument("--checkpoint-interval", type=int, default=5)
    parser.add_argument("--w-checkpoint", type=float, default=30.0)
    parser.add_argument("--dynamic-step-budget", type=lambda x: str(x).lower() in {"1", "true", "yes"}, default=False)
    parser.add_argument("--step-budget-scale", type=float, default=0.08)
    parser.add_argument("--step-budget-min", type=int, default=100)
    parser.add_argument("--step-budget-max", type=int, default=400)
    parser.add_argument("--step-budget-slack", type=int, default=20)
    parser.add_argument("--frontier-bonus", type=float, default=0.0)
    parser.add_argument("--revisit-penalty", type=float, default=0.0)
    parser.add_argument("--replan-on-block", type=lambda x: str(x).lower() in {"1", "true", "yes"}, default=True)
    parser.add_argument("--domain-rand", type=lambda x: str(x).lower() in {"1", "true", "yes"}, default=False,
                        help="Randomize hazard params (block_init_prob, snow threshold, snow prob) each episode")
    parser.add_argument("--domain-rand-block-init-max", type=float, default=0.12,
                        help="Upper bound for block_init_prob sampling when domain_rand=True")
    parser.add_argument("--domain-rand-snow-threshold-min", type=float, default=0.60,
                        help="Lower bound for block_from_snow_threshold sampling when domain_rand=True")
    parser.add_argument("--domain-rand-snow-threshold-max", type=float, default=0.92,
                        help="Upper bound for block_from_snow_threshold sampling when domain_rand=True")
    parser.add_argument("--domain-rand-snow-prob-max", type=float, default=0.007,
                        help="Upper bound for block_from_snow_prob sampling when domain_rand=True")
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
    parser.add_argument("--best-min-episode", type=int, default=50)
    parser.add_argument("--curriculum-mode", default="distance", choices=["distance", "coverage", "adaptive_distance"])
    parser.add_argument("--curriculum-start-dist", type=float, default=2000.0)
    parser.add_argument("--curriculum-end-dist", type=float, default=12000.0)
    parser.add_argument("--curriculum-start-coverage", type=float, default=0.2)
    parser.add_argument("--curriculum-end-coverage", type=float, default=1.0)
    parser.add_argument("--curriculum-full-at-episode", type=int, default=300)
    parser.add_argument("--curriculum-freeze-episode", type=int, default=-1)
    parser.add_argument("--curriculum-adaptive-window", type=int, default=20)
    parser.add_argument("--curriculum-advance-threshold", type=float, default=0.80)
    parser.add_argument("--curriculum-regress-threshold", type=float, default=0.35)
    parser.add_argument("--curriculum-adaptive-step", type=float, default=150.0)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output-dir", default="logs")
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda", "mps"])
    parser.add_argument("--init-checkpoint", default=None)
    args = parser.parse_args()

    def apply_preset(overrides):
        for key, value in overrides.items():
            if getattr(args, key) == parser.get_default(key):
                setattr(args, key, value)

    if args.preset == "stable_v1":
        # Stability-oriented preset: easier curriculum + slower epsilon decay.
        apply_preset(
            {
                "episodes": 600,
                "max_steps": 120,
                "max_neighbors": 8,
                "lr": 3e-4,
                "eps_start": 1.0,
                "eps_end": 0.05,
                "eps_decay": 0.998,
                "replay_episodes": 1200,
                "warmup_steps": 500,
                "target_update": 500,
                "batch_size": 64,
                "seq_len": 8,
                "burn_in": 4,
                "curriculum_start_dist": 300.0,
                "curriculum_end_dist": 2500.0,
                "curriculum_mode": "distance",
                "curriculum_start_coverage": 0.2,
                "curriculum_end_coverage": 1.0,
                "curriculum_full_at_episode": 400,
                "curriculum_freeze_episode": -1,
                "w_reached": 120.0,
                "w_alive": 0.0,
                "w_exposure": 2.5,
                "w_time": 1.0,
                "w_progress": 0.02,
                "goal_reward_base": 0.0,
                "goal_reward_time_bonus": 0.0,
                "best_min_episode": 50,
                "snow_dynamic": True,
                "snow_start_zero": True,
                "block_init_prob": 0.0,
                "block_from_snow_threshold": 0.72,
                "block_from_snow_prob": 0.003,
                "failure_penalty_steps": 25,
                "failure_block_on_repeat": 2,
                "allow_stay_when_move_available": True,
                "use_subgoals": False,
                "checkpoint_interval": 5,
                "w_checkpoint": 30.0,
                "dynamic_step_budget": False,
                "step_budget_scale": 0.08,
                "step_budget_min": 100,
                "step_budget_max": 400,
                "step_budget_slack": 20,
                "frontier_bonus": 0.0,
                "revisit_penalty": 0.0,
                "replan_on_block": True,
            }
        )
    elif args.preset == "stable_v2":
        # Easier and smoother learning preset for OSM DRQN.
        apply_preset(
            {
                "episodes": 800,
                "max_steps": 100,
                "max_neighbors": 8,
                "lr": 3e-4,
                "eps_start": 1.0,
                "eps_end": 0.05,
                "eps_decay": 0.999,
                "replay_episodes": 1500,
                "warmup_steps": 500,
                "target_update": 500,
                "batch_size": 64,
                "seq_len": 8,
                "burn_in": 4,
                "curriculum_mode": "coverage",
                "curriculum_start_dist": 150.0,
                "curriculum_end_dist": 600.0,
                "curriculum_start_coverage": 0.2,
                "curriculum_end_coverage": 1.0,
                "curriculum_full_at_episode": 700,
                "curriculum_freeze_episode": -1,
                "w_reached": 220.0,
                "w_alive": 0.0,
                "w_exposure": 1.0,
                "w_time": 0.8,
                "w_progress": 0.05,
                "goal_reward_base": 0.0,
                "goal_reward_time_bonus": 0.0,
                "best_min_episode": 50,
                "snow_dynamic": True,
                "snow_start_zero": True,
                "block_init_prob": 0.0,
                "block_from_snow_threshold": 0.85,
                "block_from_snow_prob": 0.0005,
                "failure_penalty_steps": 25,
                "failure_block_on_repeat": 2,
                "allow_stay_when_move_available": True,
                "use_subgoals": True,
                "checkpoint_interval": 4,
                "w_checkpoint": 25.0,
                "dynamic_step_budget": True,
                "step_budget_scale": 0.08,
                "step_budget_min": 100,
                "step_budget_max": 1000,
                "step_budget_slack": 20,
                "frontier_bonus": 0.5,
                "revisit_penalty": 0.2,
                "replan_on_block": True,
            }
        )
    elif args.preset == "stable_train":
        apply_preset(
            {
                "episodes": 800,
                "max_steps": 100,
                "max_neighbors": 8,
                "lr": 3e-4,
                "eps_start": 1.0,
                "eps_end": 0.05,
                "eps_decay": 0.999,
                "replay_episodes": 1500,
                "warmup_steps": 500,
                "target_update": 500,
                "batch_size": 64,
                "seq_len": 8,
                "burn_in": 4,
                "curriculum_mode": "coverage",
                "curriculum_start_dist": 150.0,
                "curriculum_end_dist": 600.0,
                "curriculum_start_coverage": 0.2,
                "curriculum_end_coverage": 1.0,
                "curriculum_full_at_episode": 700,
                "curriculum_freeze_episode": -1,
                "w_reached": 220.0,
                "w_alive": 0.0,
                "w_exposure": 1.0,
                "w_time": 0.8,
                "w_progress": 0.05,
                "goal_reward_base": 0.0,
                "goal_reward_time_bonus": 0.0,
                "best_min_episode": 50,
                "snow_dynamic": True,
                "snow_start_zero": True,
                "block_init_prob": 0.0,
                "block_from_snow_threshold": 0.85,
                "block_from_snow_prob": 0.0005,
                "failure_penalty_steps": 25,
                "failure_block_on_repeat": 2,
                "allow_stay_when_move_available": True,
                "use_subgoals": True,
                "checkpoint_interval": 4,
                "w_checkpoint": 25.0,
                "dynamic_step_budget": True,
                "step_budget_scale": 0.08,
                "step_budget_min": 100,
                "step_budget_max": 1000,
                "step_budget_slack": 20,
                "frontier_bonus": 0.5,
                "revisit_penalty": 0.2,
                "replan_on_block": True,
            }
        )
    elif args.preset == "stable_demo":
        apply_preset(
            {
                "episodes": 800,
                "max_steps": 100,
                "max_neighbors": 8,
                "lr": 3e-4,
                "eps_start": 1.0,
                "eps_end": 0.05,
                "eps_decay": 0.999,
                "replay_episodes": 1500,
                "warmup_steps": 500,
                "target_update": 500,
                "batch_size": 64,
                "seq_len": 8,
                "burn_in": 4,
                "curriculum_mode": "coverage",
                "curriculum_start_dist": 150.0,
                "curriculum_end_dist": 600.0,
                "curriculum_start_coverage": 0.2,
                "curriculum_end_coverage": 1.0,
                "curriculum_full_at_episode": 700,
                "curriculum_freeze_episode": -1,
                "w_reached": 220.0,
                "w_alive": 0.0,
                "w_exposure": 1.0,
                "w_time": 0.8,
                "w_progress": 0.05,
                "goal_reward_base": 0.0,
                "goal_reward_time_bonus": 0.0,
                "best_min_episode": 50,
                "snow_dynamic": True,
                "snow_start_zero": True,
                "block_init_prob": 0.0,
                "block_from_snow_threshold": 0.85,
                "block_from_snow_prob": 0.0005,
                "failure_penalty_steps": 25,
                "failure_block_on_repeat": 2,
                "allow_stay_when_move_available": False,
                "use_subgoals": True,
                "checkpoint_interval": 4,
                "w_checkpoint": 25.0,
                "dynamic_step_budget": True,
                "step_budget_scale": 0.08,
                "step_budget_min": 100,
                "step_budget_max": 1000,
                "step_budget_slack": 20,
                "frontier_bonus": 0.5,
                "revisit_penalty": 0.2,
                "replan_on_block": True,
            }
        )
    elif args.preset == "stable_train_easymax":
        apply_preset(
            {
                "episodes": 1000,
                "max_steps": 100,
                "max_neighbors": 8,
                "lr": 3e-4,
                "eps_start": 1.0,
                "eps_end": 0.05,
                "eps_decay": 0.999,
                "replay_episodes": 1500,
                "warmup_steps": 500,
                "target_update": 500,
                "batch_size": 64,
                "seq_len": 8,
                "burn_in": 4,
                "curriculum_mode": "coverage",
                "curriculum_start_dist": 150.0,
                "curriculum_end_dist": 600.0,
                "curriculum_start_coverage": 0.2,
                "curriculum_end_coverage": 1.0,
                "curriculum_full_at_episode": 1000,
                "curriculum_freeze_episode": -1,
                "w_reached": 220.0,
                "w_alive": 0.0,
                "w_exposure": 0.2,
                "w_time": 0.8,
                "w_progress": 0.10,
                "goal_reward_base": 0.0,
                "goal_reward_time_bonus": 0.0,
                "best_min_episode": 100,
                "snow_dynamic": True,
                "snow_start_zero": True,
                "block_init_prob": 0.0,
                "block_from_snow_threshold": 0.85,
                "block_from_snow_prob": 0.0005,
                "failure_penalty_steps": 25,
                "failure_block_on_repeat": 2,
                "allow_stay_when_move_available": True,
                "use_subgoals": True,
                "checkpoint_interval": 4,
                "w_checkpoint": 40.0,
                "dynamic_step_budget": True,
                "step_budget_scale": 0.08,
                "step_budget_min": 100,
                "step_budget_max": 1000,
                "step_budget_slack": 20,
                "frontier_bonus": 0.0,
                "revisit_penalty": 0.0,
                "replan_on_block": True,
            }
        )

    train(args)


if __name__ == "__main__":
    main()
