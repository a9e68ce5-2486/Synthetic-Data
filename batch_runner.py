import csv
import json
import os
import random
from collections import defaultdict
import math

# LLM behavior profiles — loaded once at module level if file exists
_AGENT_PROFILES = {}
_AGENT_PROFILES_PATH = os.path.join(os.path.dirname(__file__), "agent_profiles.json")
if os.path.exists(_AGENT_PROFILES_PATH):
    with open(_AGENT_PROFILES_PATH, "r", encoding="utf-8") as _f:
        _AGENT_PROFILES = json.load(_f)

# Persona assignment weights: maps role → list of (persona, weight)
# Reflects University of Utah population composition across 4 role categories
_PERSONA_WEIGHTS = {
    "student": [
        ("young_student",         0.30),
        ("freshman_student",      0.20),
        ("graduate_student",      0.20),
        ("international_student", 0.10),
        ("student_athlete",       0.07),
        ("student_with_anxiety",  0.08),
        ("part_time_student",     0.05),
    ],
    "faculty": [
        ("senior_faculty",   0.45),
        ("junior_faculty",   0.35),
        ("adjunct_instructor", 0.20),
    ],
    "staff": [
        ("staff_admin",        0.30),
        ("facilities_staff",   0.20),
        ("campus_security",    0.10),
        ("healthcare_staff",   0.15),
        ("research_scientist", 0.15),
        ("it_staff",           0.10),
    ],
    "visitor": [
        ("visitor",                        0.35),
        ("mobility_impaired",              0.15),
        ("conference_attendee",            0.30),
        ("prospective_student_with_parent", 0.20),
    ],
}

def _assign_persona(role: str) -> str:
    """Pick a persona for an agent based on their role, using weighted sampling."""
    if not _AGENT_PROFILES:
        return None
    weights_list = _PERSONA_WEIGHTS.get(role, [("staff_admin", 1.0)])
    personas = [p for p, _ in weights_list]
    weights  = [w for _, w in weights_list]
    return random.choices(personas, weights=weights, k=1)[0]


def _agent_familiar_goals(agent, all_goals):
    """Return the subset of goal nodes this agent knows about, based on shelter_familiarity.

    familiarity=1.0 → knows all shelters (optimal)
    familiarity=0.1 → knows only 1 shelter (random, may not be nearest)
    The subset is fixed per agent (seeded by agent id + persona) for reproducibility.
    """
    familiarity = getattr(agent, "shelter_familiarity", 1.0)
    if familiarity >= 1.0 or len(all_goals) <= 1:
        return list(all_goals)
    n_known = max(1, round(familiarity * len(all_goals)))
    rng = random.Random(agent.id ^ hash(getattr(agent, "persona", "") or ""))
    shuffled = list(all_goals)
    rng.shuffle(shuffled)
    return shuffled[:n_known]


def _apply_compliance(agent, policy_goal, familiar_goals):
    """With probability (1 - compliance_rate), override policy goal with a random familiar shelter."""
    compliance = getattr(agent, "compliance_rate", 1.0)
    if compliance >= 1.0 or not familiar_goals:
        return policy_goal
    if random.random() > compliance:
        return random.choice(familiar_goals)
    return policy_goal

def _apply_persona(agent, persona_name: str):
    """Apply LLM-generated behavior profile to an agent.

    panic_level (0–1) is baked into two fields at creation time so the
    rest of the simulation reads them naturally:
      - observation_error_multiplier *= (1 + panic_level)
        → panicky agents misread their environment more
      - compliance_rate *= (1 - 0.5 * panic_level)
        → panicky agents are less likely to follow shelter recommendations
    """
    if not persona_name or persona_name not in _AGENT_PROFILES:
        return
    p = _AGENT_PROFILES[persona_name]
    panic = float(p.get("panic_level", 0.0))
    agent.persona                    = persona_name
    agent.speed_multiplier           = float(p.get("walk_speed_multiplier", 1.0))
    agent.panic_level                = panic
    agent.decision_delay_steps       = int(p.get("decision_delay_steps", 0))
    agent.shelter_familiarity        = float(p.get("shelter_familiarity", 1.0))
    # Effective values after panic modulation
    agent.compliance_rate            = float(p.get("compliance_rate", 1.0)) * (1.0 - 0.5 * panic)
    agent.observation_error_multiplier = float(p.get("observation_error_multiplier", 1.0)) * (1.0 + panic)

import networkx as nx
import numpy as np

import config
from evac_env import EvacEnv
from agents.ped_agent import PedAgent
from agents.car_agent import CarAgent
from agents.shuttle_agent import ShuttleAgent, build_shuttle_route
from kpi import summarize_run, aggregate_policy_rows
from management_report import build_management_summary
from policy import SUPPORTED_POLICIES, select_goal
from scenario_loader import temporary_config


def _pick_torch_device(device_arg):
    import torch

    if device_arg != "auto":
        return torch.device(device_arg)
    if torch.cuda.is_available():
        return torch.device("cuda")
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


class _DRQNPedController:
    def __init__(self, env, checkpoint_path, device="auto", max_neighbors=None, max_steps=80):
        try:
            import torch
            from drqn_minimal import TorchDRQN
        except Exception as e:
            raise RuntimeError(f"DRQN policy requires PyTorch and drqn_minimal imports. {e}") from e

        if not checkpoint_path:
            raise ValueError("DRQN policy requires a checkpoint path (--drqn-checkpoint).")
        if not os.path.exists(checkpoint_path):
            raise FileNotFoundError(f"DRQN checkpoint not found: {checkpoint_path}")

        self.env = env
        self.max_steps = int(max_steps)
        self.device = _pick_torch_device(device)
        payload = torch.load(checkpoint_path, map_location=self.device)
        obs_dim = int(payload["obs_dim"])
        hidden_dim = int(payload["hidden_dim"])
        num_actions = int(payload.get("num_actions", 9))
        self.max_neighbors = int(max_neighbors) if max_neighbors is not None else int(payload.get("max_neighbors", max(1, num_actions - 1)))
        self.num_actions = 1 + self.max_neighbors
        self.use_subgoals = bool(payload.get("use_subgoals", False))
        self.checkpoint_interval = int(payload.get("checkpoint_interval", 5))
        self.base_max_steps = int(payload.get("max_steps", max_steps))
        self.dynamic_step_budget = bool(payload.get("dynamic_step_budget", False))
        self.step_budget_scale = float(payload.get("step_budget_scale", 0.08))
        self.step_budget_min = int(payload.get("step_budget_min", self.base_max_steps))
        self.step_budget_max = int(payload.get("step_budget_max", self.base_max_steps))
        self.step_budget_slack = float(payload.get("step_budget_slack", 20.0))
        self.frontier_bonus = float(payload.get("frontier_bonus", 0.0))
        self.revisit_penalty = float(payload.get("revisit_penalty", 0.0))
        self.replan_on_block = bool(payload.get("replan_on_block", True))
        self.interaction_density_enabled = bool(getattr(config, "EVAC_INTERACTION_DENSITY_ENABLED", False))
        self.interaction_edge_penalty = float(getattr(config, "EVAC_INTERACTION_EDGE_PENALTY", 0.30))
        self.interaction_node_penalty = float(getattr(config, "EVAC_INTERACTION_NODE_PENALTY", 0.35))
        self.interaction_nearby_penalty = float(getattr(config, "EVAC_INTERACTION_NEARBY_PENALTY", 0.15))
        self.interaction_radius_m = float(getattr(config, "EVAC_INTERACTION_RADIUS_M", 80.0))

        self.model = TorchDRQN(obs_dim=obs_dim, hidden_dim=hidden_dim, num_actions=self.num_actions).to(self.device)
        self.model.load_state_dict(payload["model_state_dict"])
        self.model.eval()

        self._torch = torch
        self.hidden = {}
        self.obs_dim = 12 + 5 * self.max_neighbors  # 7 base + 5 persona + 5*k neighbors
        self.agent_checkpoints = {}
        self.agent_checkpoint_idx = {}
        self.agent_current_target = {}
        self.agent_step_budget = {}
        self.agent_committed_next = {}
        self.agent_visit_counts = {}
        self._node_agent_counts = defaultdict(int)
        self._agent_xy = []
        self._edge_agent_counts = defaultdict(int)
        self._step_selected_next_counts = defaultdict(int)

        xs = [self.env.pos[n][0] for n in self.env.pos]
        ys = [self.env.pos[n][1] for n in self.env.pos]
        span_x = (max(xs) - min(xs)) if xs else 1.0
        span_y = (max(ys) - min(ys)) if ys else 1.0
        self.coord_scale = max(1.0, span_x, span_y)

    def reset_agents(self, agents):
        self.hidden = {
            a.id: self.model.init_hidden(device=self.device)
            for a in agents
        }
        self.agent_checkpoints = {a.id: [] for a in agents}
        self.agent_checkpoint_idx = {a.id: 0 for a in agents}
        self.agent_current_target = {a.id: None for a in agents}
        self.agent_step_budget = {a.id: self.base_max_steps for a in agents}
        self.agent_committed_next = {a.id: None for a in agents}
        self.agent_visit_counts = {a.id: {} for a in agents}
        self._node_agent_counts = defaultdict(int)
        self._agent_xy = []
        self._edge_agent_counts = defaultdict(int)
        self._step_selected_next_counts = defaultdict(int)

    def update_interaction_state(self, peds, cars):
        self._node_agent_counts = defaultdict(int)
        self._agent_xy = []
        self._edge_agent_counts = defaultdict(int)
        for a in list(peds) + list(cars):
            if not getattr(a, "alive", True) or getattr(a, "reached", False):
                continue
            node = getattr(a, "node", None)
            if node is None:
                continue
            self._node_agent_counts[node] += 1
            if node in self.env.pos:
                self._agent_xy.append(self.env.pos[node])
            edge_u = getattr(a, "edge_u", None)
            edge_v = getattr(a, "edge_v", None)
            edge_progress = float(getattr(a, "edge_progress", 0.0))
            if edge_u is not None and edge_v is not None and edge_progress > 0.0:
                self._edge_agent_counts[(edge_u, edge_v)] += 1

    def reset_decision_step(self):
        self._step_selected_next_counts = defaultdict(int)

    def _candidate_density_penalty(self, u, nxt):
        if not self.interaction_density_enabled or nxt is None:
            return 0.0
        edge_count = float(self._edge_agent_counts.get((u, nxt), 0) + self._step_selected_next_counts.get((u, nxt), 0))
        edge_penalty = self.interaction_edge_penalty * min(1.0, edge_count / 3.0)

        node_count = float(self._node_agent_counts.get(nxt, 0))
        node_penalty = self.interaction_node_penalty * min(1.0, node_count / 5.0)

        nearby_count = 0.0
        if u in self.env.pos and nxt in self.env.pos and self._agent_xy:
            x1, y1 = self.env.pos[u]
            x2, y2 = self.env.pos[nxt]
            x0 = 0.5 * (x1 + x2)
            y0 = 0.5 * (y1 + y2)
            for x1, y1 in self._agent_xy:
                if math.hypot(x1 - x0, y1 - y0) <= self.interaction_radius_m:
                    nearby_count += 1.0
        nearby_penalty = self.interaction_nearby_penalty * min(1.0, nearby_count / 10.0)
        return edge_penalty + node_penalty + nearby_penalty

    def init_goal(self, agent_id, start, goal):
        checkpoints = self._build_checkpoints(start, goal)
        self.agent_checkpoints[agent_id] = checkpoints
        self.agent_checkpoint_idx[agent_id] = 0
        self.agent_current_target[agent_id] = checkpoints[0] if checkpoints else goal
        self.agent_committed_next[agent_id] = None
        self.agent_visit_counts[agent_id] = {start: 1}
        try:
            start_goal_dist = float(nx.shortest_path_length(self._walk_graph_view(), start, goal, weight="weight"))
        except Exception:
            start_goal_dist = 0.0
        if self.dynamic_step_budget:
            raw_budget = self.step_budget_scale * start_goal_dist + self.step_budget_slack
            budget = int(raw_budget)
            budget = max(self.step_budget_min, min(self.step_budget_max, budget))
        else:
            budget = self.base_max_steps
        self.agent_step_budget[agent_id] = budget

    def current_target(self, agent_id, final_goal):
        return self.agent_current_target.get(agent_id) or final_goal

    def _walk_graph_view(self):
        blocked = set(getattr(self.env, "blocked_edges_walk", set()))
        if not blocked:
            return self.env.G_walk
        keep_edges = [(u, v) for u, v in self.env.G_walk.edges() if (u, v) not in blocked]
        return self.env.G_walk.edge_subgraph(keep_edges).copy()

    def _build_checkpoints(self, start, goal):
        checkpoints = [goal]
        if self.use_subgoals and self.checkpoint_interval > 0:
            try:
                path = nx.shortest_path(self._walk_graph_view(), start, goal, weight="weight")
                checkpoints = []
                for idx in range(self.checkpoint_interval, len(path) - 1, self.checkpoint_interval):
                    checkpoints.append(path[idx])
                if not checkpoints or checkpoints[-1] != goal:
                    checkpoints.append(goal)
            except Exception:
                checkpoints = [goal]
        return checkpoints

    def refresh_targets(self, agent_id, current_node, final_goal):
        checkpoints = self._build_checkpoints(current_node, final_goal)
        self.agent_checkpoints[agent_id] = checkpoints
        self.agent_checkpoint_idx[agent_id] = 0
        self.agent_current_target[agent_id] = checkpoints[0] if checkpoints else final_goal

    def advance_target(self, agent_id, final_goal):
        checkpoints = self.agent_checkpoints.get(agent_id) or [final_goal]
        idx = self.agent_checkpoint_idx.get(agent_id, 0)
        if idx + 1 < len(checkpoints):
            idx += 1
            self.agent_checkpoint_idx[agent_id] = idx
            self.agent_current_target[agent_id] = checkpoints[idx]
            return False
        self.agent_current_target[agent_id] = final_goal
        return True

    def _candidate_neighbors(self, agent_id, node, goal):
        if node not in self.env.G_walk:
            return []
        succ = list(self.env.G_walk.successors(node))
        if not succ:
            return []
        walk_graph = self._walk_graph_view()
        gx, gy = self.env.pos.get(goal, (0.0, 0.0))
        x0, y0 = self.env.pos.get(node, (0.0, 0.0))
        d0 = float(np.hypot(gx - x0, gy - y0))
        try:
            base_goal_dist = float(nx.shortest_path_length(walk_graph, node, goal, weight="weight"))
        except Exception:
            base_goal_dist = float("inf")
        ranked = []
        for nxt in succ:
            if self.env.is_blocked(node, nxt, mode="walk", belief=None):
                continue
            x1, y1 = self.env.pos.get(nxt, (x0, y0))
            d1 = float(np.hypot(gx - x1, gy - y1))
            geo_progress = d0 - d1
            try:
                nxt_goal_dist = float(nx.shortest_path_length(walk_graph, nxt, goal, weight="weight"))
            except Exception:
                nxt_goal_dist = float("inf")
            if np.isfinite(base_goal_dist) and np.isfinite(nxt_goal_dist):
                graph_progress = base_goal_dist - nxt_goal_dist
            else:
                graph_progress = -float("inf")
            edge_w = float(self.env.G_walk[node][nxt].get("weight", 1.0))
            visit_count = self.agent_visit_counts.get(agent_id, {}).get(nxt, 0)
            ranked.append((graph_progress, geo_progress, -visit_count, -edge_w, nxt))
        ranked.sort(reverse=True)
        return [n for *_rest, n in ranked[: self.max_neighbors]]

    def _obs(self, agent_id, node, target, final_goal, step, agent=None):
        if node not in self.env.pos or target not in self.env.pos:
            dx = 0.0
            dy = 0.0
        else:
            x, y = self.env.pos[node]
            gx, gy = self.env.pos[target]
            dx = (gx - x) / self.coord_scale
            dy = (gy - y) / self.coord_scale
        if node not in self.env.pos or final_goal not in self.env.pos:
            final_dx = 0.0
            final_dy = 0.0
        else:
            x, y = self.env.pos[node]
            gx, gy = self.env.pos[final_goal]
            final_dx = (gx - x) / self.coord_scale
            final_dy = (gy - y) / self.coord_scale
        local = self.env.observe(node, mode="walk")
        blocked_ratio = 0.0
        snow_avg = 0.0
        if local:
            blocked_ratio = sum(1.0 for v in local.values() if v["blocked"]) / len(local)
            snow_avg = sum(float(v["snow"]) for v in local.values()) / len(local)
        t = step / float(max(1, self.max_steps))
        features = [dx, dy, final_dx, final_dy, blocked_ratio, snow_avg, t]

        # Persona features (5 dims)
        if agent is not None:
            panic = float(getattr(agent, "panic_level", 0.0))
            speed = float(getattr(agent, "speed_multiplier", 1.0))
            familiarity = float(getattr(agent, "shelter_familiarity", 0.5))
            compliance = float(getattr(agent, "compliance_rate", 0.8))
            obs_err = float(getattr(agent, "observation_error_multiplier", 1.0))
            eff_compliance = min(1.0, max(0.0, compliance * (1.0 - 0.5 * panic)))
            eff_obs_err = min(1.0, max(0.0, obs_err * (1.0 + panic) / 3.0))
            features.extend([
                min(1.0, speed / 1.5),
                panic,
                familiarity,
                eff_compliance,
                eff_obs_err,
            ])
        else:
            features.extend([0.667, 0.0, 0.5, 0.8, 0.333])  # default neutral persona
        try:
            base_goal_dist = float(nx.shortest_path_length(self.env.G_walk, node, target, weight="weight"))
        except Exception:
            base_goal_dist = float("inf")
        candidates = self._candidate_neighbors(agent_id, node, target)
        for idx in range(self.max_neighbors):
            if idx < len(candidates):
                nxt = candidates[idx]
                edge_w = float(self.env.G_walk[node][nxt].get("weight", 1.0))
                edge_len_norm = min(1.0, edge_w / max(1.0, self.coord_scale))
                edge_snow = float(self.env.snow_depth_walk.get((node, nxt), 0.0))
                edge_blocked = 1.0 if self.env.is_blocked(node, nxt, mode="walk", belief=None) else 0.0
                try:
                    nxt_goal_dist = float(nx.shortest_path_length(self.env.G_walk, nxt, target, weight="weight"))
                except Exception:
                    nxt_goal_dist = float("inf")
                if np.isfinite(base_goal_dist) and np.isfinite(nxt_goal_dist):
                    progress = (base_goal_dist - nxt_goal_dist) / max(1.0, self.coord_scale)
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

    def select_next_node(self, agent, goal, step):
        target = self.current_target(agent.id, goal)
        candidates = self._candidate_neighbors(agent.id, agent.node, target)
        mask = np.zeros(self.num_actions, dtype=np.float32)
        if candidates:
            mask[1 : 1 + len(candidates)] = 1.0
        if np.any(mask[1:] > 0.5):
            mask[0] = 0.0
        else:
            mask[0] = 1.0

        prev_max_steps = self.max_steps
        self.max_steps = int(self.agent_step_budget.get(agent.id, self.base_max_steps))
        obs = self._obs(agent.id, agent.node, target, goal, step, agent=agent)
        obs_t = self._torch.tensor(obs, dtype=self._torch.float32, device=self.device).unsqueeze(0)
        h_prev = self.hidden.get(agent.id, self.model.init_hidden(device=self.device))
        with self._torch.no_grad():
            q_t, h_next = self.model(obs_t, h_prev)
        self.max_steps = prev_max_steps
        self.hidden[agent.id] = h_next.detach()

        q = q_t.detach().cpu().numpy().squeeze(0)
        q[mask <= 0.5] = -1e9
        if self.interaction_density_enabled:
            for idx, nxt in enumerate(candidates, start=1):
                q[idx] -= self._candidate_density_penalty(agent.node, nxt)
        action = int(np.argmax(q))
        if action == 0:
            return agent.node
        idx = action - 1
        if idx < 0 or idx >= len(candidates):
            return agent.node
        chosen = candidates[idx]
        if self.interaction_density_enabled:
            self._step_selected_next_counts[(agent.node, chosen)] += 1
        return chosen

    def committed_next(self, agent_id):
        return self.agent_committed_next.get(agent_id)

    def set_committed_next(self, agent_id, nxt):
        self.agent_committed_next[agent_id] = nxt

    def clear_committed_next(self, agent_id):
        self.agent_committed_next[agent_id] = None

    def debug_agent(self, agent, goal, step):
        target = self.current_target(agent.id, goal)
        candidates = self._candidate_neighbors(agent.id, agent.node, target)
        mask = np.zeros(self.num_actions, dtype=np.float32)
        if candidates:
            mask[1 : 1 + len(candidates)] = 1.0
        if np.any(mask[1:] > 0.5):
            mask[0] = 0.0
        else:
            mask[0] = 1.0

        prev_max_steps = self.max_steps
        self.max_steps = int(self.agent_step_budget.get(agent.id, self.base_max_steps))
        obs = self._obs(agent.id, agent.node, target, goal, step, agent=agent)
        obs_t = self._torch.tensor(obs, dtype=self._torch.float32, device=self.device).unsqueeze(0)
        h_prev = self.hidden.get(agent.id, self.model.init_hidden(device=self.device))
        with self._torch.no_grad():
            q_t, _ = self.model(obs_t, h_prev)
        self.max_steps = prev_max_steps

        q = q_t.detach().cpu().numpy().squeeze(0)
        masked_q = q.copy()
        masked_q[mask <= 0.5] = -1e9
        density_penalties = {}
        if self.interaction_density_enabled:
            for nxt in candidates:
                density_penalties[int(nxt) if isinstance(nxt, (int, np.integer)) else str(nxt)] = float(
                    self._candidate_density_penalty(agent.node, nxt)
                )
        return {
            "agent_id": int(agent.id),
            "node": int(agent.node) if isinstance(agent.node, (int, np.integer)) else str(agent.node),
            "goal": int(goal) if isinstance(goal, (int, np.integer)) else str(goal),
            "current_target": int(target) if isinstance(target, (int, np.integer)) else str(target),
            "committed_next": (
                int(self.committed_next(agent.id))
                if isinstance(self.committed_next(agent.id), (int, np.integer))
                else self.committed_next(agent.id)
            ),
            "candidates": [
                int(n) if isinstance(n, (int, np.integer)) else str(n)
                for n in candidates
            ],
            "q_values": [float(v) for v in q.tolist()],
            "masked_q_values": [float(v) for v in masked_q.tolist()],
            "action_mask": [int(v) for v in mask.astype(np.int32).tolist()],
            "density_penalties": density_penalties,
        }


def _reachable_walk_nodes(env):
    """Return walk nodes that can reach at least one shelter on the unblocked graph."""
    shelters = list(env.shelters)
    if not shelters:
        return list(env.G_walk.nodes())
    blocked = set(env.blocked_edges_walk)
    if blocked:
        keep = [(u, v) for u, v in env.G_walk.edges() if (u, v) not in blocked]
        graph = env.G_walk.edge_subgraph(keep).copy()
    else:
        graph = env.G_walk
    reachable = set()
    for s in shelters:
        if s not in graph:
            continue
        # All nodes that have a path TO shelter s
        try:
            reachable.update(nx.ancestors(graph, s))
            reachable.add(s)
        except Exception:
            pass
    # Fallback: if all nodes are isolated, allow any node
    return list(reachable) if reachable else list(env.G_walk.nodes())


def _reachable_drive_nodes(env):
    """Return drive nodes that can reach at least one walk shelter (via nearest drive node)."""
    shelters_walk = list(env.shelters)
    if not shelters_walk:
        return list(env.G_drive.nodes())
    blocked = set(env.blocked_edges_drive)
    if blocked:
        keep = [(u, v) for u, v in env.G_drive.edges() if (u, v) not in blocked]
        graph = env.G_drive.edge_subgraph(keep).copy()
    else:
        graph = env.G_drive
    # Map walk shelters to nearest drive nodes
    shelter_drive_nodes = set()
    for s in shelters_walk:
        if s in graph:
            shelter_drive_nodes.add(s)
        else:
            dn = _nearest_drive_node_for_shelter(env, s)
            if dn:
                shelter_drive_nodes.add(dn)
    if not shelter_drive_nodes:
        return list(env.G_drive.nodes())
    reachable = set()
    for s in shelter_drive_nodes:
        try:
            reachable.update(nx.ancestors(graph, s))
            reachable.add(s)
        except Exception:
            pass
    return list(reachable) if reachable else list(env.G_drive.nodes())


def _build_agents(env):
    peds = []
    cars = []
    shuttles = []
    nodes_walk = _reachable_walk_nodes(env)
    nodes_drive = _reachable_drive_nodes(env)
    _roles = list(config.EVAC_ROLE_WEIGHTS.keys())
    _role_w = list(config.EVAC_ROLE_WEIGHTS.values())
    for i in range(config.EVAC_PED_COUNT):
        start = random.choice(nodes_walk)
        ped = PedAgent(i + 1, start, env)
        ped.role = random.choices(_roles, weights=_role_w, k=1)[0]
        _apply_persona(ped, _assign_persona(ped.role))
        ped._delay_remaining = ped.decision_delay_steps
        peds.append(ped)
    for i in range(config.EVAC_CAR_COUNT):
        start = random.choice(nodes_drive)
        car = CarAgent(i + 1, start, env)
        car.role = random.choices(_roles, weights=_role_w, k=1)[0]
        _apply_persona(car, _assign_persona(car.role))
        car._delay_remaining = car.decision_delay_steps
        cars.append(car)
    route, stops = build_shuttle_route(env)
    for i in range(config.EVAC_BUS_COUNT):
        if route:
            shuttles.append(ShuttleAgent(i + 1, route[0], env, route, stops))
    return peds, cars, shuttles


def _goal_map(agents, graph, shelters, policy_name):
    state = {}
    goals = {}
    for a in agents:
        g = select_goal(a, graph, shelters, policy_name, state)
        goals[a.id] = g
        state[g] = state.get(g, 0) + 1
    return goals


def _capacity_aware_goal_map(agents, graph, goal_nodes, policy_name, env, goal_to_shelter=None):
    state = {}
    goals = {}
    goal_to_shelter = goal_to_shelter or {}

    for a in agents:
        # shelter familiarity: agent only knows a subset of shelters
        familiar_goals = _agent_familiar_goals(a, list(goal_nodes))

        if getattr(config, "EVAC_SHELTER_CAPACITY_ENABLED", False):
            available_goals = []
            for g in familiar_goals:
                shelter_node = goal_to_shelter.get(g, g)
                cap = env.shelter_capacity.get(shelter_node, 0)
                occ = env.shelter_occupancy.get(shelter_node, 0)
                assigned = state.get(shelter_node, 0)
                if occ + assigned < cap:
                    available_goals.append(g)
            candidate_goals = available_goals or familiar_goals
        else:
            candidate_goals = familiar_goals

        g = select_goal(a, graph, candidate_goals, policy_name, state)
        # compliance: non-compliant agents may ignore policy and pick randomly
        g = _apply_compliance(a, g, familiar_goals)
        goals[a.id] = g
        shelter_node = goal_to_shelter.get(g, g)
        state[shelter_node] = state.get(shelter_node, 0) + 1
    return goals


def _nearest_drive_node_for_shelter(env, shelter_node):
    if shelter_node in env.G_drive:
        return shelter_node
    if shelter_node not in env.pos:
        return None
    sx, sy = env.pos[shelter_node]
    best = None
    best_d = float("inf")
    for n in env.G_drive.nodes():
        if n not in env.pos:
            continue
        x, y = env.pos[n]
        d = (x - sx) * (x - sx) + (y - sy) * (y - sy)
        if d < best_d:
            best_d = d
            best = n
    return best


def _build_drive_shelter_mapping(env, shelters_walk):
    shelters_drive = []
    drive_goal_to_shelter = {}
    for s in shelters_walk:
        dn = s if s in env.G_drive else _nearest_drive_node_for_shelter(env, s)
        if dn is None:
            continue
        if dn not in shelters_drive:
            shelters_drive.append(dn)
        # Preserve first mapping so each drive goal has a stable shelter label.
        if dn not in drive_goal_to_shelter:
            drive_goal_to_shelter[dn] = s
    return shelters_drive, drive_goal_to_shelter


def _available_walk_shelters(env, shelters_walk):
    return [s for s in shelters_walk if env.shelter_has_capacity(s)]


def _available_drive_shelters(env, shelters_walk, drive_goal_to_shelter):
    available_walk = set(_available_walk_shelters(env, shelters_walk))
    return [dn for dn, walk_shelter in drive_goal_to_shelter.items() if walk_shelter in available_walk]


def _filter_taboo_walk_goals(candidate_goals, taboo_shelters):
    filtered = [g for g in candidate_goals if g not in taboo_shelters]
    return filtered or list(candidate_goals)


def _filter_taboo_drive_goals(candidate_goals, taboo_shelters, drive_goal_to_shelter):
    filtered = [g for g in candidate_goals if drive_goal_to_shelter.get(g, g) not in taboo_shelters]
    return filtered or list(candidate_goals)


def run_single_simulation(
    scenario_name,
    config_overrides,
    seed,
    policy_name,
    draw=False,
    draw_hooks=None,
    step_callback=None,
    emit_initial_state=False,
    drqn_checkpoint=None,
    drqn_device="auto",
    drqn_max_neighbors=None,
):
    if policy_name not in SUPPORTED_POLICIES:
        raise ValueError(f"Unsupported policy: {policy_name}. Supported: {sorted(SUPPORTED_POLICIES)}")

    random.seed(seed)
    np.random.seed(seed)

    with temporary_config(config_overrides):
        env = EvacEnv()
        peds, cars, shuttles = _build_agents(env)

        shelters_walk = list(env.shelters)
        shelters_drive, drive_goal_to_shelter = _build_drive_shelter_mapping(env, shelters_walk)

        ped_goals = _capacity_aware_goal_map(
            peds,
            env.G_walk,
            shelters_walk,
            policy_name,
            env,
        )
        car_goals = _capacity_aware_goal_map(
            cars,
            env.G_drive,
            shelters_drive,
            policy_name,
            env,
            goal_to_shelter=drive_goal_to_shelter,
        )
        for a in peds:
            a.target_shelter = ped_goals.get(a.id)
        for a in cars:
            goal_node = car_goals.get(a.id)
            a.target_shelter = drive_goal_to_shelter.get(goal_node, goal_node)

        drqn_ctrl = None
        if policy_name == "drqn":
            drqn_ctrl = _DRQNPedController(
                env=env,
                checkpoint_path=drqn_checkpoint,
                device=drqn_device,
                max_neighbors=drqn_max_neighbors,
                max_steps=config.EVAC_STEP_LIMIT,
            )
            drqn_ctrl.reset_agents(peds)
            for a in peds:
                drqn_ctrl.init_goal(a.id, a.node, ped_goals[a.id])
            drqn_ctrl.update_interaction_state(peds, cars)

        step_rows = []
        edge_counts = defaultdict(int)
        shelter_reassignments = 0
        ped_taboo_shelters = {a.id: set() for a in peds}
        car_taboo_shelters = {a.id: set() for a in cars}

        plot_state = None
        if draw and draw_hooks:
            plot_state = draw_hooks["init"](env, peds, cars, shuttles)

        if emit_initial_state:
            alive0 = sum(1 for a in peds + cars if a.alive)
            reached0 = sum(1 for a in peds + cars if a.reached)
            ped_reached0 = sum(1 for a in peds if a.reached)
            car_reached0 = sum(1 for a in cars if a.reached)
            avg_exp0 = sum(a.exposure for a in peds + cars) / max(1, (len(peds) + len(cars)))
            initial_row = {
                "step": -1,
                "alive": alive0,
                "reached": reached0,
                "ped_reached": ped_reached0,
                "car_reached": car_reached0,
                "avg_exposure": avg_exp0,
            }
            if step_callback is not None:
                step_callback(initial_row)

        stagnation_steps = 0
        stagnation_patience = max(1, int(getattr(config, "EVAC_STAGNATION_PATIENCE", 20)))
        for step in range(config.EVAC_STEP_LIMIT):
            env.step_hazards()
            if drqn_ctrl is not None:
                drqn_ctrl.update_interaction_state(peds, cars)
                drqn_ctrl.reset_decision_step()
            moved_this_step = False
            ped_stay = 0
            ped_selected_move = 0
            ped_motion = 0
            ped_node_changed = 0
            for a in peds:
                prev = a.node
                prev_edge = (a.edge_u, a.edge_v, float(a.edge_progress))
                # decision delay: agent waits N steps before starting to move
                if getattr(a, "_delay_remaining", 0) > 0:
                    a._delay_remaining -= 1
                    ped_stay += 1
                    continue
                if drqn_ctrl is not None and not a.reached and a.alive:
                    goal = ped_goals[a.id]
                    if a.node == goal:
                        a.reached = True
                    else:
                        committed_next = drqn_ctrl.committed_next(a.id)
                        if committed_next is not None and a.node == committed_next:
                            drqn_ctrl.clear_committed_next(a.id)
                            committed_next = None

                        curr_target = drqn_ctrl.current_target(a.id, goal)
                        try:
                            nx.shortest_path_length(drqn_ctrl._walk_graph_view(), a.node, curr_target, weight="weight")
                        except Exception:
                            if drqn_ctrl.replan_on_block:
                                drqn_ctrl.refresh_targets(a.id, a.node, goal)
                                curr_target = drqn_ctrl.current_target(a.id, goal)

                        if committed_next is None:
                            nxt = drqn_ctrl.select_next_node(a, goal, step)
                            if nxt != a.node:
                                drqn_ctrl.set_committed_next(a.id, nxt)
                                committed_next = nxt
                                ped_selected_move += 1
                            else:
                                ped_stay += 1

                        if committed_next is not None and committed_next != a.node:
                            if env.is_blocked(a.node, committed_next, mode="walk", belief=None):
                                drqn_ctrl.clear_committed_next(a.id)
                                if drqn_ctrl.replan_on_block:
                                    drqn_ctrl.refresh_targets(a.id, a.node, goal)
                                ped_stay += 1
                                committed_next = None

                        if committed_next is not None and committed_next != a.node:
                            # In DRQN mode, execute one graph-edge transition per action
                            # so runtime semantics match the training environment.
                            a.steps += 1
                            a.update_belief()
                            a.exposure += float(env.snow_depth_walk.get((a.node, committed_next), 0.0))
                            a.edge_u = a.node
                            a.edge_v = committed_next
                            a.edge_progress = float(env.G_walk[a.node][committed_next].get("weight", 1.0))
                            a.node = committed_next
                            visit_map = drqn_ctrl.agent_visit_counts.setdefault(a.id, {})
                            visit_map[a.node] = visit_map.get(a.node, 0) + 1
                            if a.node == committed_next:
                                drqn_ctrl.clear_committed_next(a.id)
                            curr_target = drqn_ctrl.current_target(a.id, goal)
                            if a.node == curr_target and curr_target != goal:
                                drqn_ctrl.advance_target(a.id, goal)
                            if a.node == goal:
                                a.reached = True
                        else:
                            a.move_along_path(a.node, config.EVAC_SPEED_WALK, mark_reached=False)
                else:
                    a.step(ped_goals[a.id])
                if a.reached and not getattr(a, "shelter_admitted", False):
                    target_shelter = getattr(a, "target_shelter", ped_goals.get(a.id))
                    if env.admit_to_shelter(target_shelter):
                        a.shelter_admitted = True
                    else:
                        a.reached = False
                        a.shelter_admitted = False
                        shelter_reassignments += 1
                        ped_taboo_shelters.setdefault(a.id, set()).add(target_shelter)
                        available_walk = _available_walk_shelters(env, shelters_walk)
                        if available_walk:
                            candidate_walk = _filter_taboo_walk_goals(
                                available_walk,
                                ped_taboo_shelters.get(a.id, set()),
                            )
                            assignment_state = dict(env.shelter_occupancy)
                            new_goal = select_goal(a, env.G_walk, candidate_walk, policy_name, assignment_state)
                            ped_goals[a.id] = new_goal
                            a.target_shelter = new_goal
                            a.path = []
                            a.path_pos = 0
                            if drqn_ctrl is not None:
                                drqn_ctrl.refresh_targets(a.id, a.node, new_goal)
                curr_edge = (a.edge_u, a.edge_v, float(a.edge_progress))
                if curr_edge != prev_edge:
                    ped_motion += 1
                if a.node != prev:
                    ped_node_changed += 1
                    moved_this_step = True
                    edge_counts[(prev, a.node)] += 1
            for a in cars:
                prev = a.node
                if getattr(a, "_delay_remaining", 0) > 0:
                    a._delay_remaining -= 1
                    continue
                a.step(car_goals[a.id])
                if a.reached and not getattr(a, "shelter_admitted", False):
                    target_shelter = getattr(a, "target_shelter", drive_goal_to_shelter.get(car_goals.get(a.id), car_goals.get(a.id)))
                    if env.admit_to_shelter(target_shelter):
                        a.shelter_admitted = True
                    else:
                        a.reached = False
                        a.shelter_admitted = False
                        shelter_reassignments += 1
                        car_taboo_shelters.setdefault(a.id, set()).add(target_shelter)
                        available_drive = _available_drive_shelters(env, shelters_walk, drive_goal_to_shelter)
                        if available_drive:
                            candidate_drive = _filter_taboo_drive_goals(
                                available_drive,
                                car_taboo_shelters.get(a.id, set()),
                                drive_goal_to_shelter,
                            )
                            assignment_state = {
                                dn: env.shelter_occupancy.get(drive_goal_to_shelter.get(dn), 0)
                                for dn in candidate_drive
                            }
                            new_drive_goal = select_goal(a, env.G_drive, candidate_drive, policy_name, assignment_state)
                            car_goals[a.id] = new_drive_goal
                            a.target_shelter = drive_goal_to_shelter.get(new_drive_goal, new_drive_goal)
                            a.path = []
                            a.path_pos = 0
                if a.node != prev:
                    moved_this_step = True
                    edge_counts[(prev, a.node)] += 1
            for b in shuttles:
                b.step()

            alive = sum(1 for a in peds + cars if a.alive)
            reached = sum(1 for a in peds + cars if a.reached)
            ped_reached = sum(1 for a in peds if a.reached)
            car_reached = sum(1 for a in cars if a.reached)
            avg_exp = sum(a.exposure for a in peds + cars) / max(1, (len(peds) + len(cars)))
            row = {
                "step": step,
                "alive": alive,
                "reached": reached,
                "ped_reached": ped_reached,
                "car_reached": car_reached,
                "ped_stay": ped_stay,
                "ped_selected_move": ped_selected_move,
                "ped_motion": ped_motion,
                "ped_node_changed": ped_node_changed,
                "avg_exposure": avg_exp,
            }
            step_rows.append(row)
            if step_callback is not None:
                step_callback(row)

            if reached >= len(peds) + len(cars):
                break

            if moved_this_step:
                stagnation_steps = 0
            else:
                stagnation_steps += 1
                if stagnation_steps >= stagnation_patience:
                    break

            if draw and draw_hooks and step % config.EVAC_DRAW_EVERY == 0:
                draw_hooks["update"](env, peds, cars, shuttles, step, plot_state)

        if draw and draw_hooks:
            draw_hooks["finalize"]()

        summary = summarize_run(
            step_rows=step_rows,
            peds=peds,
            cars=cars,
            edge_counts=edge_counts,
            step_limit=config.EVAC_STEP_LIMIT,
            scenario_name=scenario_name,
            seed=seed,
            policy_name=policy_name,
            extra_metrics={
                "shelter_reassignments": int(shelter_reassignments),
                "shelter_capacity_enabled": bool(getattr(config, "EVAC_SHELTER_CAPACITY_ENABLED", False)),
                "interaction_density_enabled": bool(getattr(config, "EVAC_INTERACTION_DENSITY_ENABLED", False)),
            },
        )
        if drqn_ctrl is not None:
            unreached_debug = []
            final_step = step_rows[-1]["step"] if step_rows else 0
            for a in peds:
                if a.alive and not a.reached:
                    unreached_debug.append(drqn_ctrl.debug_agent(a, ped_goals[a.id], final_step))
            summary["drqn_unreached_debug"] = unreached_debug
    return step_rows, summary


def run_batch(scenario, output_dir, drqn_checkpoint=None, drqn_device="auto", drqn_max_neighbors=None):
    os.makedirs(output_dir, exist_ok=True)

    all_rows = []
    policy_summary = {}
    num_runs = int(scenario.get("num_runs", 20))
    base_seed = int(scenario.get("base_seed", 42))
    policies = scenario.get("policies", ["round_robin"])
    config_overrides = scenario.get("config_overrides", {})

    for policy_name in policies:
        print(f"[batch] policy={policy_name} ({num_runs} runs)", flush=True)
        rows = []
        for i in range(num_runs):
            seed = base_seed + i
            print(f"[batch]   run {i + 1}/{num_runs}, seed={seed}", flush=True)
            _, summary = run_single_simulation(
                scenario_name=scenario.get("name", "scenario"),
                config_overrides=config_overrides,
                seed=seed,
                policy_name=policy_name,
                draw=False,
                draw_hooks=None,
                step_callback=None,
                drqn_checkpoint=drqn_checkpoint or scenario.get("drqn_checkpoint"),
                drqn_device=drqn_device,
                drqn_max_neighbors=drqn_max_neighbors,
            )
            rows.append(summary)
            all_rows.append(summary)
        policy_summary[policy_name] = aggregate_policy_rows(rows)

    runs_csv = os.path.join(output_dir, f"{scenario.get('name', 'scenario')}_runs.csv")
    if all_rows:
        fieldnames = []
        seen = set()
        for row in all_rows:
            for key in row.keys():
                if key not in seen:
                    seen.add(key)
                    fieldnames.append(key)
        with open(runs_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            w.writeheader()
            w.writerows(all_rows)

    summary_json = os.path.join(output_dir, f"{scenario.get('name', 'scenario')}_summary.json")
    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(
            {
                "scenario": scenario.get("name"),
                "num_runs": num_runs,
                "base_seed": base_seed,
                "policies": policies,
                "policy_summary": policy_summary,
            },
            f,
            indent=2,
        )

    management_summary = build_management_summary(
        policy_summary=policy_summary,
        scenario_name=scenario.get("name", "scenario"),
        baseline_policy=scenario.get("baseline_policy", "round_robin"),
    )
    management_txt = os.path.join(output_dir, f"{scenario.get('name', 'scenario')}_management_summary.txt")
    with open(management_txt, "w", encoding="utf-8") as f:
        f.write(management_summary)
        f.write("\n")

    return runs_csv, summary_json, management_txt, policy_summary
