import argparse
import json
import os
import random

import networkx as nx
import numpy as np

import config
from agents.ped_agent import PedAgent
from batch_runner import _DRQNPedController
from evac_env import EvacEnv
from policy import select_goal
from scenario_loader import load_scenario, temporary_config


def _serialize_node(node):
    if isinstance(node, (np.integer, int)):
        return int(node)
    return str(node)


def extract_route(env, checkpoint_path, start_node, role, seed, device, max_neighbors, goal_override=None):
    random.seed(seed)
    np.random.seed(seed)

    ped = PedAgent(1, start_node, env)
    ped.role = role
    shelters_walk = list(env.shelters)
    goal = goal_override if goal_override is not None else select_goal(ped, env.G_walk, shelters_walk, "drqn", {})

    ctrl = _DRQNPedController(
        env=env,
        checkpoint_path=checkpoint_path,
        device=device,
        max_neighbors=max_neighbors,
        max_steps=config.EVAC_STEP_LIMIT,
    )
    ctrl.reset_agents([ped])
    ctrl.init_goal(ped.id, ped.node, goal)

    path_nodes = [ped.node]
    traversed_edges = []
    replan_count = 0
    target_history = [ctrl.current_target(ped.id, goal)]

    for step in range(config.EVAC_STEP_LIMIT):
        env.step_hazards()
        if ped.node == goal:
            ped.reached = True
            break

        committed_next = ctrl.committed_next(ped.id)
        if committed_next is not None and ped.node == committed_next:
            ctrl.clear_committed_next(ped.id)
            committed_next = None

        curr_target = ctrl.current_target(ped.id, goal)
        try:
            nx.shortest_path_length(ctrl._walk_graph_view(), ped.node, curr_target, weight="weight")
        except Exception:
            if ctrl.replan_on_block:
                ctrl.refresh_targets(ped.id, ped.node, goal)
                replan_count += 1
                curr_target = ctrl.current_target(ped.id, goal)
                target_history.append(curr_target)

        if committed_next is None:
            nxt = ctrl.select_next_node(ped, goal, step)
            if nxt != ped.node:
                ctrl.set_committed_next(ped.id, nxt)
                committed_next = nxt

        if committed_next is None or committed_next == ped.node:
            ped.steps += 1
            ped.update_belief()
            continue

        if env.is_blocked(ped.node, committed_next, mode="walk", belief=None):
            ctrl.clear_committed_next(ped.id)
            if ctrl.replan_on_block:
                ctrl.refresh_targets(ped.id, ped.node, goal)
                replan_count += 1
                target_history.append(ctrl.current_target(ped.id, goal))
            ped.steps += 1
            ped.update_belief()
            continue

        prev = ped.node
        ped.steps += 1
        ped.update_belief()
        ped.exposure += float(env.snow_depth_walk.get((ped.node, committed_next), 0.0))
        ped.edge_u = ped.node
        ped.edge_v = committed_next
        ped.edge_progress = float(env.G_walk[ped.node][committed_next].get("weight", 1.0))
        ped.node = committed_next
        visit_map = ctrl.agent_visit_counts.setdefault(ped.id, {})
        visit_map[ped.node] = visit_map.get(ped.node, 0) + 1
        ctrl.clear_committed_next(ped.id)

        traversed_edges.append((prev, ped.node))
        path_nodes.append(ped.node)

        curr_target = ctrl.current_target(ped.id, goal)
        if ped.node == curr_target and curr_target != goal:
            ctrl.advance_target(ped.id, goal)
            target_history.append(ctrl.current_target(ped.id, goal))
        if ped.node == goal:
            ped.reached = True
            break

    return {
        "start_node": _serialize_node(start_node),
        "recommended_shelter": _serialize_node(goal),
        "reached": bool(ped.reached),
        "steps": int(ped.steps),
        "exposure": float(ped.exposure),
        "path_nodes": [_serialize_node(n) for n in path_nodes],
        "path_length_nodes": len(path_nodes),
        "traversed_edges": [[_serialize_node(u), _serialize_node(v)] for u, v in traversed_edges],
        "replan_count": int(replan_count),
        "target_history": [_serialize_node(n) for n in target_history],
    }


def main():
    parser = argparse.ArgumentParser(description="Extract a DRQN-recommended shelter route for one pedestrian start node.")
    parser.add_argument("--scenario", default="scenarios/enterprise_baseline.json")
    parser.add_argument("--checkpoint", default="logs/drqn_blocked_finetune/drqn_torch_best.pt")
    parser.add_argument("--start-node", required=True)
    parser.add_argument("--seed", type=int, default=20260323)
    parser.add_argument("--role", default="staff", choices=["staff", "faculty"])
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-neighbors", type=int, default=None)
    parser.add_argument("--output-dir", default="logs/route_recommendation")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    os.makedirs(args.output_dir, exist_ok=True)

    try:
        start_node = int(args.start_node)
    except ValueError:
        start_node = args.start_node

    with temporary_config(scenario.get("config_overrides", {})):
        env = EvacEnv()
        if start_node not in env.G_walk:
            raise ValueError(f"Start node not found in walk graph: {start_node}")
        result = extract_route(
            env=env,
            checkpoint_path=args.checkpoint,
            start_node=start_node,
            role=args.role,
            seed=args.seed,
            device=args.device,
            max_neighbors=args.max_neighbors,
            goal_override=None,
        )

    result.update(
        {
            "scenario": scenario.get("name", "scenario"),
            "checkpoint": args.checkpoint,
            "seed": int(args.seed),
            "role": args.role,
        }
    )

    stem = f"{result['scenario']}_{result['start_node']}"
    json_path = os.path.join(args.output_dir, f"{stem}_route.json")
    md_path = os.path.join(args.output_dir, f"{stem}_route.md")

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Route Recommendation\n\n")
        f.write(f"- Scenario: `{result['scenario']}`\n")
        f.write(f"- Checkpoint: `{result['checkpoint']}`\n")
        f.write(f"- Seed: `{result['seed']}`\n")
        f.write(f"- Role: `{result['role']}`\n")
        f.write(f"- Start node: `{result['start_node']}`\n")
        f.write(f"- Recommended shelter: `{result['recommended_shelter']}`\n")
        f.write(f"- Reached: `{result['reached']}`\n")
        f.write(f"- Steps: `{result['steps']}`\n")
        f.write(f"- Exposure: `{result['exposure']:.4f}`\n")
        f.write(f"- Replan count: `{result['replan_count']}`\n")
        f.write(f"- Path length (nodes): `{result['path_length_nodes']}`\n")
        f.write(f"- Path nodes: `{result['path_nodes']}`\n")

    print(f"[route] json: {json_path}")
    print(f"[route] md: {md_path}")
    print(f"[route] shelter: {result['recommended_shelter']}")
    print(f"[route] reached: {result['reached']}")
    print(f"[route] path_length_nodes: {result['path_length_nodes']}")


if __name__ == "__main__":
    main()
