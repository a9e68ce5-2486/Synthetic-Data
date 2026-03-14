import csv
import json
import os
import random
from collections import defaultdict

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


def _build_agents(env):
    peds = []
    cars = []
    shuttles = []
    nodes_walk = list(env.G_walk.nodes())
    nodes_drive = list(env.G_drive.nodes())
    for i in range(config.EVAC_PED_COUNT):
        start = random.choice(nodes_walk)
        ped = PedAgent(i + 1, start, env)
        ped.role = "faculty" if random.random() < config.EVAC_FACULTY_RATIO else "staff"
        peds.append(ped)
    for i in range(config.EVAC_CAR_COUNT):
        start = random.choice(nodes_drive)
        car = CarAgent(i + 1, start, env)
        car.role = "faculty" if random.random() < config.EVAC_FACULTY_RATIO else "staff"
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


def run_single_simulation(
    scenario_name,
    config_overrides,
    seed,
    policy_name,
    draw=False,
    draw_hooks=None,
    step_callback=None,
    emit_initial_state=False,
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

        ped_goals = _goal_map(peds, env.G_walk, shelters_walk, policy_name)
        car_goals = _goal_map(cars, env.G_drive, shelters_drive, policy_name)
        for a in peds:
            a.target_shelter = ped_goals.get(a.id)
        for a in cars:
            goal_node = car_goals.get(a.id)
            a.target_shelter = drive_goal_to_shelter.get(goal_node, goal_node)

        step_rows = []
        edge_counts = defaultdict(int)

        plot_state = None
        if draw and draw_hooks:
            plot_state = draw_hooks["init"](env, peds, cars, shuttles)

        if emit_initial_state:
            alive0 = sum(1 for a in peds + cars if a.alive)
            reached0 = sum(1 for a in peds + cars if a.reached)
            avg_exp0 = sum(a.exposure for a in peds + cars) / max(1, (len(peds) + len(cars)))
            initial_row = {"step": -1, "alive": alive0, "reached": reached0, "avg_exposure": avg_exp0}
            if step_callback is not None:
                step_callback(initial_row)

        for step in range(config.EVAC_STEP_LIMIT):
            env.step_hazards()
            for a in peds:
                prev = a.node
                a.step(ped_goals[a.id])
                if a.node != prev:
                    edge_counts[(prev, a.node)] += 1
            for a in cars:
                prev = a.node
                a.step(car_goals[a.id])
                if a.node != prev:
                    edge_counts[(prev, a.node)] += 1
            for b in shuttles:
                b.step()

            alive = sum(1 for a in peds + cars if a.alive)
            reached = sum(1 for a in peds + cars if a.reached)
            avg_exp = sum(a.exposure for a in peds + cars) / max(1, (len(peds) + len(cars)))
            row = {"step": step, "alive": alive, "reached": reached, "avg_exposure": avg_exp}
            step_rows.append(row)
            if step_callback is not None:
                step_callback(row)

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
        )
    return step_rows, summary


def run_batch(scenario, output_dir):
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
            )
            rows.append(summary)
            all_rows.append(summary)
        policy_summary[policy_name] = aggregate_policy_rows(rows)

    runs_csv = os.path.join(output_dir, f"{scenario.get('name', 'scenario')}_runs.csv")
    if all_rows:
        with open(runs_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
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
