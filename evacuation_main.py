# evacuation_main.py
import argparse
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import config

try:
    import osmnx as ox
    from shapely.geometry import Point
except Exception:
    ox = None
    Point = None

from batch_runner import run_single_simulation, run_batch
from scenario_loader import load_scenario
from transit_stops import fetch_shuttle_stops


def _agent_xy(env, agent):
    if agent.edge_u is None or agent.edge_v is None:
        return env.pos[agent.node]
    u = agent.edge_u
    v = agent.edge_v
    if u not in env.pos or v not in env.pos:
        return env.pos[agent.node]
    x1, y1 = env.pos[u]
    x2, y2 = env.pos[v]
    edge_len = env.G_walk[u][v]["weight"] if agent.mode == "walk" else env.G_drive[u][v]["weight"]
    ratio = min(1.0, max(0.0, agent.edge_progress / edge_len)) if edge_len else 0.0
    return (x1 + (x2 - x1) * ratio, y1 + (y2 - y1) * ratio)


def _draw_bus_routes(env, buses, color="#ff6b6b", alpha=0.35):
    for b in buses:
        if not b.route or len(b.route) < 2:
            continue
        xs = []
        ys = []
        for n in b.route:
            if n in env.pos:
                x, y = env.pos[n]
                xs.append(x)
                ys.append(y)
        if xs and ys:
            plt.plot(xs, ys, color=color, linewidth=1.2, alpha=alpha, zorder=2)


def _hazard_edge_style(env, u, v):
    show_hazard = getattr(config, "EVAC_SHOW_HAZARD_OVERLAY", True)
    snow_cmap = plt.get_cmap("Blues")
    if (u, v) in env.blocked_edges_walk:
        return "#b33939", 1.2, 0.9
    snow = env.snow_depth_walk.get((u, v), 0.0)
    snow = max(0.0, min(1.0, float(snow)))
    if show_hazard and snow > 0.02:
        return snow_cmap(0.35 + 0.6 * snow), 0.6 + 1.6 * snow, 0.2 + 0.6 * snow
    return "#2f333a", 0.7, 0.7


def _init_plot(env, peds, cars, shuttles):
    fig, ax = plt.subplots()
    ax.set_facecolor("#0f1116")
    stops_only = getattr(config, "EVAC_SHOW_STOPS_ONLY", False)
    xs = [p[0] for p in env.pos.values()]
    ys = [p[1] for p in env.pos.values()]
    if xs and ys:
        xpad = 0.0
        ypad = 0.0
        plt.xlim(min(xs) - xpad, max(xs) + xpad)
        plt.ylim(min(ys) - ypad, max(ys) + ypad)

    if not stops_only:
        edge_lines = {}
        for u, v in env.G_walk.edges():
            x1, y1 = env.pos[u]
            x2, y2 = env.pos[v]
            color, width, alpha = _hazard_edge_style(env, u, v)
            (line,) = plt.plot([x1, x2], [y1, y2], color=color, linewidth=width, alpha=alpha, zorder=1)
            edge_lines[(u, v)] = line
    else:
        edge_lines = {}

    if not stops_only:
        _draw_bus_routes(env, shuttles, color="#ff6b6b", alpha=0.35)

    shelter_texts = {}
    shelter_rank = sorted(list(env.shelters), key=lambda n: str(n))
    if not stops_only:
        for s in env.shelters:
            x, y = env.pos[s]
            plt.scatter([x], [y], c="#2ecc71", s=40, marker="P", zorder=5)
            t = plt.text(x, y, "0", color="#c8f7c5", fontsize=8, ha="left", va="bottom", zorder=10)
            shelter_texts[s] = t
        panel = ax.text(
            1.01,
            0.98,
            "",
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            color="#d8f3dc",
            family="monospace",
            bbox={"facecolor": "#0b1020", "edgecolor": "#2a3448", "alpha": 0.85, "boxstyle": "round,pad=0.4"},
        )
    else:
        panel = None

    def _map_stops_to_xy(stops):
        pts = []
        for s in stops:
            lat = s.get("lat")
            lon = s.get("lon")
            if lat is None or lon is None:
                continue
            if ox is not None and Point is not None and env.G_walk is not None:
                try:
                    geom = Point(lon, lat)
                    projected, _ = ox.projection.project_geometry(geom, to_crs=env.G_walk.graph.get("crs"))
                    pts.append((projected.x, projected.y))
                    continue
                except Exception:
                    pass
            n = env.nearest_node(lat, lon)
            if n is None or n not in env.pos:
                continue
            pts.append(env.pos[n])
        return pts

    def _sample_points(points, min_dist_m):
        if not points:
            return []
        min_sq = min_dist_m * min_dist_m
        kept = []
        for x, y in points:
            ok = True
            for kx, ky in kept:
                dx = x - kx
                dy = y - ky
                if dx * dx + dy * dy <= min_sq:
                    ok = False
                    break
            if ok:
                kept.append((x, y))
        return kept

    shuttle_stops = fetch_shuttle_stops(config.EVAC_FALLBACK_CENTER, config.EVAC_RADIUS_M)
    print(f"[stops] shuttle stops fetched: {len(shuttle_stops)}")

    shuttle_xy = _map_stops_to_xy(shuttle_stops)
    shuttle_xy = _sample_points(shuttle_xy, config.EVAC_STOP_SAMPLE_M)
    print(f"[stops] shuttle stops mapped: {len(shuttle_xy)}")
    if shuttle_xy:
        xs_s, ys_s = zip(*shuttle_xy)
        plt.scatter(xs_s, ys_s, c="#f72585", s=40, marker="D", zorder=9, alpha=1.0, linewidths=0.6, edgecolors="#ffffff")
    if stops_only:
        xs_all = []
        ys_all = []
        if shuttle_xy:
            xs_all.extend(xs_s)
            ys_all.extend(ys_s)
        if xs_all and ys_all:
            xpad = (max(xs_all) - min(xs_all)) * 0.05
            ypad = (max(ys_all) - min(ys_all)) * 0.05
            plt.xlim(min(xs_all) - xpad, max(xs_all) + xpad)
            plt.ylim(min(ys_all) - ypad, max(ys_all) + ypad)

    active_peds = [a for a in peds if not a.reached]
    active_cars = [a for a in cars if not a.reached]
    ped_offsets = np.array([_agent_xy(env, a) for a in active_peds]) if active_peds else np.empty((0, 2))
    car_offsets = np.array([_agent_xy(env, a) for a in active_cars]) if active_cars else np.empty((0, 2))
    shuttle_offsets = np.array([_agent_xy(env, b) for b in shuttles]) if shuttles else np.empty((0, 2))

    if stops_only:
        ped_scatter = plt.scatter([], [], c="#ffd166", s=28, marker="o")
        car_scatter = plt.scatter([], [], c="#00b4d8", s=40, marker="^")
        shuttle_scatter = plt.scatter([], [], c="#ff6b6b", s=58, marker="s")
    else:
        ped_scatter = plt.scatter(
            ped_offsets[:, 0] if ped_offsets.size else [],
            ped_offsets[:, 1] if ped_offsets.size else [],
            c="#ffd166",
            s=28,
            marker="o",
            edgecolors="#1a1a1a",
            linewidths=0.4,
            zorder=6,
        )
        car_scatter = plt.scatter(
            car_offsets[:, 0] if car_offsets.size else [],
            car_offsets[:, 1] if car_offsets.size else [],
            c="#00b4d8",
            s=40,
            marker="^",
            edgecolors="#1a1a1a",
            linewidths=0.4,
            zorder=7,
        )
        shuttle_scatter = plt.scatter(
            shuttle_offsets[:, 0] if shuttle_offsets.size else [],
            shuttle_offsets[:, 1] if shuttle_offsets.size else [],
            c="#ff6b6b",
            s=58,
            marker="s",
            edgecolors="#1a1a1a",
            linewidths=0.4,
            zorder=8,
        )

    title = ax.set_title("Evacuation Step 0", color="#e6e6e6", fontsize=12)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.text(0.01, 0.01, "ped: ●  car: ▲  shuttle: ■  shelter: ✚  shuttle stop: ◆  blocked: red  snow: blue",
            transform=ax.transAxes, fontsize=8, color="#cfd2d6", alpha=0.9)

    if not shuttle_xy:
        print("[stops] no stops plotted")
    else:
        print(f"[stops] plotted shuttle stops: {len(shuttle_xy)}")

    return {
        "fig": fig,
        "ax": ax,
        "title": title,
        "ped_scatter": ped_scatter,
        "car_scatter": car_scatter,
        "shuttle_scatter": shuttle_scatter,
        "edge_lines": edge_lines,
        "shelter_texts": shelter_texts,
        "shelter_rank": shelter_rank,
        "panel": panel,
    }


def _update_plot(env, peds, cars, shuttles, step, state):
    stops_only = getattr(config, "EVAC_SHOW_STOPS_ONLY", False)
    if stops_only:
        state["title"].set_text(f"Evacuation Step {step}")
        state["fig"].canvas.draw_idle()
        plt.pause(0.001)
        return

    hazard_draw_every = max(1, int(getattr(config, "EVAC_HAZARD_DRAW_EVERY", 3)))
    if step % hazard_draw_every == 0 and state.get("edge_lines"):
        for (u, v), line in state["edge_lines"].items():
            color, width, alpha = _hazard_edge_style(env, u, v)
            line.set_color(color)
            line.set_linewidth(width)
            line.set_alpha(alpha)

    active_peds = [a for a in peds if not a.reached]
    active_cars = [a for a in cars if not a.reached]
    ped_offsets = np.array([_agent_xy(env, a) for a in active_peds]) if active_peds else np.empty((0, 2))
    car_offsets = np.array([_agent_xy(env, a) for a in active_cars]) if active_cars else np.empty((0, 2))
    shuttle_offsets = np.array([_agent_xy(env, b) for b in shuttles]) if shuttles else np.empty((0, 2))

    state["ped_scatter"].set_offsets(ped_offsets)
    state["car_scatter"].set_offsets(car_offsets)
    state["shuttle_scatter"].set_offsets(shuttle_offsets)
    state["ped_scatter"].set_color("#ffd166")
    state["car_scatter"].set_color("#00b4d8")

    if state.get("shelter_texts"):
        counts = {}
        for a in peds + cars:
            if a.reached:
                s = a.node if a.node in state["shelter_texts"] else getattr(a, "target_shelter", None)
                if s is not None:
                    counts[s] = counts.get(s, 0) + 1
        for s, txt in state["shelter_texts"].items():
            txt.set_text(str(counts.get(s, 0)))
        if state.get("panel") is not None:
            lines = []
            total_arrived_to_shelter = 0
            for i, s in enumerate(state.get("shelter_rank", []), start=1):
                c = counts.get(s, 0)
                total_arrived_to_shelter += c
                lines.append(f"S{i:02d}: {c:3d}")
            total_reached = sum(1 for a in peds + cars if a.reached)
            lines.append(f"Step: {step:4d}")
            lines.append("-----")
            lines.append(f"Shelter Total: {total_arrived_to_shelter:3d}")
            lines.append(f"Reached Total: {total_reached:3d}")
            state["panel"].set_text("\n".join(lines))

    state["title"].set_text(f"Evacuation Step {step}")
    state["fig"].canvas.draw_idle()
    plt.pause(0.001)


def _write_step_csv(path, step_rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "step",
            "alive",
            "reached",
            "ped_reached",
            "car_reached",
            "ped_stay",
            "ped_selected_move",
            "ped_motion",
            "ped_node_changed",
            "avg_exposure",
        ])
        for row in step_rows:
            w.writerow([
                row["step"],
                row["alive"],
                row["reached"],
                row.get("ped_reached", ""),
                row.get("car_reached", ""),
                row.get("ped_stay", ""),
                row.get("ped_selected_move", ""),
                row.get("ped_motion", ""),
                row.get("ped_node_changed", ""),
                f"{row['avg_exposure']:.3f}",
            ])


def main():
    parser = argparse.ArgumentParser(description="Campus evacuation simulation")
    parser.add_argument("--mode", choices=["interactive", "batch"], default="interactive")
    parser.add_argument("--scenario", default="scenarios/enterprise_baseline.json")
    parser.add_argument("--policy", default="round_robin")
    parser.add_argument("--output-dir", default="logs")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--visual-speed-scale", type=float, default=6.0)
    parser.add_argument("--log-every", type=int, default=5)
    parser.add_argument("--drqn-checkpoint", default=None, help="Path to DRQN checkpoint (.pt) for --policy drqn")
    parser.add_argument("--drqn-device", default="auto", choices=["auto", "cpu", "cuda", "mps"])
    parser.add_argument("--drqn-max-neighbors", type=int, default=None)
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)

    if args.mode == "batch":
        runs_csv, summary_json, management_txt, policy_summary = run_batch(
            scenario,
            args.output_dir,
            drqn_checkpoint=args.drqn_checkpoint,
            drqn_device=args.drqn_device,
            drqn_max_neighbors=args.drqn_max_neighbors,
        )
        print(f"[batch] runs csv: {runs_csv}")
        print(f"[batch] summary json: {summary_json}")
        print(f"[batch] management summary: {management_txt}")
        for policy_name, summary in policy_summary.items():
            print(f"[batch] {policy_name}: {summary}")
        return

    seed = args.seed if args.seed is not None else int(scenario.get("base_seed", 42))
    draw = not args.no_plot
    hooks = None
    if draw:
        plt.ion()
        hooks = {
            "init": _init_plot,
            "update": _update_plot,
            "finalize": lambda: (plt.ioff(), plt.show()),
        }

    config_overrides = dict(scenario.get("config_overrides", {}))
    if draw and args.visual_speed_scale != 1.0:
        config_overrides["EVAC_SPEED_WALK"] = float(config_overrides.get("EVAC_SPEED_WALK", config.EVAC_SPEED_WALK)) * args.visual_speed_scale
        config_overrides["EVAC_SPEED_CAR"] = float(config_overrides.get("EVAC_SPEED_CAR", config.EVAC_SPEED_CAR)) * args.visual_speed_scale
        config_overrides["EVAC_SPEED_BUS"] = float(config_overrides.get("EVAC_SPEED_BUS", config.EVAC_SPEED_BUS)) * args.visual_speed_scale

    metrics_path = os.path.join(args.output_dir, "phase1_evac_metrics.csv")
    os.makedirs(args.output_dir, exist_ok=True)
    metrics_file = open(metrics_path, "w", newline="", encoding="utf-8")
    w = csv.writer(metrics_file)
    w.writerow([
        "step",
        "alive",
        "reached",
        "ped_reached",
        "car_reached",
        "ped_stay",
        "ped_selected_move",
        "ped_motion",
        "ped_node_changed",
        "avg_exposure",
    ])

    def _on_step(row):
        w.writerow([
            row["step"],
            row["alive"],
            row["reached"],
            row.get("ped_reached", ""),
            row.get("car_reached", ""),
            row.get("ped_stay", ""),
            row.get("ped_selected_move", ""),
            row.get("ped_motion", ""),
            row.get("ped_node_changed", ""),
            f"{row['avg_exposure']:.3f}",
        ])
        if row["step"] == -1:
            print(
                f"[step   -1] alive={row['alive']:3d} "
                f"reached={row['reached']:3d} "
                f"ped={row.get('ped_reached', 0):3d} "
                f"car={row.get('car_reached', 0):3d} "
                f"stay={row.get('ped_stay', 0):3d} "
                f"sel={row.get('ped_selected_move', 0):3d} "
                f"motion={row.get('ped_motion', 0):3d} "
                f"nodechg={row.get('ped_node_changed', 0):3d} "
                f"avg_exposure={row['avg_exposure']:.3f}",
                flush=True,
            )
        elif args.log_every > 0 and row["step"] % args.log_every == 0:
            print(
                f"[step {row['step']:4d}] alive={row['alive']:3d} "
                f"reached={row['reached']:3d} "
                f"ped={row.get('ped_reached', 0):3d} "
                f"car={row.get('car_reached', 0):3d} "
                f"stay={row.get('ped_stay', 0):3d} "
                f"sel={row.get('ped_selected_move', 0):3d} "
                f"motion={row.get('ped_motion', 0):3d} "
                f"nodechg={row.get('ped_node_changed', 0):3d} "
                f"avg_exposure={row['avg_exposure']:.3f}",
                flush=True,
            )

    try:
        step_rows, summary = run_single_simulation(
            scenario_name=scenario.get("name", "interactive"),
            config_overrides=config_overrides,
            seed=seed,
            policy_name=args.policy,
            draw=draw,
            draw_hooks=hooks,
            step_callback=_on_step,
            emit_initial_state=True,
            drqn_checkpoint=args.drqn_checkpoint,
            drqn_device=args.drqn_device,
            drqn_max_neighbors=args.drqn_max_neighbors,
        )
    finally:
        metrics_file.close()

    if not step_rows:
        _write_step_csv(metrics_path, step_rows)
    print(f"[interactive] step metrics: {metrics_path}")
    print(f"[interactive] summary: {summary}")
    drqn_debug = summary.get("drqn_unreached_debug")
    if drqn_debug:
        print("[interactive] unreached DRQN pedestrians:")
        for item in drqn_debug:
            print(
                f"  agent={item['agent_id']} node={item['node']} "
                f"goal={item['goal']} target={item['current_target']} "
                f"committed_next={item['committed_next']}"
            )
            print(f"    candidates={item['candidates']}")
            print(f"    action_mask={item['action_mask']}")
            print(f"    q_values={item['q_values']}")
            print(f"    masked_q_values={item['masked_q_values']}")


if __name__ == "__main__":
    main()
