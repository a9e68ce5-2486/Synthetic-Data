# evacuation_main.py
import csv
import os
import random
import numpy as np
import matplotlib.pyplot as plt
import config
try:
    import osmnx as ox
    from shapely.geometry import Point
except Exception:
    ox = None
    Point = None
from evac_env import EvacEnv
from agents.ped_agent import PedAgent
from agents.car_agent import CarAgent
from agents.shuttle_agent import ShuttleAgent, build_shuttle_route
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

    # edges
    if not stops_only:
        for u, v in env.G_walk.edges():
            x1, y1 = env.pos[u]
            x2, y2 = env.pos[v]
            color = "#2f333a"
            if (u, v) in env.blocked_edges_walk:
                color = "#b33939"
            plt.plot([x1, x2], [y1, y2], color=color, linewidth=0.7, alpha=0.7)

    # bus routes (static)
    if not stops_only:
        _draw_bus_routes(env, shuttles, color="#ff6b6b", alpha=0.35)

    # shelters (static)
    if not stops_only:
        for s in env.shelters:
            x, y = env.pos[s]
            plt.scatter([x], [y], c="#2ecc71", s=40, marker="P", zorder=5)

    # transit stops (static)
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
        # Projected coords are in meters; enforce minimum spacing.
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

    # agents (dynamic)
    ped_offsets = np.array([_agent_xy(env, a) for a in peds]) if peds else np.empty((0, 2))
    car_offsets = np.array([_agent_xy(env, a) for a in cars]) if cars else np.empty((0, 2))
    shuttle_offsets = np.array([_agent_xy(env, b) for b in shuttles]) if shuttles else np.empty((0, 2))

    if stops_only:
        ped_scatter = plt.scatter([], [], c="#ffd166", s=18, marker="o")
        car_scatter = plt.scatter([], [], c="#00b4d8", s=28, marker="^")
        shuttle_scatter = plt.scatter([], [], c="#ff6b6b", s=46, marker="s")
    else:
        ped_scatter = plt.scatter(
            ped_offsets[:, 0] if ped_offsets.size else [],
            ped_offsets[:, 1] if ped_offsets.size else [],
            c=["#ffd166" if not a.reached else "#7f8c8d" for a in peds],
            s=18,
            marker="o",
            edgecolors="#1a1a1a",
            linewidths=0.4,
            zorder=6,
        )
        car_scatter = plt.scatter(
            car_offsets[:, 0] if car_offsets.size else [],
            car_offsets[:, 1] if car_offsets.size else [],
            c=["#00b4d8" if not a.reached else "#95a5a6" for a in cars],
            s=28,
            marker="^",
            edgecolors="#1a1a1a",
            linewidths=0.4,
            zorder=7,
        )
        shuttle_scatter = plt.scatter(
            shuttle_offsets[:, 0] if shuttle_offsets.size else [],
            shuttle_offsets[:, 1] if shuttle_offsets.size else [],
            c="#ff6b6b",
            s=46,
            marker="s",
            edgecolors="#1a1a1a",
            linewidths=0.4,
            zorder=8,
        )

    title = ax.set_title("Evacuation Step 0", color="#e6e6e6", fontsize=12)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.text(0.01, 0.01, "ped: ●  car: ▲  shuttle: ■  shelter: ✚  shuttle stop: ◆  blocked: red",
            transform=ax.transAxes, fontsize=8, color="#cfd2d6", alpha=0.9)

    if not shuttle_xy:
        print("[stops] no stops plotted")
    else:
        if shuttle_xy:
            print(f"[stops] plotted shuttle stops: {len(shuttle_xy)}")

    return {
        "fig": fig,
        "ax": ax,
        "title": title,
        "ped_scatter": ped_scatter,
        "car_scatter": car_scatter,
        "shuttle_scatter": shuttle_scatter,
    }


def _update_plot(env, peds, cars, shuttles, step, state):
    stops_only = getattr(config, "EVAC_SHOW_STOPS_ONLY", False)
    if stops_only:
        state["title"].set_text(f"Evacuation Step {step}")
        state["fig"].canvas.draw_idle()
        plt.pause(0.001)
        return
    ped_offsets = np.array([_agent_xy(env, a) for a in peds]) if peds else np.empty((0, 2))
    car_offsets = np.array([_agent_xy(env, a) for a in cars]) if cars else np.empty((0, 2))
    shuttle_offsets = np.array([_agent_xy(env, b) for b in shuttles]) if shuttles else np.empty((0, 2))

    state["ped_scatter"].set_offsets(ped_offsets)
    state["car_scatter"].set_offsets(car_offsets)
    state["shuttle_scatter"].set_offsets(shuttle_offsets)
    state["ped_scatter"].set_color(["#ffd166" if not a.reached else "#7f8c8d" for a in peds])
    state["car_scatter"].set_color(["#00b4d8" if not a.reached else "#95a5a6" for a in cars])
    state["title"].set_text(f"Evacuation Step {step}")
    state["fig"].canvas.draw_idle()
    plt.pause(0.001)


def main():
    env = EvacEnv()
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
    # shuttle buses
    for i in range(config.EVAC_BUS_COUNT):
        route, stops = build_shuttle_route(env)
        print(f"[shuttle] route length for bus {i + 1}: {len(route)}")
        if route:
            shuttles.append(ShuttleAgent(i + 1, route[0], env, route, stops))

    shelters = list(env.shelters)
    shelters_drive = [s for s in shelters if s in env.G_drive]
    if not shelters_drive:
        shelters_drive = nodes_drive[:]

    os.makedirs("logs", exist_ok=True)
    metrics_path = os.path.join("logs", "phase1_evac_metrics.csv")
    with open(metrics_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "alive", "reached", "avg_exposure"])

        plt.ion()
        plot_state = _init_plot(env, peds, cars, shuttles)
        for step in range(config.EVAC_STEP_LIMIT):
            for a in peds:
                if not shelters:
                    break
                goal = shelters[a.id % len(shelters)]
                a.step(goal)
            for a in cars:
                if not shelters_drive:
                    break
                goal = shelters_drive[a.id % len(shelters_drive)]
                a.step(goal)
            for b in shuttles:
                b.step()

            alive = sum(1 for a in peds + cars if a.alive)
            reached = sum(1 for a in peds + cars if a.reached)
            avg_exp = sum(a.exposure for a in peds + cars) / max(1, (len(peds) + len(cars)))
            w.writerow([step, alive, reached, f"{avg_exp:.3f}"])

            if step % config.EVAC_DRAW_EVERY == 0:
                _update_plot(env, peds, cars, shuttles, step, plot_state)

        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
