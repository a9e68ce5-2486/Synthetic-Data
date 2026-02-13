# agents/shuttle_agent.py
import random
import math
import networkx as nx
import config
from bus_api import fetch_jsonp, extract_routes, extract_stops
from agents.base_agent import BaseAgent


class ShuttleAgent(BaseAgent):
    def __init__(self, aid, start, env, route, stop_nodes):
        super().__init__(aid, start, env, mode="drive")
        self.route = route
        self.route_idx = 0
        self.stop_nodes = set(stop_nodes)
        self.dwell = 0

    def step(self):
        if self.dwell > 0:
            self.dwell -= 1
            return
        if not self.route:
            return
        # Follow route nodes directly with smooth motion along edges.
        remaining = config.EVAC_SPEED_BUS
        guard = 0
        while remaining > 0 and guard < len(self.route):
            guard += 1
            goal = self.route[self.route_idx]
            if self.node == goal:
                self.route_idx = (self.route_idx + 1) % len(self.route)
                continue

            if not self.env.G_drive.has_edge(self.node, goal):
                self.route_idx = (self.route_idx + 1) % len(self.route)
                continue

            edge_len = self.env.G_drive[self.node][goal]["weight"]
            if self.edge_u != self.node or self.edge_v != goal:
                self.edge_u = self.node
                self.edge_v = goal
                self.edge_progress = 0.0
            self.edge_progress += remaining
            remaining = 0

            if self.edge_progress >= edge_len:
                self.node = goal
                self.edge_progress = self.edge_progress - edge_len
                self.edge_u = None
                self.edge_v = None
                if self.node in self.stop_nodes:
                    self.dwell = config.EVAC_SHUTTLE_DWELL_STEPS
                    break
                self.route_idx = (self.route_idx + 1) % len(self.route)


def _haversine_m(a, b, c, d):
    r = 6371000.0
    phi1 = math.radians(a)
    phi2 = math.radians(c)
    dphi = math.radians(c - a)
    dl = math.radians(d - b)
    h = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def build_shuttle_route(env):
    if not config.EVAC_BUS_API_URL:
        return [], []
    try:
        payload = fetch_jsonp(config.EVAC_BUS_API_URL)
        polylines = extract_routes(payload)
        stops = extract_stops(payload)
    except Exception:
        polylines = []
        stops = []

    route_nodes = []
    stop_nodes = []

    # route points
    if polylines:
        center_lat, center_lon = config.EVAC_FALLBACK_CENTER
        route_radius = config.EVAC_BUS_ROUTE_RADIUS_M
        max_snap = config.EVAC_BUS_SNAP_MAX_M
        for route in polylines:
            if not route:
                continue
            # filter to campus radius
            route = [
                (lat, lon)
                for lat, lon in route
                if _haversine_m(center_lat, center_lon, lat, lon) <= route_radius
            ]
            if not route:
                continue
            pts = route[:: max(1, config.EVAC_BUS_SAMPLE_EVERY)]
            nodes = []
            for lat, lon in pts:
                n, dist, dist_is_m = env.nearest_node_with_dist(lat, lon)
                if n is None:
                    continue
                if dist_is_m and dist > max_snap:
                    continue
                if not nodes or nodes[-1] != n:
                    nodes.append(n)
            if len(nodes) >= 2:
                route_nodes = nodes
                break

    # stop points
    if stops:
        center_lat, center_lon = config.EVAC_FALLBACK_CENTER
        route_radius = config.EVAC_BUS_ROUTE_RADIUS_M
        max_snap = config.EVAC_BUS_SNAP_MAX_M
        for s in stops:
            lat = s.get("lat")
            lon = s.get("lon")
            if lat is None or lon is None:
                continue
            if _haversine_m(center_lat, center_lon, lat, lon) > route_radius:
                continue
            n, dist, dist_is_m = env.nearest_node_with_dist(lat, lon)
            if n is None:
                continue
            if dist_is_m and dist > max_snap:
                continue
            stop_nodes.append(n)

    if not route_nodes:
        # fallback: synthetic loop over random stops
        nodes = list(env.G_drive.nodes())
        if len(nodes) < 2:
            return [], []
        stops = random.sample(nodes, min(config.EVAC_BUS_STOPS, len(nodes)))
        route = []
        for i in range(len(stops)):
            a = stops[i]
            b = stops[(i + 1) % len(stops)]
            try:
                path = nx.shortest_path(env.G_drive, a, b, weight="weight")
                if route and route[-1] == path[0]:
                    route.extend(path[1:])
                else:
                    route.extend(path)
            except Exception:
                continue
        return route, stops

    return route_nodes, stop_nodes
