# evac_env.py
import random
import math
import networkx as nx
import config
import shelter

try:
    import osmnx as ox
except Exception:
    ox = None


class EvacEnv:
    def __init__(self):
        self.G_walk = nx.DiGraph()
        self.G_drive = nx.DiGraph()
        self.G_drive_ll = None
        self.pos = {}
        self.blocked_edges_walk = set()
        self.blocked_edges_drive = set()
        self.snow_depth_walk = {}
        self.snow_depth_drive = {}
        self.shelters = set()
        self.shelter_capacity = {}
        self.shelter_occupancy = {}
        self._build_graph()
        self._init_hazards()
        self._init_shelters()

    def _build_graph(self):
        if not config.EVAC_USE_OSM:
            raise RuntimeError(
                "Fallback grid has been removed. Set EVAC_USE_OSM=True and provide a valid OSM setup."
            )
        if ox is None:
            raise RuntimeError(
                "OSMnx is unavailable. Install osmnx and retry; fallback grid is no longer supported."
            )
        ok = self._build_from_osm()
        if not ok:
            raise RuntimeError(
                "OSM graph build failed; fallback grid is no longer supported."
            )

    def _build_from_osm(self):
        if ox is not None:
            try:
                ox.settings.use_cache = config.EVAC_OSM_USE_CACHE
                ox.settings.log_console = False
                ox.settings.timeout = config.EVAC_OSM_TIMEOUT_S
                if hasattr(ox.settings, "requests_timeout"):
                    ox.settings.requests_timeout = config.EVAC_OSM_TIMEOUT_S
            except Exception:
                pass
        try:
            # Use a fixed-radius campus graph so map extent stays within school area.
            G_walk = ox.graph_from_point(
                config.EVAC_FALLBACK_CENTER,
                dist=config.EVAC_RADIUS_M,
                network_type="walk",
                simplify=True,
            )
            G_drive = ox.graph_from_point(
                config.EVAC_FALLBACK_CENTER,
                dist=config.EVAC_RADIUS_M,
                network_type="drive",
                simplify=True,
            )
        except Exception:
            try:
                G_walk = ox.graph_from_point(
                    config.EVAC_FALLBACK_CENTER,
                    dist=config.EVAC_RADIUS_M,
                    network_type="walk",
                    simplify=True,
                )
                G_drive = ox.graph_from_point(
                    config.EVAC_FALLBACK_CENTER,
                    dist=config.EVAC_RADIUS_M,
                    network_type="drive",
                    simplify=True,
                )
            except Exception:
                return False
        # keep lat/lon drive graph for nearest-node lookup
        self.G_drive_ll = G_drive
        try:
            G_walk = ox.project_graph(G_walk)
            G_drive = ox.project_graph(G_drive, to_crs=G_walk.graph.get("crs"))
        except Exception:
            return False

        # keep largest component
        G_walk = G_walk.subgraph(max(nx.weakly_connected_components(G_walk), key=len)).copy()
        G_drive = G_drive.subgraph(max(nx.weakly_connected_components(G_drive), key=len)).copy()

        # build digraph with edge features
        self.G_walk = nx.DiGraph()
        self.G_drive = nx.DiGraph()
        self.pos = {}
        for n, data in G_walk.nodes(data=True):
            self.pos[n] = (data.get("x", 0.0), data.get("y", 0.0))
            self.G_walk.add_node(n)
        # include drive-only nodes in position map
        for n, data in G_drive.nodes(data=True):
            if n not in self.pos:
                self.pos[n] = (data.get("x", 0.0), data.get("y", 0.0))
            self.G_drive.add_node(n)
        for u, v, data in G_walk.edges(data=True):
            length = data.get("length", 1.0)
            x1, y1 = self.pos[u]
            x2, y2 = self.pos[v]
            dx, dy = x2 - x1, y2 - y1
            horiz = max(1e-6, math.hypot(dx, dy))
            slope = abs(dy) / horiz  # proxy slope from geometry
            self.G_walk.add_edge(u, v, weight=length, slope=slope)
        for u, v, data in G_drive.edges(data=True):
            if u not in self.pos or v not in self.pos:
                continue
            length = data.get("length", 1.0)
            x1, y1 = self.pos[u]
            x2, y2 = self.pos[v]
            dx, dy = x2 - x1, y2 - y1
            horiz = max(1e-6, math.hypot(dx, dy))
            slope = abs(dy) / horiz
            self.G_drive.add_edge(u, v, weight=length, slope=slope)
        return True

    def _init_hazards(self):
        self.blocked_edges_walk.clear()
        self.blocked_edges_drive.clear()
        self.snow_depth_walk.clear()
        self.snow_depth_drive.clear()
        start_zero = getattr(config, "EVAC_SNOW_DYNAMIC", False) and getattr(config, "EVAC_SNOW_START_ZERO", False)
        init_block_prob = getattr(config, "EVAC_BLOCK_INIT_PROB", config.EVAC_BLOCK_PROB)
        for u, v in self.G_walk.edges():
            if random.random() < init_block_prob:
                self.blocked_edges_walk.add((u, v))
            self.snow_depth_walk[(u, v)] = 0.0 if start_zero else random.uniform(config.EVAC_SNOW_MIN, config.EVAC_SNOW_MAX)
        for u, v in self.G_drive.edges():
            if random.random() < init_block_prob:
                self.blocked_edges_drive.add((u, v))
            self.snow_depth_drive[(u, v)] = 0.0 if start_zero else random.uniform(config.EVAC_SNOW_MIN, config.EVAC_SNOW_MAX)

    def _init_shelters(self):
        nodes = list(self.G_walk.nodes())
        self.shelters = shelter.select_shelters(nodes, config.EVAC_SHELTER_COUNT)
        default_capacity = int(getattr(config, "EVAC_SHELTER_CAPACITY_PER_SITE", 999999))
        self.shelter_capacity = {s: default_capacity for s in self.shelters}
        self.shelter_occupancy = {s: 0 for s in self.shelters}

    def shelter_has_capacity(self, shelter_node):
        if not getattr(config, "EVAC_SHELTER_CAPACITY_ENABLED", False):
            return True
        if shelter_node not in self.shelter_capacity:
            return False
        return self.shelter_occupancy.get(shelter_node, 0) < self.shelter_capacity.get(shelter_node, 0)

    def admit_to_shelter(self, shelter_node):
        if shelter_node not in self.shelter_capacity:
            return False
        if not self.shelter_has_capacity(shelter_node):
            return False
        self.shelter_occupancy[shelter_node] = self.shelter_occupancy.get(shelter_node, 0) + 1
        return True

    def available_shelters(self):
        return [s for s in self.shelters if self.shelter_has_capacity(s)]

    def step_hazards(self):
        if not getattr(config, "EVAC_SNOW_DYNAMIC", False):
            return

        def _update_snow(depth_map, blocked_set):
            for edge, depth in depth_map.items():
                growth = config.EVAC_SNOW_ACCUM_PER_STEP + random.uniform(
                    -config.EVAC_SNOW_ACCUM_NOISE, config.EVAC_SNOW_ACCUM_NOISE
                )
                new_depth = max(0.0, min(1.0, depth + growth))
                depth_map[edge] = new_depth
                if (
                    edge not in blocked_set
                    and new_depth >= config.EVAC_BLOCK_FROM_SNOW_THRESHOLD
                    and random.random() < config.EVAC_BLOCK_FROM_SNOW_PROB
                ):
                    blocked_set.add(edge)

        _update_snow(self.snow_depth_walk, self.blocked_edges_walk)
        _update_snow(self.snow_depth_drive, self.blocked_edges_drive)

    def nearest_node(self, lat, lon):
        if ox is not None:
            try:
                if self.G_drive_ll is not None:
                    return ox.distance.nearest_nodes(self.G_drive_ll, lon, lat)
                return ox.distance.nearest_nodes(self.G_drive, lon, lat)
            except Exception:
                pass
        # brute-force fallback (lat/lon in projected coords may be inaccurate)
        best = None
        best_d = 1e18
        for n, (x, y) in self.pos.items():
            d = (x - lon) ** 2 + (y - lat) ** 2
            if d < best_d:
                best_d = d
                best = n
        return best

    def nearest_node_with_dist(self, lat, lon):
        if ox is not None:
            try:
                if self.G_drive_ll is not None:
                    node, dist = ox.distance.nearest_nodes(self.G_drive_ll, lon, lat, return_dist=True)
                    return node, dist, True
                node, dist = ox.distance.nearest_nodes(self.G_drive, lon, lat, return_dist=True)
                return node, dist, True
            except Exception:
                pass
        # fallback: brute-force on projected coords (distance not meaningful for lat/lon)
        best = None
        best_d = 1e18
        for n, (x, y) in self.pos.items():
            d = (x - lon) ** 2 + (y - lat) ** 2
            if d < best_d:
                best_d = d
                best = n
        return best, math.sqrt(best_d), False

    def observe(self, node, mode="walk", obs_error_multiplier=1.0):
        # partial observation within radius
        if node not in self.pos:
            return {}
        x0, y0 = self.pos[node]
        obs = {}
        G = self.G_walk if mode == "walk" else self.G_drive
        blocked_set = self.blocked_edges_walk if mode == "walk" else self.blocked_edges_drive
        snow = self.snow_depth_walk if mode == "walk" else self.snow_depth_drive
        obs_error = (config.EVAC_OBS_ERROR_WALK if mode == "walk" else config.EVAC_OBS_ERROR_DRIVE) * obs_error_multiplier
        for u, v in G.edges():
            x1, y1 = self.pos[u]
            if math.hypot(x1 - x0, y1 - y0) <= config.EVAC_OBS_RADIUS_M:
                blocked = (u, v) in blocked_set
                # observation noise
                if random.random() < obs_error:
                    blocked = not blocked
                obs[(u, v)] = {
                    "blocked": blocked,
                    "snow": snow[(u, v)],
                    "slope": G[u][v]["slope"],
                }
        return obs

    def edge_cost(self, u, v, mode="walk"):
        G = self.G_walk if mode == "walk" else self.G_drive
        snow = self.snow_depth_walk if mode == "walk" else self.snow_depth_drive
        base = G[u][v]["weight"]
        slope = G[u][v]["slope"]
        return base * (1.0 + config.EVAC_SNOW_ALPHA * snow + config.EVAC_SLOPE_ALPHA * slope)

    def is_blocked(self, u, v, mode="walk", belief=None):
        blocked_set = self.blocked_edges_walk if mode == "walk" else self.blocked_edges_drive
        if belief is None:
            return (u, v) in blocked_set
        if (u, v) in belief:
            return belief[(u, v)]["blocked"]
        return False
