# agents/base_agent.py
import networkx as nx
import config


class BaseAgent:
    def __init__(self, aid, start, env, mode):
        self.id = aid
        self.node = start
        self.env = env
        self.mode = mode  # walk or drive
        self.role = None
        self.alive = True
        self.reached = False
        self.exposure = 0.0
        self.steps = 0
        self.belief = {}
        self.path = []
        self.path_pos = 0
        self.edge_u = None
        self.edge_v = None
        self.edge_progress = 0.0

    def graph(self):
        return self.env.G_walk if self.mode == "walk" else self.env.G_drive

    def update_belief(self):
        obs = self.env.observe(self.node, self.mode)
        self.belief.update(obs)

    def _cost(self, u, v, data):
        base = data.get("weight", 1.0)
        snow = self.env.snow_depth_walk.get((u, v), 0.0) if self.mode == "walk" else self.env.snow_depth_drive.get((u, v), 0.0)
        slope = data.get("slope", 0.0)
        return base * (1.0 + config.EVAC_SNOW_ALPHA * snow + config.EVAC_SLOPE_ALPHA * slope)

    def plan(self, goal):
        G = self.graph().copy()
        for (u, v), info in self.belief.items():
            if info["blocked"] and G.has_edge(u, v):
                G.remove_edge(u, v)
        try:
            self.path = nx.shortest_path(G, self.node, goal, weight=self._cost)
        except Exception:
            self.path = []
        self.path_pos = 0
        self.edge_u = None
        self.edge_v = None
        self.edge_progress = 0.0

    def move_along_path(self, goal, speed, mark_reached=True):
        if (mark_reached and self.reached) or not self.alive:
            return
        self.steps += 1
        self.update_belief()
        if self.node == goal:
            if mark_reached:
                self.reached = True
            return

        if not self.path or self.node not in self.path:
            self.plan(goal)
        if len(self.path) < 2:
            self.exposure += 1.0
            return

        remaining = speed
        while remaining > 0 and not self.reached:
            if self.path_pos >= len(self.path) or self.path[self.path_pos] != self.node:
                try:
                    self.path_pos = self.path.index(self.node)
                except ValueError:
                    self.plan(goal)
                    if not self.path or len(self.path) < 2:
                        self.exposure += 1.0
                        return
            if self.path_pos + 1 >= len(self.path):
                if mark_reached:
                    self.reached = True
                break
            nxt = self.path[self.path_pos + 1]
            if self.env.is_blocked(self.node, nxt, self.mode, self.belief):
                self.plan(goal)
                self.exposure += 1.0
                break

            edge_len = self.graph()[self.node][nxt]["weight"]
            self.exposure += self.env.snow_depth_walk.get((self.node, nxt), 0.0) if self.mode == "walk" else self.env.snow_depth_drive.get((self.node, nxt), 0.0)

            if self.edge_u != self.node or self.edge_v != nxt:
                self.edge_u = self.node
                self.edge_v = nxt
                self.edge_progress = 0.0
            self.edge_progress += remaining
            remaining = 0

            if self.edge_progress >= edge_len:
                # advance to next node
                self.node = nxt
                self.path_pos += 1
                self.edge_progress = self.edge_progress - edge_len
                self.edge_u = None
                self.edge_v = None
                if self.node == goal:
                    if mark_reached:
                        self.reached = True
                    break
