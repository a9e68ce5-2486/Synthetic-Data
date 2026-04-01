import networkx as nx


SUPPORTED_POLICIES = {
    "round_robin",
    "nearest",
    "priority_faculty",
    "drqn",
}


def _safe_shortest_length(graph, start, goal):
    try:
        return nx.shortest_path_length(graph, source=start, target=goal, weight="weight")
    except Exception:
        return float("inf")


def _nearest_goal(agent, graph, shelters):
    if not shelters:
        return agent.node
    best = shelters[0]
    best_dist = float("inf")
    for s in shelters:
        d = _safe_shortest_length(graph, agent.node, s)
        if d < best_dist:
            best_dist = d
            best = s
    return best


def select_goal(agent, graph, shelters, policy_name, assignment_state):
    if not shelters:
        return agent.node
    if policy_name == "nearest":
        return _nearest_goal(agent, graph, shelters)
    if policy_name == "priority_faculty":
        if agent.role == "faculty":
            return _nearest_goal(agent, graph, shelters)
        # Staff are balanced to reduce congestion around popular shelters.
        least_loaded = min(shelters, key=lambda s: assignment_state.get(s, 0))
        return least_loaded
    if policy_name == "drqn":
        # DRQN currently controls step-level pedestrian movement; shelter target
        # is initialized with nearest shelter for stable endpoint semantics.
        return _nearest_goal(agent, graph, shelters)
    # Default: stable deterministic baseline.
    idx = (agent.id - 1) % len(shelters)
    return shelters[idx]
