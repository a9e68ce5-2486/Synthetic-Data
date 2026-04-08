"""
llm_zone_coordinator.py

Layer 2: LLM-based Zone Coordinator using a ReAct (Reason + Act) loop.

The coordinator receives the current evacuation state (zones, shelters,
road conditions) and uses an LLM with tool-calling to produce capacity-aware
zone-to-shelter assignments with explicit reasoning.

Tools available to the LLM:
  get_shelter_status(shelter_id)            → capacity, occupancy, available seats
  get_zone_population(zone_id)              → demand count, vulnerability breakdown
  get_road_conditions(zone_id, shelter_id)  → reachability, avg distance, blocked flag
  get_distance_matrix()                     → all zone×shelter avg distances

Output format matches zone_assignment.py so existing batch_runner infrastructure
can consume LLM coordinator results directly.

Usage (CLI):
    python llm_zone_coordinator.py \\
        --scenario scenarios/enterprise_baseline.json \\
        --api-key YOUR_GROQ_KEY \\
        --output-dir logs/zone_llm

Usage (API):
    from llm_zone_coordinator import LLMZoneCoordinator, build_coordinator_state
    from zone_assignment import _build_zones, _build_ped_population
    from evac_env import EvacEnv

    env = EvacEnv()
    peds = _build_ped_population(env, seed=42)
    zones = _build_zones(env, peds, num_zones=6, seed=42)
    state = build_coordinator_state(zones, env)

    coordinator = LLMZoneCoordinator(api_key="...", verbose=True)
    assignment = coordinator.assign(state, env)
"""

import argparse
import json
import os
import re
import sys

import networkx as nx

import config
from evac_env import EvacEnv
from scenario_loader import load_scenario, temporary_config
from zone_assignment import _build_zones, _build_ped_population, _assign_zone_shelters

# ---------------------------------------------------------------------------
# ReAct system prompt
# ---------------------------------------------------------------------------

SYSTEM_PROMPT = """\
You are an Emergency Evacuation Zone Coordinator for the University of Utah campus.
Your task is to assign each evacuation zone to a primary shelter (and optionally a
backup shelter), taking into account:
  - Shelter remaining capacity
  - Zone population size and vulnerability (mobility-impaired, high-panic individuals)
  - Road conditions and travel distance to each shelter
  - Load balancing: avoid sending everyone to the same shelter

You have access to the following tools. Call them one at a time using this exact format:

  Action: tool_name(argument)

Available tools:
  get_shelter_status(shelter_id)
    → Returns capacity, current occupancy, and available seats for the shelter.

  get_zone_population(zone_id)
    → Returns demand count and population breakdown for a zone.

  get_road_conditions(zone_id, shelter_id)
    → Returns average travel distance, blocked status, and a feasibility score.

  get_distance_matrix()
    → Returns a table of average travel distances from every zone to every shelter.

After each tool call, you will receive an Observation with the result.
Use Thought: lines to reason before acting.

When you have enough information, output your final assignments in this exact format:

  Final Answer:
  {
    "assignments": [
      {
        "zone_id": <int>,
        "primary_shelter": "<shelter_id>",
        "backup_shelter": "<shelter_id or null>",
        "reasoning": "<one sentence>"
      },
      ...
    ]
  }

Rules:
  - Every zone MUST receive a primary_shelter assignment.
  - Prefer shelters with sufficient remaining capacity for the zone's demand.
  - For zones with mobility-impaired or high-panic population, prefer closer shelters.
  - Backup shelter should be different from primary.
  - Output valid JSON — no markdown fences.
"""

# ---------------------------------------------------------------------------
# Tool implementations
# ---------------------------------------------------------------------------

def _tool_get_shelter_status(shelter_id, state):
    sid = str(shelter_id).strip()
    shelters = state["shelters"]
    # Accept integer keys stored as strings
    for k in shelters:
        if str(k) == sid:
            s = shelters[k]
            return {
                "shelter_id": sid,
                "capacity": s["capacity"],
                "occupancy": s["occupancy"],
                "available": max(0, s["capacity"] - s["occupancy"]),
                "load_pct": round(100.0 * s["occupancy"] / max(1, s["capacity"]), 1),
            }
    return {"error": f"shelter '{sid}' not found. Valid IDs: {list(shelters.keys())[:10]}"}


def _tool_get_zone_population(zone_id, state):
    try:
        zid = int(zone_id)
    except (ValueError, TypeError):
        return {"error": f"zone_id must be an integer, got '{zone_id}'"}
    for z in state["zones"]:
        if z["zone_id"] == zid:
            return {
                "zone_id": zid,
                "demand": z["demand"],
                "mobility_impaired_approx": z.get("mobility_impaired_approx", 0),
                "high_panic_approx": z.get("high_panic_approx", 0),
                "center_x": round(z["center_x"], 1),
                "center_y": round(z["center_y"], 1),
            }
    return {"error": f"zone_id '{zone_id}' not found. Valid IDs: {[z['zone_id'] for z in state['zones']]}"}


def _tool_get_road_conditions(zone_id, shelter_id, state):
    try:
        zid = int(zone_id)
    except (ValueError, TypeError):
        return {"error": f"zone_id must be an integer"}
    sid = str(shelter_id).strip()
    zone = next((z for z in state["zones"] if z["zone_id"] == zid), None)
    if zone is None:
        return {"error": f"zone_id '{zone_id}' not found"}
    scores = {str(k): v for k, v in zone["shelter_scores"]}
    if sid not in scores:
        return {"error": f"shelter '{sid}' not found for zone {zone_id}. Available: {list(scores.keys())[:10]}"}
    dist = scores[sid]
    blocked_info = state.get("blocked_shelters", set())
    is_blocked = (sid in blocked_info)
    feasibility = "unreachable" if not isinstance(dist, float) or dist == float("inf") else (
        "blocked" if is_blocked else ("close" if dist < 500 else ("medium" if dist < 1200 else "far"))
    )
    return {
        "zone_id": zid,
        "shelter_id": sid,
        "avg_distance_m": round(dist, 1) if isinstance(dist, float) and dist != float("inf") else "unreachable",
        "is_blocked": is_blocked,
        "feasibility": feasibility,
    }


def _tool_get_distance_matrix(state):
    matrix = {}
    for z in state["zones"]:
        row = {}
        for shelter_id, dist in z["shelter_scores"]:
            row[str(shelter_id)] = round(dist, 1) if dist != float("inf") else "unreachable"
        matrix[str(z["zone_id"])] = row
    return {"distance_matrix": matrix}


def _dispatch_tool(action_str, state):
    """Parse and execute a tool call from the LLM's Action line."""
    action_str = action_str.strip()

    # get_distance_matrix() — no args
    if re.match(r"get_distance_matrix\s*\(\s*\)", action_str):
        return _tool_get_distance_matrix(state)

    # get_shelter_status(shelter_id)
    m = re.match(r"get_shelter_status\s*\(\s*(.+?)\s*\)", action_str)
    if m:
        return _tool_get_shelter_status(m.group(1).strip("'\""), state)

    # get_zone_population(zone_id)
    m = re.match(r"get_zone_population\s*\(\s*(.+?)\s*\)", action_str)
    if m:
        return _tool_get_zone_population(m.group(1).strip("'\""), state)

    # get_road_conditions(zone_id, shelter_id)
    m = re.match(r"get_road_conditions\s*\(\s*(.+?)\s*,\s*(.+?)\s*\)", action_str)
    if m:
        return _tool_get_road_conditions(
            m.group(1).strip("'\""),
            m.group(2).strip("'\""),
            state,
        )

    return {"error": f"Unknown tool call: '{action_str}'"}


# ---------------------------------------------------------------------------
# ReAct loop
# ---------------------------------------------------------------------------

def _run_react_loop(client, state, max_iterations=20, verbose=False):
    """Run the ReAct loop. Returns the parsed assignment list or None on failure."""
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    # Initial user message: brief situation summary
    shelter_ids = list(state["shelters"].keys())
    zone_ids = [z["zone_id"] for z in state["zones"]]
    total_demand = sum(z["demand"] for z in state["zones"])
    total_capacity = sum(s["capacity"] for s in state["shelters"].values())
    user_msg = (
        f"Current evacuation state:\n"
        f"  Zones: {zone_ids} (total demand: {total_demand} people)\n"
        f"  Shelters: {shelter_ids} (total capacity: {total_capacity})\n"
        f"  Disaster type: {state.get('disaster_type', 'unknown')}\n"
        f"  Severity: {state.get('severity', 'unknown')}\n\n"
        f"Please investigate the situation using the tools and produce zone-to-shelter assignments."
    )
    messages.append({"role": "user", "content": user_msg})

    for iteration in range(max_iterations):
        response = client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=messages,
            temperature=0.1,
            max_tokens=1024,
            stop=["Observation:"],
        )
        reply = response.choices[0].message.content.strip()
        messages.append({"role": "assistant", "content": reply})

        if verbose:
            print(f"\n[coordinator] step {iteration + 1}")
            print(reply)

        # Check for Final Answer
        if "Final Answer:" in reply:
            json_text = reply.split("Final Answer:", 1)[1].strip()
            json_text = re.sub(r"^```[a-zA-Z]*\n?", "", json_text)
            json_text = re.sub(r"\n?```$", "", json_text)
            try:
                parsed = json.loads(json_text)
                return parsed.get("assignments", [])
            except json.JSONDecodeError as e:
                if verbose:
                    print(f"[coordinator] JSON parse error: {e}")
                return None

        # Check for Action
        action_match = re.search(r"Action:\s*(.+)", reply)
        if action_match:
            action_str = action_match.group(1).strip()
            result = _dispatch_tool(action_str, state)
            obs_text = f"Observation: {json.dumps(result, indent=2)}"
            if verbose:
                print(obs_text)
            messages.append({"role": "user", "content": obs_text})
        else:
            # No action and no final answer — prompt to continue
            messages.append({
                "role": "user",
                "content": "Please continue. Use a tool (Action: ...) or provide the Final Answer.",
            })

    if verbose:
        print("[coordinator] max iterations reached without Final Answer")
    return None


# ---------------------------------------------------------------------------
# State builder
# ---------------------------------------------------------------------------

def build_coordinator_state(zones, env, disaster_type="unknown", severity="unknown"):
    """Build the state dict consumed by tools and the ReAct loop.

    zones: output of zone_assignment._build_zones()
    env:   EvacEnv instance
    """
    # Shelter info
    shelters = {}
    for s in env.shelters:
        shelters[str(s)] = {
            "capacity": env.shelter_capacity.get(s, 999999),
            "occupancy": env.shelter_occupancy.get(s, 0),
        }

    # Blocked shelters: shelters whose all incoming walk edges are blocked
    blocked_edges = set(getattr(env, "blocked_edges_walk", set()))
    blocked_shelters = set()
    for s in env.shelters:
        if s not in env.G_walk:
            blocked_shelters.add(str(s))
            continue
        in_edges = list(env.G_walk.predecessors(s))
        if in_edges and all((u, s) in blocked_edges for u in in_edges):
            blocked_shelters.add(str(s))

    # Estimate vulnerability counts per zone (rough approximation from agent_profiles)
    try:
        import json as _json
        _profiles_path = os.path.join(os.path.dirname(__file__), "agent_profiles.json")
        with open(_profiles_path) as _f:
            _profiles = _json.load(_f)
        _mobility_impaired_personas = {
            k for k, v in _profiles.items()
            if float(v.get("walk_speed_multiplier", 1.0)) <= 0.35
        }
        _high_panic_personas = {
            k for k, v in _profiles.items()
            if float(v.get("panic_level", 0.0)) >= 0.70
        }
    except Exception:
        _mobility_impaired_personas = set()
        _high_panic_personas = set()

    zone_data = []
    for z in zones:
        demand = z["demand"]
        # Rough estimates: 5% mobility-impaired, 15% high-panic (visitor/freshman mix)
        zone_data.append({
            "zone_id": z["zone_id"],
            "demand": demand,
            "mobility_impaired_approx": max(0, round(demand * 0.05)),
            "high_panic_approx": max(0, round(demand * 0.15)),
            "center_x": z["center_x"],
            "center_y": z["center_y"],
            "shelter_scores": z["shelter_scores"],
        })

    return {
        "zones": zone_data,
        "shelters": shelters,
        "blocked_shelters": blocked_shelters,
        "disaster_type": disaster_type,
        "severity": severity,
    }


# ---------------------------------------------------------------------------
# Main coordinator class
# ---------------------------------------------------------------------------

class LLMZoneCoordinator:
    """LLM-based zone coordinator using a ReAct tool loop (Layer 2).

    Falls back to the algorithmic assignment from zone_assignment.py if
    the LLM fails (API error, malformed output, or no Groq key).
    """

    def __init__(self, api_key=None, verbose=False, max_iterations=20):
        self.verbose = verbose
        self.max_iterations = max_iterations
        self._client = None

        key = api_key or os.environ.get("GROQ_API_KEY", "")
        if key:
            try:
                from groq import Groq
                self._client = Groq(api_key=key)
            except ImportError:
                if verbose:
                    print("[coordinator] groq package not installed — using algorithmic fallback")
        elif verbose:
            print("[coordinator] no GROQ_API_KEY — using algorithmic fallback")

    def assign(self, state, env, zones_raw):
        """Run the coordinator and return annotated zones list.

        Parameters
        ----------
        state     : output of build_coordinator_state()
        env       : EvacEnv instance
        zones_raw : original zones list from _build_zones() (will be annotated in place)

        Returns
        -------
        zones_raw with 'primary_shelter', 'backup_shelter', 'llm_reasoning' fields set.
        source: "llm" or "algorithmic"
        """
        if self._client is not None:
            assignments = self._run_llm(state)
        else:
            assignments = None

        if assignments:
            return self._apply_llm_assignments(assignments, zones_raw, env), "llm"
        else:
            if self.verbose:
                print("[coordinator] using algorithmic fallback assignment")
            _assign_zone_shelters(zones_raw, list(env.shelters), env)
            for z in zones_raw:
                z.setdefault("llm_reasoning", "algorithmic fallback")
            return zones_raw, "algorithmic"

    def _run_llm(self, state):
        try:
            return _run_react_loop(
                self._client, state,
                max_iterations=self.max_iterations,
                verbose=self.verbose,
            )
        except Exception as e:
            if self.verbose:
                print(f"[coordinator] LLM error: {e}")
            return None

    def _apply_llm_assignments(self, assignments, zones_raw, env):
        """Map LLM-produced assignment list back to zones_raw format."""
        shelters_set = set(str(s) for s in env.shelters)
        # Build lookup: str(zone_id) → assignment
        llm_map = {str(a["zone_id"]): a for a in assignments}

        for z in zones_raw:
            zid = str(z["zone_id"])
            if zid not in llm_map:
                if self.verbose:
                    print(f"[coordinator] no LLM assignment for zone {zid}, using nearest")
                # Fallback: pick nearest shelter for this zone
                scores = z.get("shelter_scores", [])
                primary = min(scores, key=lambda kv: kv[1])[0] if scores else None
                backup = None
                if scores and len(scores) > 1:
                    backup = sorted(scores, key=lambda kv: kv[1])[1][0]
                z["primary_shelter"] = primary
                z["backup_shelter"] = backup
                z["llm_reasoning"] = "fallback: zone not covered by LLM output"
                continue

            a = llm_map[zid]
            # Validate shelter IDs — fall back to algorithmic if LLM hallucinated a node
            primary_str = str(a.get("primary_shelter", ""))
            backup_str = str(a.get("backup_shelter", "")) if a.get("backup_shelter") else None

            if primary_str not in shelters_set:
                if self.verbose:
                    print(f"[coordinator] LLM gave invalid primary shelter '{primary_str}' for zone {zid}")
                scores = z.get("shelter_scores", [])
                primary = min(scores, key=lambda kv: kv[1])[0] if scores else None
            else:
                # Convert back to original node type (int if possible)
                try:
                    primary = int(primary_str)
                except ValueError:
                    primary = primary_str

            if backup_str and backup_str in shelters_set and backup_str != primary_str:
                try:
                    backup = int(backup_str)
                except ValueError:
                    backup = backup_str
            else:
                scores = z.get("shelter_scores", [])
                valid = [k for k, _ in sorted(scores, key=lambda kv: kv[1]) if k != primary]
                backup = valid[0] if valid else None

            z["primary_shelter"] = primary
            z["backup_shelter"] = backup
            z["llm_reasoning"] = a.get("reasoning", "")

        return zones_raw


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LLM Zone Coordinator (Layer 2)")
    parser.add_argument("--scenario", default="scenarios/enterprise_baseline.json")
    parser.add_argument("--api-key", default="", help="Groq API key (or set GROQ_API_KEY env var)")
    parser.add_argument("--seed", type=int, default=20260323)
    parser.add_argument("--num-zones", type=int, default=6)
    parser.add_argument("--output-dir", default="logs/zone_llm")
    parser.add_argument("--disaster-type", default="blizzard")
    parser.add_argument("--severity", default="moderate")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    os.makedirs(args.output_dir, exist_ok=True)

    with temporary_config(scenario.get("config_overrides", {})):
        import random, numpy as np
        random.seed(args.seed)
        np.random.seed(args.seed)

        env = EvacEnv()
        peds = _build_ped_population(env, seed=args.seed)
        zones = _build_zones(env, peds, num_zones=args.num_zones, seed=args.seed)

        state = build_coordinator_state(
            zones, env,
            disaster_type=args.disaster_type,
            severity=args.severity,
        )

    coordinator = LLMZoneCoordinator(
        api_key=args.api_key or os.environ.get("GROQ_API_KEY", ""),
        verbose=args.verbose,
    )
    zones_annotated, source = coordinator.assign(state, env, zones)

    # Serialize results
    def _ser(v):
        import numpy as np
        if isinstance(v, (np.integer,)):
            return int(v)
        if isinstance(v, (np.floating,)):
            return float(v)
        return v

    result = {
        "scenario": scenario.get("name", "scenario"),
        "seed": args.seed,
        "num_zones": args.num_zones,
        "ped_count": len(peds),
        "assignment_source": source,
        "disaster_type": args.disaster_type,
        "severity": args.severity,
        "zones": [
            {
                "zone_id": int(z["zone_id"]),
                "demand": int(z["demand"]),
                "primary_shelter": _ser(z.get("primary_shelter")),
                "backup_shelter": _ser(z.get("backup_shelter")),
                "llm_reasoning": z.get("llm_reasoning", ""),
            }
            for z in zones_annotated
        ],
    }

    out_path = os.path.join(args.output_dir, "zone_llm_assignment.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    print(f"\n[coordinator] assignment source: {source}")
    print(f"[coordinator] saved -> {out_path}\n")
    for z in result["zones"]:
        print(
            f"  Zone {z['zone_id']:2d} ({z['demand']:3d} people) → "
            f"primary: {z['primary_shelter']}  "
            f"backup: {z['backup_shelter']}  "
            f"| {z['llm_reasoning']}"
        )


if __name__ == "__main__":
    main()
