"""
personal_advisor.py

Personal Evacuation Advisor — end-to-end three-layer pipeline:

  Layer 1: LLM behavior profiling
    User describes themselves in natural language.
    LLM (Llama 3.3 70B) generates a quantitative behavior profile.

  Layer 3: DRQN navigation
    Profile is applied to a pedestrian agent.
    DRQN finds the best route from the user's current location to a shelter.

  Output layer: LLM recommendation
    LLM translates route data + behavior profile into a plain-language
    personalized evacuation recommendation.

Usage (interactive):
    python personal_advisor.py \\
        --checkpoint logs/drqn_torch_best.pt \\
        --scenario scenarios/enterprise_baseline.json

Usage (non-interactive / scripted):
    python personal_advisor.py \\
        --checkpoint logs/drqn_torch_best.pt \\
        --scenario scenarios/enterprise_baseline.json \\
        --description "I am a first-year international student. \\
                       This is my first time on campus and I don't know \\
                       where any emergency shelters are." \\
        --start-node 1234567890 \\
        --disaster-type blizzard --severity moderate \\
        --output-dir logs/personal_advisor
"""

import argparse
import json
import os
import random
import re
import sys

import networkx as nx
import numpy as np

import config
from agents.ped_agent import PedAgent
from batch_runner import _apply_persona, _AGENT_PROFILES
from evac_env import EvacEnv
from policy import select_goal
from route_recommendation import extract_route
from scenario_loader import load_scenario, temporary_config

# ---------------------------------------------------------------------------
# Layer 1: Behavior profiling from natural language
# ---------------------------------------------------------------------------

_PROFILER_SYSTEM_PROMPT = """\
You are a behavioral scientist helping model emergency evacuee behavior.

Given a natural language description of a person on the University of Utah campus,
generate quantitative behavior parameters that describe how they would behave during
an evacuation.

Parameters to generate:
- walk_speed_multiplier (float, 0.3–1.5): speed relative to average adult walking pace.
  0.3 = very slow (wheelchair/severe injury), 1.0 = normal, 1.4 = very fast (athlete).
- compliance_rate (float, 0.0–1.0): probability of following official shelter assignments.
  0.0 = never follows instructions, 1.0 = always follows.
- panic_level (float, 0.0–1.0): degree of panic and anxiety during emergency.
  0.0 = completely calm, 1.0 = extreme panic.
- observation_error_multiplier (float, 0.5–3.0): how much the person misreads their
  environment. 0.5 = very accurate, 1.0 = normal, 3.0 = highly confused.
- decision_delay_steps (int, 0–5): how many simulation steps the person waits before
  starting to move after the evacuation signal.
- shelter_familiarity (float, 0.0–1.0): how well the person knows shelter locations.
  0.0 = knows none, 1.0 = knows all shelters exactly.

Respond ONLY with a valid JSON object — no explanation, no markdown fences.
Example:
{
  "walk_speed_multiplier": 1.10,
  "compliance_rate": 0.40,
  "panic_level": 0.70,
  "observation_error_multiplier": 2.20,
  "decision_delay_steps": 2,
  "shelter_familiarity": 0.15
}
"""

def _profile_from_description(client, description: str) -> dict:
    """Call LLM to produce a behavior profile from user's natural language description."""
    response = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[
            {"role": "system", "content": _PROFILER_SYSTEM_PROMPT},
            {"role": "user", "content": f"Person description:\n{description}"},
        ],
        temperature=0.15,
        max_tokens=256,
    )
    raw = response.choices[0].message.content.strip()
    raw = re.sub(r"^```[a-zA-Z]*\n?", "", raw)
    raw = re.sub(r"\n?```$", "", raw)
    return json.loads(raw)


def _validate_profile(profile: dict) -> dict:
    defaults = {
        "walk_speed_multiplier": 1.0,
        "compliance_rate": 0.8,
        "panic_level": 0.3,
        "observation_error_multiplier": 1.0,
        "decision_delay_steps": 1,
        "shelter_familiarity": 0.5,
    }
    clamps = {
        "walk_speed_multiplier": (0.3, 1.5),
        "compliance_rate": (0.0, 1.0),
        "panic_level": (0.0, 1.0),
        "observation_error_multiplier": (0.5, 3.0),
        "decision_delay_steps": (0, 5),
        "shelter_familiarity": (0.0, 1.0),
    }
    validated = {}
    for k, (lo, hi) in clamps.items():
        v = profile.get(k, defaults[k])
        validated[k] = max(lo, min(hi, float(v) if k != "decision_delay_steps" else int(v)))
    return validated


# ---------------------------------------------------------------------------
# Layer 3: Apply profile and run DRQN
# ---------------------------------------------------------------------------

def _apply_profile_to_agent(agent, profile: dict):
    """Wire behavior profile fields onto a PedAgent (mirrors _apply_persona logic)."""
    panic = float(profile.get("panic_level", 0.0))
    agent.persona = "user_generated"
    agent.speed_multiplier = float(profile.get("walk_speed_multiplier", 1.0))
    agent.panic_level = panic
    agent.decision_delay_steps = int(profile.get("decision_delay_steps", 0))
    agent.shelter_familiarity = float(profile.get("shelter_familiarity", 1.0))
    # Apply panic modulation (same as batch_runner._apply_persona)
    agent.compliance_rate = float(profile.get("compliance_rate", 1.0)) * (1.0 - 0.5 * panic)
    agent.observation_error_multiplier = float(profile.get("observation_error_multiplier", 1.0)) * (1.0 + panic)


def _run_drqn_route(env, checkpoint_path, start_node, profile, seed, device, max_neighbors):
    """Run DRQN route extraction with the user's behavior profile applied."""
    random.seed(seed)
    np.random.seed(seed)

    ped = PedAgent(1, start_node, env)
    ped.role = "student"
    _apply_profile_to_agent(ped, profile)

    # Familiarity-filtered shelter list
    shelters_all = list(env.shelters)
    familiarity = ped.shelter_familiarity
    if familiarity < 1.0 and len(shelters_all) > 1:
        n_known = max(1, round(familiarity * len(shelters_all)))
        rng = random.Random(42)
        shuffled = list(shelters_all)
        rng.shuffle(shuffled)
        shelters_known = shuffled[:n_known]
    else:
        shelters_known = shelters_all

    goal = select_goal(ped, env.G_walk, shelters_known, "nearest", {})

    return extract_route(
        env=env,
        checkpoint_path=checkpoint_path,
        start_node=start_node,
        role="student",
        seed=seed,
        device=device,
        max_neighbors=max_neighbors,
        goal_override=goal,
    )


# ---------------------------------------------------------------------------
# Output layer: LLM natural language recommendation
# ---------------------------------------------------------------------------

_ADVISOR_SYSTEM_PROMPT = """\
You are a personal emergency evacuation advisor for the University of Utah campus.

Given information about a person and their computed evacuation route, write a
clear, calm, and encouraging personalized evacuation recommendation. The tone
should be direct and reassuring — not alarming.

RULE 1 — Guidance detail level scales with shelter_familiarity:

  familiarity = high:  Brief directions. The person knows the campus layout.
  familiarity = moderate: Key landmarks and main decision points.
  familiarity = low/none: Full step-by-step: every turn, signs to look for,
    what to do if confused, who to ask. Assume they have never been to a shelter.

RULE 2 — Road blockage information is ALWAYS required, regardless of familiarity:

  If route_replans > 0 OR severity is severe/extreme:
    → Always include a blocked-road warning and a concrete alternative:
      "Your usual/direct route may be blocked. If you find [path] blocked,
       turn [direction] and use [alternative landmark/street] instead."
    → Even a senior faculty member who knows campus well needs this —
      a familiar route being blocked is exactly when people get confused.
    → The alternative should be described with enough detail for someone
      under stress to follow without a map.

Structure your response as:
1. One sentence acknowledging their situation.
2. The shelter destination and why it suits them.
3. Primary movement guidance (detail level per Rule 1).
4. If route_replans > 0 or severity is severe/extreme: blocked-road warning
   + alternative route (always include, regardless of familiarity).
5. If panic is high (≥ 0.5): one short calming tip.
6. One closing reassurance sentence.

Do NOT mention node IDs, simulation steps, or technical details.
Write in English. Total response: 120–250 words.
"""

def _generate_recommendation(client, description: str, profile: dict, route: dict,
                               disaster_type: str, severity: str) -> str:
    """Call LLM to produce a natural language recommendation."""
    # Build a human-readable route summary (no node IDs)
    replan_count = route["replan_count"]
    high_severity = severity in ("severe", "extreme")
    route_summary = {
        "reached_shelter": route["reached"],
        "estimated_walking_time_minutes": round(route["steps"] * 5 / 60, 1),
        "route_replans_due_to_blockages": replan_count,
        "road_blockages_encountered": replan_count > 0,
        "blockage_warning_required": replan_count > 0 or high_severity,
        "severity_note": (
            "Roads may be partially blocked — always include alternative route guidance."
            if high_severity or replan_count > 0 else
            "Conditions relatively clear, but mention that routes could change."
        ),
    }

    panic = float(profile.get("panic_level", 0.0))
    familiarity = float(profile.get("shelter_familiarity", 0.5))
    speed = float(profile.get("walk_speed_multiplier", 1.0))
    compliance = float(profile.get("compliance_rate", 1.0)) * (1.0 - 0.5 * panic)
    delay = int(profile.get("decision_delay_steps", 0))

    if familiarity <= 0.15:
        familiarity_label = "none — does not know where any shelters are (give DETAILED step-by-step guidance)"
    elif familiarity <= 0.35:
        familiarity_label = "low — knows 1–2 shelters but not their exact routes (give DETAILED guidance with landmarks)"
    elif familiarity <= 0.65:
        familiarity_label = "moderate — knows some shelters (give key landmarks and decision points)"
    else:
        familiarity_label = "high — knows most shelters well (give brief high-level directions only)"

    profile_summary = {
        "walking_speed": "very slow" if speed <= 0.4 else "slow" if speed <= 0.7 else "normal" if speed <= 1.1 else "fast",
        "panic_level": "very high" if panic >= 0.75 else "high" if panic >= 0.50 else "moderate" if panic >= 0.25 else "low",
        "shelter_familiarity": familiarity_label,
        "likely_to_follow_instructions": compliance >= 0.6,
        "reaction_delay_seconds": delay * 5,  # Each step ≈ 5 seconds
        "guidance_detail_required": (
            "DETAILED step-by-step" if familiarity <= 0.35 else
            "MODERATE (landmarks + decision points)" if familiarity <= 0.65 else
            "BRIEF high-level only"
        ),
    }

    disaster_desc = {
        "blizzard": f"a {severity} blizzard with snow and possible road blockages",
        "earthquake": f"a {severity} earthquake with possible road damage",
        "compound": f"a compound {severity} disaster (blizzard + earthquake)",
    }.get(disaster_type, f"a {severity} {disaster_type} emergency")

    user_context = (
        f"Person description: {description}\n\n"
        f"Disaster situation: {disaster_desc}\n\n"
        f"Behavioral profile:\n{json.dumps(profile_summary, indent=2)}\n\n"
        f"Route analysis:\n{json.dumps(route_summary, indent=2)}"
    )

    response = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[
            {"role": "system", "content": _ADVISOR_SYSTEM_PROMPT},
            {"role": "user", "content": user_context},
        ],
        temperature=0.3,
        max_tokens=400,
    )
    return response.choices[0].message.content.strip()


# ---------------------------------------------------------------------------
# Main advisor class
# ---------------------------------------------------------------------------

class PersonalAdvisor:
    """End-to-end personal evacuation advisor.

    Combines:
      - LLM behavior profiling (Layer 1)
      - DRQN route extraction (Layer 3)
      - LLM natural language recommendation (output layer)
    """

    def __init__(self, api_key=None, checkpoint_path=None, device="auto",
                 max_neighbors=None, verbose=False):
        self.checkpoint_path = checkpoint_path
        self.device = device
        self.max_neighbors = max_neighbors
        self.verbose = verbose
        self._client = None

        key = api_key or os.environ.get("GROQ_API_KEY", "")
        if key:
            try:
                from groq import Groq
                self._client = Groq(api_key=key)
            except ImportError:
                print("[advisor] groq not installed — LLM steps will be skipped")
        else:
            print("[advisor] no GROQ_API_KEY — LLM steps will be skipped")

    def advise(self, description: str, env, start_node, seed=20260323,
               disaster_type="unknown", severity="unknown") -> dict:
        """Run the full pipeline and return a result dict.

        Returns
        -------
        dict with keys:
          profile       : behavior profile dict
          route         : route result from extract_route()
          recommendation: natural language advice string
        """
        # --- Layer 1: Profile from description ---
        if self._client is not None and description:
            if self.verbose:
                print("[advisor] generating behavior profile from description...")
            try:
                profile = _validate_profile(_profile_from_description(self._client, description))
                if self.verbose:
                    print(f"[advisor] profile: {json.dumps(profile, indent=2)}")
            except Exception as e:
                if self.verbose:
                    print(f"[advisor] profile generation failed ({e}), using defaults")
                profile = _validate_profile({})
        else:
            if self.verbose:
                print("[advisor] no LLM available — using default profile")
            profile = _validate_profile({})

        # --- Layer 3: DRQN route ---
        if self.verbose:
            print("[advisor] running DRQN route extraction...")
        if not self.checkpoint_path or not os.path.exists(self.checkpoint_path):
            raise FileNotFoundError(
                f"DRQN checkpoint not found: {self.checkpoint_path}. "
                "Pass --checkpoint <path> or set checkpoint_path."
            )
        route = _run_drqn_route(
            env=env,
            checkpoint_path=self.checkpoint_path,
            start_node=start_node,
            profile=profile,
            seed=seed,
            device=self.device,
            max_neighbors=self.max_neighbors,
        )
        if self.verbose:
            print(f"[advisor] route: reached={route['reached']}, steps={route['steps']}, "
                  f"exposure={route['exposure']:.2f}")

        # --- Output layer: LLM recommendation ---
        if self._client is not None and description:
            if self.verbose:
                print("[advisor] generating natural language recommendation...")
            try:
                recommendation = _generate_recommendation(
                    self._client, description, profile, route, disaster_type, severity
                )
            except Exception as e:
                if self.verbose:
                    print(f"[advisor] recommendation generation failed ({e})")
                recommendation = _fallback_recommendation(profile, route, disaster_type, severity)
        else:
            recommendation = _fallback_recommendation(profile, route, disaster_type, severity)

        return {
            "description": description,
            "profile": profile,
            "route": route,
            "recommendation": recommendation,
            "disaster_type": disaster_type,
            "severity": severity,
        }


def _fallback_recommendation(profile, route, disaster_type, severity):
    """Simple rule-based recommendation when LLM is unavailable."""
    reached = route["reached"]
    steps = route["steps"]
    familiarity = float(profile.get("shelter_familiarity", 0.5))
    panic = float(profile.get("panic_level", 0.3))

    lines = [f"Evacuation advisory — {severity} {disaster_type} in progress."]
    if reached:
        lines.append(f"A shelter route has been identified ({steps} steps estimated).")
    else:
        lines.append("A route to a nearby shelter has been planned.")

    if familiarity < 0.3:
        lines.append("Since you may not know where shelters are: follow posted evacuation signs or ask campus security.")
    if panic >= 0.6:
        lines.append("Take a slow breath, look for exit signs, and follow the crowd toward open outdoor spaces.")
    lines.append("Move steadily toward the nearest shelter and follow all official instructions.")
    return " ".join(lines)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Personal Evacuation Advisor")
    parser.add_argument("--scenario", default="scenarios/enterprise_baseline.json")
    parser.add_argument("--checkpoint", default="logs/drqn_torch_best.pt",
                        help="Path to DRQN checkpoint (.pt)")
    parser.add_argument("--description", default="",
                        help="Natural language description of yourself (person evacuating)")
    parser.add_argument("--start-node", type=str, required=True,
                        help="OSM node ID of your current location")
    parser.add_argument("--api-key", default="")
    parser.add_argument("--seed", type=int, default=20260323)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-neighbors", type=int, default=None)
    parser.add_argument("--disaster-type", default="blizzard",
                        choices=["blizzard", "earthquake", "compound"])
    parser.add_argument("--severity", default="moderate",
                        choices=["light", "moderate", "severe", "extreme"])
    parser.add_argument("--output-dir", default="logs/personal_advisor")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--interactive", action="store_true",
                        help="Prompt for description interactively if --description is empty")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    os.makedirs(args.output_dir, exist_ok=True)

    description = args.description.strip()
    if not description and args.interactive:
        print("\n=== Personal Evacuation Advisor ===")
        print("Please describe yourself so I can personalize your evacuation plan.")
        print("Example: 'I am a freshman student, this is my first week on campus'")
        print("         'I have a mobility impairment and use a cane'")
        print("         'I am a staff member who has worked here for 5 years'\n")
        description = input("Your description: ").strip()

    try:
        start_node = int(args.start_node)
    except ValueError:
        start_node = args.start_node

    with temporary_config(scenario.get("config_overrides", {})):
        env = EvacEnv()
        if start_node not in env.G_walk:
            # Find nearest walk node
            if env.pos and start_node not in env.pos:
                print(f"[advisor] Warning: node {start_node} not in walk graph. "
                      f"Picking a random reachable node.")
                start_node = random.choice(list(env.G_walk.nodes()))
            else:
                raise ValueError(f"Start node {start_node} not found in walk graph.")

        advisor = PersonalAdvisor(
            api_key=args.api_key or os.environ.get("GROQ_API_KEY", ""),
            checkpoint_path=args.checkpoint,
            device=args.device,
            max_neighbors=args.max_neighbors,
            verbose=args.verbose,
        )

        result = advisor.advise(
            description=description,
            env=env,
            start_node=start_node,
            seed=args.seed,
            disaster_type=args.disaster_type,
            severity=args.severity,
        )

    # Output
    print("\n" + "=" * 60)
    print("PERSONAL EVACUATION RECOMMENDATION")
    print("=" * 60)
    print(result["recommendation"])
    print("=" * 60)
    print(f"\nBehavior profile inferred from your description:")
    for k, v in result["profile"].items():
        print(f"  {k}: {v}")
    print(f"\nRoute: {'reached shelter' if result['route']['reached'] else 'route planned'} "
          f"in {result['route']['steps']} steps "
          f"(exposure: {result['route']['exposure']:.2f})")

    # Save results
    stem = f"{scenario.get('name', 'scenario')}_{start_node}"
    out_path = os.path.join(args.output_dir, f"{stem}_advice.json")
    save_result = {
        "scenario": scenario.get("name", "scenario"),
        "start_node": int(start_node) if isinstance(start_node, (int, np.integer)) else str(start_node),
        "disaster_type": args.disaster_type,
        "severity": args.severity,
        "description": description,
        "profile": result["profile"],
        "route": {
            k: v for k, v in result["route"].items()
            if k not in ("path_nodes", "traversed_edges", "target_history")
        },
        "recommendation": result["recommendation"],
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(save_result, f, indent=2, ensure_ascii=False)
    print(f"\n[advisor] saved -> {out_path}")


if __name__ == "__main__":
    main()
