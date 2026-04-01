"""
llm_behavior_profiler.py

Uses a Gemini LLM to generate quantitative behavior parameters for each
evacuee persona. The LLM reasons from a natural language persona description
to produce a JSON profile that is consumed by batch_runner.py.

Usage:
    python llm_behavior_profiler.py --api-key YOUR_KEY --output agent_profiles.json
    python llm_behavior_profiler.py --output agent_profiles.json  # uses GEMINI_API_KEY env var

Output format (agent_profiles.json):
    {
        "senior_faculty": { "walk_speed_multiplier": 0.65, ... },
        "young_student":  { ... },
        ...
    }
"""

import argparse
import json
import os
import re
import sys

from groq import Groq

# ---------------------------------------------------------------------------
# Persona descriptions (natural language, fed to the LLM)
# ---------------------------------------------------------------------------

PERSONAS = {
    "senior_faculty": (
        "A senior faculty member in their early 60s with mild mobility limitations "
        "(slow gait, occasional knee pain). They are calm under pressure, know the "
        "campus well, and have attended multiple fire drills. They are likely to "
        "follow official instructions and help others around them."
    ),
    "young_student": (
        "An undergraduate student in their early 20s, physically fit and highly "
        "mobile. They are somewhat prone to panic in unexpected emergencies, may "
        "hesitate before following instructions, and are not very familiar with "
        "all shelter locations on campus."
    ),
    "staff_admin": (
        "An administrative staff member in their 30s-40s who works on campus daily "
        "and has received evacuation training. They know the building layouts and "
        "shelter locations well, remain calm, and are likely to assist others and "
        "follow procedures closely."
    ),
    "mobility_impaired": (
        "A person with significant mobility impairment who uses a wheelchair or "
        "requires a walking aid. Their movement is slow and restricted to accessible "
        "paths. They may feel high anxiety during an evacuation and depend heavily "
        "on assistance from others or accessible signage."
    ),
    "visitor": (
        "An external visitor or prospective student who has never been to this campus "
        "before. They have no knowledge of shelter locations, are easily confused by "
        "the campus layout, and are likely to panic or freeze under an emergency. "
        "They will rely on signage and other people for guidance."
    ),
}

# ---------------------------------------------------------------------------
# Prompt template
# ---------------------------------------------------------------------------

SYSTEM_PROMPT = """\
You are an expert in emergency evacuation behavior modeling. Your task is to
translate a natural language description of an evacuee persona into a JSON
object with precise quantitative behavior parameters for an agent-based
evacuation simulation.

The simulation runs on a real campus OSM graph. Each agent walks or drives
to a shelter. The parameters you produce will directly control how the agent
behaves during the simulation.

Parameter definitions:
- walk_speed_multiplier (float, 0.3–1.5): multiplier on the base walking speed
  (1.4 m/step). Values <1.0 = slower, >1.0 = faster.
- compliance_rate (float, 0.0–1.0): probability that the agent follows the
  recommended shelter assignment rather than choosing randomly. 1.0 = always
  follows instructions.
- panic_level (float, 0.0–1.0): degree of panic. Higher panic increases
  observation error and may cause suboptimal decisions.
- observation_error_multiplier (float, 0.5–3.0): multiplier on the base
  observation noise. Higher = less accurate perception of surroundings.
- decision_delay_steps (int, 0–5): number of simulation steps the agent waits
  before starting to move after the evacuation signal.
- shelter_familiarity (float, 0.0–1.0): how well the agent knows the shelter
  locations. 1.0 = knows exactly where all shelters are.

Respond ONLY with a valid JSON object — no explanation, no markdown fences.
Example format:
{
  "walk_speed_multiplier": 0.90,
  "compliance_rate": 0.80,
  "panic_level": 0.25,
  "observation_error_multiplier": 1.10,
  "decision_delay_steps": 1,
  "shelter_familiarity": 0.70
}
"""

USER_PROMPT_TEMPLATE = """\
Persona: {persona_name}
Description: {description}

Generate the behavior parameter JSON for this persona.
"""

# ---------------------------------------------------------------------------
# LLM call
# ---------------------------------------------------------------------------

def _call_groq(client, persona_name: str, description: str) -> dict:
    prompt = USER_PROMPT_TEMPLATE.format(
        persona_name=persona_name,
        description=description,
    )
    response = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": prompt},
        ],
        temperature=0.2,
        max_tokens=256,
    )
    raw = response.choices[0].message.content.strip()
    # Strip markdown fences if the model adds them despite instructions
    raw = re.sub(r"^```[a-zA-Z]*\n?", "", raw)
    raw = re.sub(r"\n?```$", "", raw)
    return json.loads(raw)


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

PARAM_RANGES = {
    "walk_speed_multiplier":    (0.3, 1.5),
    "compliance_rate":          (0.0, 1.0),
    "panic_level":              (0.0, 1.0),
    "observation_error_multiplier": (0.5, 3.0),
    "decision_delay_steps":     (0, 5),
    "shelter_familiarity":      (0.0, 1.0),
}

def _validate_profile(name: str, profile: dict) -> dict:
    for param, (lo, hi) in PARAM_RANGES.items():
        if param not in profile:
            raise ValueError(f"[{name}] missing parameter: {param}")
        val = profile[param]
        if not (lo <= val <= hi):
            raise ValueError(f"[{name}] {param}={val} out of range [{lo}, {hi}]")
    return profile


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def generate_profiles(api_key: str, output_path: str, verbose: bool = True) -> dict:
    client = Groq(api_key=api_key)
    profiles = {}

    for persona_name, description in PERSONAS.items():
        if verbose:
            print(f"[profiler] generating profile for: {persona_name} ...", end=" ", flush=True)
        try:
            profile = _call_groq(client, persona_name, description)
            profile = _validate_profile(persona_name, profile)
            profiles[persona_name] = profile
            if verbose:
                print("ok")
                for k, v in profile.items():
                    print(f"           {k}: {v}")
        except Exception as e:
            print(f"ERROR: {e}")
            sys.exit(1)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(profiles, f, indent=2)

    if verbose:
        print(f"\n[profiler] saved {len(profiles)} profiles -> {output_path}")

    return profiles


def main():
    parser = argparse.ArgumentParser(description="Generate LLM-based agent behavior profiles.")
    parser.add_argument("--api-key", default=os.environ.get("GROQ_API_KEY", ""),
                        help="Groq API key (or set GROQ_API_KEY env var)")
    parser.add_argument("--output", default="agent_profiles.json",
                        help="Output JSON path (default: agent_profiles.json)")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    if not args.api_key:
        print("ERROR: provide --api-key or set GROQ_API_KEY environment variable.")
        sys.exit(1)

    generate_profiles(args.api_key, args.output, verbose=not args.quiet)


if __name__ == "__main__":
    main()
