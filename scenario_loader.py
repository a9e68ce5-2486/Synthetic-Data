import json
import os
from contextlib import contextmanager

import config

# ---------------------------------------------------------------------------
# LLM-generated severity presets (loaded at import time if file exists)
# File is produced by llm_scenario_generator.py
# Falls back to hard-coded tables below if not found.
# ---------------------------------------------------------------------------

_LLM_PRESETS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "scenarios", "llm_severity_presets.json",
)
_LLM_SEVERITY_PRESETS: dict = {}

def _load_llm_presets():
    global _LLM_SEVERITY_PRESETS
    if os.path.exists(_LLM_PRESETS_PATH):
        try:
            with open(_LLM_PRESETS_PATH, "r", encoding="utf-8") as f:
                _LLM_SEVERITY_PRESETS = json.load(f)
            print(f"[scenario_loader] loaded LLM severity presets from {_LLM_PRESETS_PATH}")
        except Exception as e:
            print(f"[scenario_loader] WARNING: could not load LLM presets ({e}), using defaults")
            _LLM_SEVERITY_PRESETS = {}

_load_llm_presets()


DEFAULT_SCENARIO = {
    "name": "enterprise_baseline",
    "description": "Enterprise-oriented evacuation baseline scenario.",
    "disaster_type": "custom",
    "baseline_policy": "round_robin",
    "num_runs": 20,
    "base_seed": 42,
    "policies": ["round_robin", "nearest"],
    "profile": None,
    "config_overrides": {},
}


REQUIRED_TOP_LEVEL_KEYS = ("name", "num_runs", "base_seed", "policies", "config_overrides")


BLIZZARD_SEVERITY_OVERRIDES = {
    "light": {
        "EVAC_BLOCK_PROB": 0.05,
        "EVAC_BLOCK_INIT_PROB": 0.00,
        "EVAC_SNOW_MIN": 0.10,
        "EVAC_SNOW_MAX": 0.35,
        "EVAC_SNOW_ALPHA": 0.9,
        "EVAC_SNOW_ACCUM_PER_STEP": 0.0010,
        "EVAC_SNOW_ACCUM_NOISE": 0.0005,
        "EVAC_BLOCK_FROM_SNOW_THRESHOLD": 0.92,
        "EVAC_BLOCK_FROM_SNOW_PROB": 0.0010,
        "EVAC_OBS_ERROR_WALK": 0.10,
        "EVAC_OBS_ERROR_DRIVE": 0.05,
    },
    "moderate": {
        "EVAC_BLOCK_PROB": 0.10,
        "EVAC_BLOCK_INIT_PROB": 0.02,
        "EVAC_SNOW_MIN": 0.20,
        "EVAC_SNOW_MAX": 0.60,
        "EVAC_SNOW_ALPHA": 1.2,
        "EVAC_SNOW_ACCUM_PER_STEP": 0.0020,
        "EVAC_SNOW_ACCUM_NOISE": 0.0008,
        "EVAC_BLOCK_FROM_SNOW_THRESHOLD": 0.82,
        "EVAC_BLOCK_FROM_SNOW_PROB": 0.0020,
        "EVAC_OBS_ERROR_WALK": 0.15,
        "EVAC_OBS_ERROR_DRIVE": 0.08,
    },
    "severe": {
        "EVAC_BLOCK_PROB": 0.14,
        "EVAC_BLOCK_INIT_PROB": 0.06,
        "EVAC_SNOW_MIN": 0.35,
        "EVAC_SNOW_MAX": 0.90,
        "EVAC_SNOW_ALPHA": 1.5,
        "EVAC_SNOW_ACCUM_PER_STEP": 0.0030,
        "EVAC_SNOW_ACCUM_NOISE": 0.0010,
        "EVAC_BLOCK_FROM_SNOW_THRESHOLD": 0.72,
        "EVAC_BLOCK_FROM_SNOW_PROB": 0.0040,
        "EVAC_OBS_ERROR_WALK": 0.20,
        "EVAC_OBS_ERROR_DRIVE": 0.10,
    },
    "extreme": {
        "EVAC_BLOCK_PROB": 0.20,
        "EVAC_BLOCK_INIT_PROB": 0.07,
        "EVAC_SNOW_MIN": 0.50,
        "EVAC_SNOW_MAX": 1.00,
        "EVAC_SNOW_ALPHA": 1.8,
        "EVAC_SNOW_ACCUM_PER_STEP": 0.0045,
        "EVAC_SNOW_ACCUM_NOISE": 0.0015,
        "EVAC_BLOCK_FROM_SNOW_THRESHOLD": 0.60,
        "EVAC_BLOCK_FROM_SNOW_PROB": 0.0070,
        "EVAC_OBS_ERROR_WALK": 0.25,
        "EVAC_OBS_ERROR_DRIVE": 0.15,
    },
}


EARTHQUAKE_SEVERITY_OVERRIDES = {
    "light": {
        "earthquake_magnitude": 5.8,
        "EVAC_BLOCK_PROB": 0.10,
        "EVAC_BLOCK_INIT_PROB": 0.08,
        "EVAC_SNOW_MIN": 0.00,
        "EVAC_SNOW_MAX": 0.05,
        "EVAC_SNOW_ALPHA": 0.10,
        "EVAC_OBS_ERROR_WALK": 0.08,
        "EVAC_OBS_ERROR_DRIVE": 0.10,
    },
    "moderate": {
        "earthquake_magnitude": 6.8,
        "EVAC_BLOCK_PROB": 0.16,
        "EVAC_BLOCK_INIT_PROB": 0.18,
        "EVAC_SNOW_MIN": 0.00,
        "EVAC_SNOW_MAX": 0.08,
        "EVAC_SNOW_ALPHA": 0.15,
        "EVAC_OBS_ERROR_WALK": 0.12,
        "EVAC_OBS_ERROR_DRIVE": 0.15,
    },
    "severe": {
        "earthquake_magnitude": 7.8,
        "EVAC_BLOCK_PROB": 0.22,
        "EVAC_BLOCK_INIT_PROB": 0.35,
        "EVAC_SNOW_MIN": 0.00,
        "EVAC_SNOW_MAX": 0.10,
        "EVAC_SNOW_ALPHA": 0.20,
        "EVAC_OBS_ERROR_WALK": 0.15,
        "EVAC_OBS_ERROR_DRIVE": 0.20,
    },
    "extreme": {
        "earthquake_magnitude": 8.5,
        "EVAC_BLOCK_PROB": 0.30,
        "EVAC_BLOCK_INIT_PROB": 0.55,
        "EVAC_SNOW_MIN": 0.00,
        "EVAC_SNOW_MAX": 0.12,
        "EVAC_SNOW_ALPHA": 0.25,
        "EVAC_OBS_ERROR_WALK": 0.20,
        "EVAC_OBS_ERROR_DRIVE": 0.25,
    },
}


COMPOUND_SEVERITY_OVERRIDES = {
    "light": {
        **BLIZZARD_SEVERITY_OVERRIDES["light"],
        "EVAC_BLOCK_INIT_PROB": max(
            BLIZZARD_SEVERITY_OVERRIDES["light"]["EVAC_BLOCK_INIT_PROB"],
            EARTHQUAKE_SEVERITY_OVERRIDES["light"]["EVAC_BLOCK_INIT_PROB"],
        ),
        "earthquake_magnitude": EARTHQUAKE_SEVERITY_OVERRIDES["light"]["earthquake_magnitude"],
    },
    "moderate": {
        **BLIZZARD_SEVERITY_OVERRIDES["moderate"],
        "EVAC_BLOCK_INIT_PROB": max(
            BLIZZARD_SEVERITY_OVERRIDES["moderate"]["EVAC_BLOCK_INIT_PROB"],
            EARTHQUAKE_SEVERITY_OVERRIDES["moderate"]["EVAC_BLOCK_INIT_PROB"],
        ),
        "earthquake_magnitude": EARTHQUAKE_SEVERITY_OVERRIDES["moderate"]["earthquake_magnitude"],
    },
    "severe": {
        **BLIZZARD_SEVERITY_OVERRIDES["severe"],
        "EVAC_BLOCK_INIT_PROB": max(
            BLIZZARD_SEVERITY_OVERRIDES["severe"]["EVAC_BLOCK_INIT_PROB"],
            EARTHQUAKE_SEVERITY_OVERRIDES["severe"]["EVAC_BLOCK_INIT_PROB"],
        ),
        "earthquake_magnitude": EARTHQUAKE_SEVERITY_OVERRIDES["severe"]["earthquake_magnitude"],
    },
    "extreme": {
        **BLIZZARD_SEVERITY_OVERRIDES["extreme"],
        "EVAC_BLOCK_INIT_PROB": max(
            BLIZZARD_SEVERITY_OVERRIDES["extreme"]["EVAC_BLOCK_INIT_PROB"],
            EARTHQUAKE_SEVERITY_OVERRIDES["extreme"]["EVAC_BLOCK_INIT_PROB"],
        ),
        "earthquake_magnitude": EARTHQUAKE_SEVERITY_OVERRIDES["extreme"]["earthquake_magnitude"],
    },
}


def _deep_merge_dict(base, override):
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge_dict(out[key], value)
        else:
            out[key] = value
    return out


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _resolve_profile_path(scenario_path, profile_name):
    scenario_dir = os.path.dirname(os.path.abspath(scenario_path))
    profile_dir = os.path.join(scenario_dir, "profiles")
    return os.path.join(profile_dir, f"{profile_name}.json")


def validate_scenario(scenario):
    missing = [k for k in REQUIRED_TOP_LEVEL_KEYS if k not in scenario]
    if missing:
        raise ValueError(f"Scenario missing keys: {missing}")
    if not isinstance(scenario["policies"], list) or not scenario["policies"]:
        raise ValueError("Scenario 'policies' must be a non-empty list.")
    if not isinstance(scenario["config_overrides"], dict):
        raise ValueError("Scenario 'config_overrides' must be a dict.")


def _normalize_disaster_severity(raw):
    if raw is None:
        return None
    if isinstance(raw, (int, float)):
        val = float(raw)
        if val <= 1.5:
            return "light"
        if val <= 2.5:
            return "moderate"
        if val <= 3.5:
            return "severe"
        return "extreme"
    val = str(raw).strip().lower()
    aliases = {
        "low": "light",
        "minor": "light",
        "mid": "moderate",
        "medium": "moderate",
        "high": "severe",
        "major": "severe",
        "critical": "extreme",
    }
    return aliases.get(val, val if val in {"light", "moderate", "severe", "extreme"} else None)


def _severity_overrides(disaster_type, severity):
    if severity is None:
        return {}
    # Prefer LLM-generated presets if available
    if _LLM_SEVERITY_PRESETS:
        llm_table = _LLM_SEVERITY_PRESETS.get(disaster_type, {})
        if severity in llm_table:
            return dict(llm_table[severity])
    # Fallback: hard-coded tables
    table = {
        "blizzard": BLIZZARD_SEVERITY_OVERRIDES,
        "earthquake": EARTHQUAKE_SEVERITY_OVERRIDES,
        "compound": COMPOUND_SEVERITY_OVERRIDES,
    }.get(disaster_type, {})
    return dict(table.get(severity, {}))


def _apply_disaster_rules(scenario):
    scenario_overrides = dict(scenario.get("_scenario_overrides", scenario.get("config_overrides", {})))
    profile_overrides = dict(scenario.get("_profile_overrides", {}))
    overrides = _deep_merge_dict(profile_overrides, scenario_overrides)
    disaster_type = scenario.get("disaster_type", "")
    eq_mag = scenario.get("earthquake_magnitude")
    severity = _normalize_disaster_severity(scenario.get("disaster_severity"))

    if severity is not None:
        severity_cfg = _severity_overrides(disaster_type, severity)
        eq_mag = severity_cfg.pop("earthquake_magnitude", eq_mag)
        overrides = _deep_merge_dict(profile_overrides, severity_cfg)
        overrides = _deep_merge_dict(overrides, scenario_overrides)
        scenario["disaster_severity"] = severity

    # If earthquake magnitude >= 8.0, enforce at least 50% initial road damage.
    if disaster_type == "earthquake" and eq_mag is not None:
        try:
            if float(eq_mag) >= 8.0:
                overrides["EVAC_BLOCK_INIT_PROB"] = max(
                    0.5, float(overrides.get("EVAC_BLOCK_INIT_PROB", 0.0))
                )
        except Exception:
            pass
    if eq_mag is not None:
        scenario["earthquake_magnitude"] = eq_mag

    scenario["config_overrides"] = overrides
    return scenario


def load_scenario(path):
    raw = _load_json(path)
    scenario = dict(DEFAULT_SCENARIO)
    scenario.update(raw)
    scenario["_scenario_overrides"] = dict(raw.get("config_overrides", {}))
    scenario["_profile_overrides"] = {}

    profile_name = scenario.get("profile")
    if profile_name:
        profile_path = _resolve_profile_path(path, profile_name)
        if not os.path.exists(profile_path):
            raise FileNotFoundError(f"Scenario profile not found: {profile_path}")
        profile_raw = _load_json(profile_path)
        profile_overrides = dict(profile_raw.get("config_overrides", {}))
        scenario["_profile_overrides"] = dict(profile_overrides)
        scenario["config_overrides"] = _deep_merge_dict(profile_overrides, scenario.get("config_overrides", {}))
        scenario["disaster_type"] = scenario.get("disaster_type", profile_raw.get("disaster_type", "custom"))
        scenario["profile_data"] = profile_raw
        if "disaster_severity" in profile_raw and "disaster_severity" not in scenario:
            scenario["disaster_severity"] = profile_raw["disaster_severity"]
        if "earthquake_magnitude" in profile_raw and "earthquake_magnitude" not in scenario:
            scenario["earthquake_magnitude"] = profile_raw["earthquake_magnitude"]

    scenario["policies"] = list(scenario.get("policies", DEFAULT_SCENARIO["policies"]))
    scenario["config_overrides"] = dict(scenario.get("config_overrides", {}))
    scenario = _apply_disaster_rules(scenario)
    validate_scenario(scenario)
    return scenario


@contextmanager
def temporary_config(overrides):
    overrides = overrides or {}
    previous = {}
    for key, value in overrides.items():
        if hasattr(config, key):
            previous[key] = getattr(config, key)
            setattr(config, key, value)
    try:
        yield
    finally:
        for key, value in previous.items():
            setattr(config, key, value)
