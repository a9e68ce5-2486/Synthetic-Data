import json
import os
from contextlib import contextmanager

import config


DEFAULT_SCENARIO = {
    "name": "enterprise_baseline",
    "description": "Enterprise-oriented evacuation baseline scenario.",
    "disaster_type": "custom",
    "baseline_policy": "round_robin",
    "num_runs": 20,
    "base_seed": 42,
    "policies": ["round_robin", "nearest", "priority_faculty"],
    "profile": None,
    "config_overrides": {},
}


REQUIRED_TOP_LEVEL_KEYS = ("name", "num_runs", "base_seed", "policies", "config_overrides")


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


def load_scenario(path):
    raw = _load_json(path)
    scenario = dict(DEFAULT_SCENARIO)
    scenario.update(raw)

    profile_name = scenario.get("profile")
    if profile_name:
        profile_path = _resolve_profile_path(path, profile_name)
        if not os.path.exists(profile_path):
            raise FileNotFoundError(f"Scenario profile not found: {profile_path}")
        profile_raw = _load_json(profile_path)
        profile_overrides = dict(profile_raw.get("config_overrides", {}))
        scenario["config_overrides"] = _deep_merge_dict(profile_overrides, scenario.get("config_overrides", {}))
        scenario["disaster_type"] = scenario.get("disaster_type", profile_raw.get("disaster_type", "custom"))
        scenario["profile_data"] = profile_raw

    scenario["policies"] = list(scenario.get("policies", DEFAULT_SCENARIO["policies"]))
    scenario["config_overrides"] = dict(scenario.get("config_overrides", {}))
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
