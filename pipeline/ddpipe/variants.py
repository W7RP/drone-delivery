"""Scenario variants: the pipeline writes one scenario directory per entry of params.yaml
`scenarios`, all from scenario.toml.in and the same pad selection. A variant can select a
flight profile (spec 8.7) and override drone keys; its lanes, flows and fleet are then decided
with that variant's flight model, so each variant has its own derived/ files."""

import copy
import re
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from .paths import REPO

PROFILE_DIR = REPO / "configs" / "flight_profiles"
PROFILE_MARKER = "# @flight_profile: written by the pipeline when the scenario selects a profile.\n"


@dataclass
class Variant:
    name: str
    description: str
    flight_profile: str | None = None
    drone: dict = field(default_factory=dict)

    @property
    def scenario_dir(self) -> Path:
        return REPO / "configs" / self.name

    @property
    def derived_dir(self) -> Path:
        return self.scenario_dir / "derived"

    def profile_path(self) -> Path | None:
        return PROFILE_DIR / f"{self.flight_profile}.toml" if self.flight_profile else None


def load(params: dict) -> list[Variant]:
    return [Variant(v["name"], v["description"], v.get("flight_profile"), v.get("drone", {}))
            for v in params["scenarios"]]


def config(template: dict, variant: Variant) -> dict:
    """The template's config as the sim will read this variant: profile values, then the
    variant's drone overrides, over the drone section."""
    cfg = copy.deepcopy(template)
    path = variant.profile_path()
    if path is not None:
        with open(path, "rb") as f:
            cfg["drone"].update(tomllib.load(f)["drone"])
    cfg["drone"].update(copy.deepcopy(variant.drone))
    return cfg


def _quantity(q: dict) -> str:
    return (f'{{ value = {q["value"]!r}, unit = "{q["unit"]}", '
            f'assumption = {str(q["assumption"]).lower()} }}')


def text(template_text: str, variant: Variant) -> str:
    """scenario.toml.in with this variant's name, description, profile and drone overrides."""
    def sub(pattern: str, repl: str, s: str) -> str:
        out, n = re.subn(pattern, repl, s, count=1, flags=re.M)
        if n != 1:
            raise ValueError(f"scenario.toml.in: {pattern!r} not found")
        return out

    t = sub(r'^name = "[^"]*"$', f'name = "{variant.name}"', template_text)
    t = sub(r'^description = "[^"]*"$', f'description = "{variant.description}"', t)
    if t.count(PROFILE_MARKER) != 1:
        raise ValueError("scenario.toml.in: the @flight_profile marker must appear once")
    profile = ""
    if variant.flight_profile:
        rel = Path("..") / "flight_profiles" / f"{variant.flight_profile}.toml"
        profile = (f"# Flight profile (spec 8.7): replaces the drone section's flight-model keys,\n"
                   f"# which stay the placeholder profile.\n[flight_profile]\nfile = \"{rel}\"\n")
    t = t.replace(PROFILE_MARKER, profile)
    for key, q in variant.drone.items():
        t = sub(rf"^{key} = \{{[^}}]*\}}$", f"# Variant override ({variant.name}).\n{key} = {_quantity(q)}", t)
    return t
