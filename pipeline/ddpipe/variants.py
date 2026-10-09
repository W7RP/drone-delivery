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
    # Battery-mass coupling (A68): airframe_mass_kg and battery_specific_energy_wh_per_kg
    # replace the template's empty_mass_kg (schema 1.6.0 keys).
    coupling: dict = field(default_factory=dict)
    # Keys outside the drone section: "section.key" -> value (a quantity table, bool or
    # string), replacing the template's line or added under the section header; `remove`
    # drops template keys; `schema_version` overrides the template's.
    set: dict = field(default_factory=dict)
    remove: list = field(default_factory=list)
    schema_version: str | None = None

    @property
    def scenario_dir(self) -> Path:
        return REPO / "configs" / self.name

    @property
    def derived_dir(self) -> Path:
        return self.scenario_dir / "derived"

    def profile_path(self) -> Path | None:
        return PROFILE_DIR / f"{self.flight_profile}.toml" if self.flight_profile else None


def load(params: dict) -> list[Variant]:
    return [Variant(v["name"], v["description"], v.get("flight_profile"), v.get("drone", {}),
                    v.get("coupling", {}), v.get("set", {}), v.get("remove", []),
                    v.get("schema_version"))
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
    if variant.coupling:
        cfg["drone"].pop("empty_mass_kg")
        cfg["drone"].update(copy.deepcopy(variant.coupling))
    for path in variant.remove:
        section, key = path.split(".")
        cfg[section].pop(key)
    for path, value in variant.set.items():
        section, key = path.split(".")
        cfg[section][key] = copy.deepcopy(value)
    return cfg


def _value(v) -> str:
    if isinstance(v, dict):
        return _quantity(v)
    if isinstance(v, bool):
        return str(v).lower()
    return f'"{v}"'


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
    if variant.coupling:
        lines = "\n".join(f"{k} = {_quantity(q)}" for k, q in variant.coupling.items())
        t = sub(r"^empty_mass_kg = \{[^}]*\}$",
                f"# Battery-mass coupling (spec A68): empty mass = airframe + battery / specific energy.\n{lines}", t)
        t = sub(r'^schema_version = "[^"]*"$', 'schema_version = "1.6.0"', t)
    for path in variant.remove:
        key = path.split(".")[1]
        t = sub(rf"^{key} = [^\n]*\n", "", t)
    for path, value in variant.set.items():
        section, key = path.split(".")
        line = f"{key} = {_value(value)}"
        if re.search(rf"^{key} = ", t, flags=re.M):
            t = sub(rf"^{key} = [^\n]*$", f"# Variant override ({variant.name}).\n{line}", t)
        else:
            t = sub(rf"^\[{section}\]$", f"[{section}]\n# Variant key ({variant.name}).\n{line}", t)
    if variant.schema_version:
        t = sub(r'^schema_version = "[^"]*"$', f'schema_version = "{variant.schema_version}"', t)
    return t
