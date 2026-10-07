"""Read the pipeline YAML files. Numbers are `{value, unit, assumption}` tables, as in the
scenario configs; `q()` checks the unit and returns the value."""

import yaml

from .paths import PIPELINE


def load(name: str) -> dict:
    with open(PIPELINE / name) as f:
        return yaml.safe_load(f)


def q(table: dict, unit: str) -> float:
    if not isinstance(table, dict) or table.get("unit") != unit or "value" not in table:
        raise ValueError(f"expected a quantity in {unit}, got {table!r}")
    return float(table["value"])

