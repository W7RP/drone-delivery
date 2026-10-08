"""Fixed locations of pipeline inputs and outputs, relative to the repository."""

from pathlib import Path

PIPELINE = Path(__file__).resolve().parent.parent
REPO = PIPELINE.parent
RAW = PIPELINE / "data" / "raw"
MANIFEST = RAW / "manifest.json"
CACHE = PIPELINE / "data" / "cache"
OUT = PIPELINE / "out"
SCENARIO_NAME = "parts_logistics_pittsburgh"
SCENARIO_DIR = REPO / "configs" / SCENARIO_NAME
DERIVED = SCENARIO_DIR / "derived"
WEATHER_DIR = REPO / "configs" / "weather" / "pittsburgh"
