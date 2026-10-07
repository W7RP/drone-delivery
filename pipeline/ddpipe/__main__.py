"""Entry point: `python -m ddpipe [fetch|all]` (default all)."""

import argparse
import logging
import sys

import tomllib

from . import constraints, energy, fetch, lanes, osm, pads
from .params import load
from .paths import CACHE, PIPELINE, RAW
from .population import Population
from .terrain import Terrain

OSM_FILE = "pennsylvania-latest.osm.pbf"


def build(manifest: dict) -> None:
    sources = fetch.load_sources()
    sites_cfg = load("sites.yaml")
    params = load("params.yaml")
    extract = osm.read_cached(RAW / OSM_FILE, manifest["files"]["osm_pennsylvania"]["sha256"],
                              CACHE, sources["area"], pads.site_keys(sites_cfg))
    cons = constraints.build(params, extract.water)
    pad_list, _ = pads.select(sites_cfg, extract.sites, cons)
    with open(PIPELINE / "scenario.toml.in", "rb") as f:
        template = tomllib.load(f)
    model = energy.FlightModel.from_scenario(template)
    area = sources["area"]
    lanes.build(pad_list, cons, model, params, Terrain(area), Population(area))


def main() -> int:
    parser = argparse.ArgumentParser(prog="ddpipe")
    parser.add_argument("step", nargs="?", default="all", choices=["fetch", "all"])
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    manifest = fetch.fetch_all()
    if args.step == "all":
        build(manifest)
    return 0


if __name__ == "__main__":
    sys.exit(main())
