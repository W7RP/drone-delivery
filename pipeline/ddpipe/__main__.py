"""Entry point: `python -m ddpipe [fetch|all]` (default all)."""

import argparse
import logging
import sys

from . import constraints, fetch, osm, pads
from .params import load
from .paths import RAW

OSM_FILE = "pennsylvania-latest.osm.pbf"


def build() -> None:
    sources = fetch.load_sources()
    sites_cfg = load("sites.yaml")
    params = load("params.yaml")
    extract = osm.read(RAW / OSM_FILE, sources["area"], pads.site_keys(sites_cfg))
    cons = constraints.build(params, extract.water)
    pads.select(sites_cfg, extract.sites, cons)


def main() -> int:
    parser = argparse.ArgumentParser(prog="ddpipe")
    parser.add_argument("step", nargs="?", default="all", choices=["fetch", "all"])
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    fetch.fetch_all()
    if args.step == "all":
        build()
    return 0


if __name__ == "__main__":
    sys.exit(main())
