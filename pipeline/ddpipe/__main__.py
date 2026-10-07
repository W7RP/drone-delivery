"""Entry point: `python -m ddpipe [fetch|all]` (default all)."""

import argparse
import logging
import sys

import tomllib

import numpy as np

from . import constraints, energy, fetch, lanes, osm, outputs, pads, sanity, truck
from .geo import Enu
from .params import load, q
from .paths import CACHE, PIPELINE, RAW
from .population import Population
from .provenance import pipeline_revision
from .terrain import Terrain

OSM_FILE = "pennsylvania-latest.osm.pbf"


def build(manifest: dict) -> None:
    sources = fetch.load_sources()
    sites_cfg = load("sites.yaml")
    params = load("params.yaml")
    extract = osm.read_cached(RAW / OSM_FILE, manifest["files"]["osm_pennsylvania"]["sha256"],
                              CACHE, sources["area"], pads.site_keys(sites_cfg))
    cons = constraints.build(params, extract.water)
    pad_list, pad_rejections = pads.select(sites_cfg, extract.sites, cons)
    with open(PIPELINE / "scenario.toml.in", "rb") as f:
        template = tomllib.load(f)
    model = energy.FlightModel.from_scenario(template)
    area = sources["area"]
    lane_list, lane_rejections, lane_range_m = lanes.build(pad_list, cons, model, params, Terrain(area), Population(area))
    geo = template["geo"]
    enu = Enu(geo["origin_lat_deg"]["value"], geo["origin_lon_deg"]["value"])
    graph = truck.build_graph(extract.roads, params)
    tm = truck.build(pad_list, graph, enu, params)
    ratios = truck.detour_ratios(pad_list, tm, [(l.a, l.b) for l in lane_list])
    logging.info("road / straight on lane pairs: median %.2f, p90 %.2f",
                 np.median(ratios), np.percentile(ratios, 90))
    revision = pipeline_revision()
    doc = outputs.report(pad_list, pad_rejections, lane_list, lane_rejections, lane_range_m,
                         q(params["lanes"]["payload_kg"], "kg"), len(cons.cells), tm, graph,
                         ratios, revision)
    outputs.write_derived(pad_list, sites_cfg["pad_defaults"], lane_list, tm, doc, manifest,
                          revision)
    sanity.check(params, extract.water)


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
