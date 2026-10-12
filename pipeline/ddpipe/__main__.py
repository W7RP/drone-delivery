"""Entry point: `python -m ddpipe [fetch|all]` (default all)."""

import argparse
import dataclasses
import logging
import sys

import tomllib

import numpy as np

from . import (constraints, energy, fetch, lanes, osm, outputs, pads, plot, sanity, scenario,
               truck, variants)
from .geo import Enu
from .params import load, q
from .paths import CACHE, PIPELINE, RAW
from .population import Population
from .provenance import pipeline_revision
from .terrain import Terrain

OSM_FILE = "pennsylvania-latest.osm.pbf"


def restrict(pad_list, lane_list, tm, keep: list[int]):
    """The pads in `keep` (old IDs, ascending) renumbered from 0, their lanes, their truck
    matrix rows and columns."""
    index = {old: new for new, old in enumerate(keep)}
    sub = np.ix_(keep, keep)
    return ([pad_list[i] for i in keep],
            [dataclasses.replace(l, a=index[l.a], b=index[l.b]) for l in lane_list
             if l.a in index and l.b in index],
            dataclasses.replace(tm, free_flow_s=tm.free_flow_s[sub], road_m=tm.road_m[sub],
                                snap_m=tm.snap_m[keep], snap_node=tm.snap_node[keep]))


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
    area = sources["area"]
    terrain, population = Terrain(area), Population(area)
    geo = template["geo"]
    enu = Enu(geo["origin_lat_deg"]["value"], geo["origin_lon_deg"]["value"])
    graph = truck.build_graph(extract.roads, params)
    tm_all = truck.build(pad_list, graph, enu, params)
    revision = pipeline_revision()
    retrieved = manifest["files"]["osm_pennsylvania"]["retrieved_utc"]
    hub = next(i for i, p in enumerate(pad_list) if p.is_hub)
    for n, variant in enumerate(variants.load(params)):
        logging.info("== scenario %s", variant.name)
        cfg = variants.config(template, variant)
        model = energy.FlightModel.from_scenario(cfg)
        lane_list, lane_rejections, lane_range_m = lanes.build(
            pad_list, cons, model, params, terrain, population)
        # A flight model that cannot reach every cluster splits the lane graph; the scenario
        # keeps the hub's part (spec A65), and the report names the pads left out.
        keep = lanes.component(len(pad_list), lane_list, hub)
        dropped = [pad_list[i].name for i in range(len(pad_list)) if i not in keep]
        for name in dropped:
            logging.info("pad %s left out of %s: no lane connects it to the hub", name, variant.name)
        v_pads, v_lanes, v_tm = restrict(pad_list, lane_list, tm_all, keep)
        ratios = truck.detour_ratios(v_pads, v_tm, [(l.a, l.b) for l in v_lanes])
        logging.info("road / straight on lane pairs: median %.2f, p90 %.2f",
                     np.median(ratios), np.percentile(ratios, 90))
        scenario_doc = scenario.write(v_pads, v_lanes, cfg, model, params, variant)
        doc = outputs.report(v_pads, pad_rejections, v_lanes, lane_rejections, lane_range_m,
                             q(params["lanes"]["payload_kg"], "kg"), len(cons.cells), v_tm,
                             graph, ratios, scenario_doc, revision)
        doc["variant"] = {"name": variant.name, "flight_profile": variant.flight_profile,
                          "drone_overrides": variant.drone,
                          "pads_left_out": [{"name": d, "reason": "no lane to the hub"} for d in dropped]}
        outputs.write_derived(variant.derived_dir, v_pads, sites_cfg, v_lanes, v_tm, doc,
                              manifest, revision)
        sanity.check(params, extract.water, variant.derived_dir)
        if n == 0:
            # Every selected pad, so the pads a variant leaves out still show.
            plot.overview(pad_list, lane_list, lane_rejections, cons, retrieved)


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
