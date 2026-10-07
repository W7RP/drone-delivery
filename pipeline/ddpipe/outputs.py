"""Step 5: write the scenario's derived/ directory (spec 8.4): the pad catalog pads.toml, the
binary lane graph and truck matrix (spec 9.1, 9.2) keyed to the catalog by its FNV-1a hash,
report.json (every choice and rejection the pipeline made) and manifest.json (sources)."""

import json
import logging
from pathlib import Path

from . import binfmt
from .params import q
from .paths import DERIVED

log = logging.getLogger(__name__)

PADS_FILE = "pads.toml"
LANES_FILE = "lanes.csr.bin"
TRUCK_FILE = "truck_matrix.bin"
REPORT_FILE = "report.json"
MANIFEST_FILE = "manifest.json"


def _toml_str(s: str) -> str:
    return json.dumps(s, ensure_ascii=True)


def _quantity(value, unit: str, assumption: bool) -> str:
    return f'{{ value = {value!r}, unit = "{unit}", assumption = {str(assumption).lower()} }}'


def pads_toml(pads, defaults: dict) -> str:
    lines = [
        "# Pad catalog of parts_logistics_pittsburgh in the form of a scenario's pads[] (spec 8.2),",
        "# written by pipeline/run.sh; do not edit. Coordinates are OSM data (assumption = false);",
        "# the other fields are placeholders from pipeline/sites.yaml. lanes.csr.bin and",
        "# truck_matrix.bin carry the FNV-1a 64 hash of this file as their source_hash (spec 9.0).",
    ]
    pods = int(q(defaults["initial_empty_pods"], "count"))
    for p in pads:
        parking_key = "hub_parking_capacity" if p.is_hub else "parking_capacity"
        parking = int(q(defaults[parking_key], "count"))
        lines += [
            "",
            "[[pads]]",
            f"# OSM {p.osm_key} {_toml_str(p.osm_name)}: {p.cluster}, {p.role}, "
            f"footprint {p.site_area_m2:.0f} m2",
            f"name = {_toml_str(p.name)}",
            f"lat_deg = {_quantity(p.lat_deg, 'deg', False)}",
            f"lon_deg = {_quantity(p.lon_deg, 'deg', False)}",
            f"is_hub = {str(p.is_hub).lower()}",
            f"service_mode = {_toml_str(defaults['service_mode'])}",
            f"pod_return_policy = {_toml_str(defaults['pod_return_policy'])}",
            f"initial_empty_pods = {_quantity(pods, 'count', True)}",
            f"parking_capacity = {_quantity(parking, 'count', True)}",
        ]
    return "\n".join(lines) + "\n"


def _r(x: float, nd: int = 3) -> float:
    return round(float(x), nd)


def report(pads, pad_rejections, lane_list, lane_rejections, lane_range_m, payload_kg,
           exclusion_cells, tm, graph, ratios, revision: str) -> dict:
    import numpy as np

    return {
        "pipeline_git": revision,
        "pads": [
            {"id": i, "name": p.name, "osm_name": p.osm_name, "osm": p.osm_key,
             "cluster": p.cluster, "role": p.role, "is_hub": p.is_hub,
             "lat_deg": p.lat_deg, "lon_deg": p.lon_deg, "site_area_m2": _r(p.site_area_m2, 0),
             "truck_snap_m": _r(tm.snap_m[i], 1)}
            for i, p in enumerate(pads)
        ],
        "pad_rejections": [vars(r) for r in pad_rejections],
        "lane_payload_kg": payload_kg,
        "lane_range_m": _r(lane_range_m, 1),
        "airspace_exclusion_cells": exclusion_cells,
        "lanes": [
            {"a": pads[l.a].name, "b": pads[l.b].name, "length_m": _r(l.length_m, 1),
             "max_terrain_m": _r(l.max_terrain_m, 1),
             "ground_risk_per_km2": _r(l.ground_risk_per_km2, 1)}
            for l in lane_list
        ],
        "lane_rejections": [
            {"a": r.a, "b": r.b, "length_m": _r(r.length_m, 1), "reason": r.reason}
            for r in lane_rejections
        ],
        "truck": {
            "road_nodes": int(len(graph.node_ids)),
            "ways_with_maxspeed": graph.tagged_ways,
            "ways_with_class_default_speed": graph.default_ways,
            "max_speed_mps": _r(tm.max_speed_mps, 3),
            "road_over_straight_on_lane_pairs": {
                "median": _r(np.median(ratios), 3),
                "p90": _r(np.percentile(ratios, 90), 3),
                "min": _r(ratios.min(), 3),
                "max": _r(ratios.max(), 3),
            },
        },
    }


def _write(path: Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + ".part")
    tmp.write_bytes(data)
    tmp.rename(path)


def write_derived(pads, defaults, lane_list, tm, report_doc: dict, manifest: dict,
                  revision: str) -> dict[str, Path]:
    DERIVED.mkdir(parents=True, exist_ok=True)
    catalog = pads_toml(pads, defaults).encode()
    source_hash = binfmt.fnv1a64(catalog)
    files = {
        PADS_FILE: catalog,
        LANES_FILE: binfmt.write_lanes(binfmt.lane_graph(len(pads), lane_list), source_hash),
        TRUCK_FILE: binfmt.write_truck(tm.free_flow_s, tm.road_m, source_hash),
        REPORT_FILE: (json.dumps(report_doc, indent=1, sort_keys=True) + "\n").encode(),
        MANIFEST_FILE: (json.dumps({"pipeline_git": revision, "sources": manifest["files"]},
                                   indent=1, sort_keys=True) + "\n").encode(),
    }
    out = {}
    for name, data in files.items():
        _write(DERIVED / name, data)
        out[name] = DERIVED / name
    log.info("derived: %s (source_hash %016x)", ", ".join(f"{k} {len(v)} B" for k, v in
                                                          files.items()), source_hash)
    return out
