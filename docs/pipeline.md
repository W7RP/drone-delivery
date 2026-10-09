# Offline pipeline

`pipeline/run.sh` turns public data for Allegheny County into the network files the sim reads
(spec 8.4 and 9) and writes the `parts_logistics_pittsburgh` scenario. The sim never touches
map data. The rules live in three YAML files, and every number in them is a placeholder
(spec A01, A45 to A53):

- `pipeline/sources.yaml`: what to download.
- `pipeline/sites.yaml`: how pads are chosen.
- `pipeline/params.yaml`: everything else.

## Sources

| Source | Used for | License |
|---|---|---|
| Geofabrik Pennsylvania OSM extract | drive network, water polygons, named sites | ODbL |
| USGS 3DEP 1 arc-second DEM, tiles n41w080 and n41w081 | highest terrain along each lane | public domain |
| Census 2020 TIGER/Line block groups and blocks (POP20) | population density for lane ground risk | public domain |
| FAA UAS Facility Maps | airspace exclusion zones | public domain |

None needs an account or key. Two sources changed from the plan:

- **Census population:** the Census data API now refuses requests without a key, so population
  comes from the POP20 field of the block shapefile, summed into block groups.
- **Airspace:** the FAA Class Airspace layer was replaced by the Facility Map layer. Treating the
  whole Allegheny County Airport (AGC) class D surface area as a wall would cut Clairton off from
  every other pad. The facility map gives a LAANC ceiling per 30 arc-second cell, and only cells
  below the placeholder 200 ft cruise altitude are excluded (A46).

Downloads go to `pipeline/data/raw/` (git-ignored, 897 MB). `manifest.json` there records each
file's URL, retrieval date, license, sha256 and the pipeline revision that fetched it. A
cached file is re-hashed against the manifest on every run and never downloaded again. Every
source here was fetched by work-in-progress pipeline code, so each entry's revision ends in
`-dirty`.

## Steps

1. **Pads.** For each cluster in `sites.yaml` the pipeline takes the largest named OSM footprints
   that match the cluster's tags within the search radius. It rejects a site whose pad point is
   in water, in an exclusion cell, or within 400 m of a pad already chosen. The pad stands at a
   point inside the footprint, and its name is the OSM name. Whether the lot or roof is usable is
   not checked (A45).

   Strip District has only two mapped warehouse footprints that are large enough and 400 m apart,
   so the result is 9 pads, not 10:

   | Cluster | Pads |
   |---|---|
   | Strip District | 2 warehouses (the larger is the hub) |
   | Oakland | 3 hospitals |
   | Braddock | 2 industrial sites |
   | Clairton | 2 industrial sites |

2. **Lanes.** Candidate lanes are pad pairs within the flight model's longest feasible lane at
   5 kg, 21.1 km. A candidate is rejected if its straight path crosses an exclusion cell, or if
   the sim's R12 energy guard would refuse it.

   The guard check mirrors `sim/src/flight.rs`, and the sim re-checks every lane at load. Each
   kept lane carries its length, the highest DEM point under it, and a ground-risk score: the mean
   block-group population density along the path, with water counted as zero (A47 to A49).

   Result: 24 lanes kept, 12 rejected. All 12 cross the cells around the AGC runways, which leaves
   Clairton reachable only through the Edgar Thomson plant.
3. **Truck matrix.** The pipeline builds a directed road graph from OSM:
   - Speed comes from `maxspeed` where it parses and is plausible, else from a per-class
     placeholder.
   - OSM one-way rules apply.

   Each pad snaps to the nearest non-motorway node of the largest strongly connected component,
   and scipy Dijkstra runs from every pad. The time-of-day profile stays in the scenario
   (`truck.tod_speed_factors`), as for synthetic networks (A50). On the 24 lane pairs, road
   distance is a median 1.54 and p90 2.09 times the straight line.
4. **Files.** `configs/parts_logistics_pittsburgh/derived/` holds:

   | File | Contents |
   |---|---|
   | `pads.toml` | the pad catalog, in the scenario's `pads[]` form |
   | `lanes.csr.bin` | lanes, spec 9.1, version 1.1 |
   | `truck_matrix.bin` | truck matrix, spec 9.2, version 1.1 |
   | `report.json` | every choice and rejection, with reasons |
   | `manifest.json` | the data sources |

   Both binary files carry the FNV-1a hash of `pads.toml`, so the sim refuses them next to any
   other catalog. `derived/` is committed: the OSM extract changes daily and cannot be fetched
   again as it was (A51).
5. **Sanity.** The files are read back from disk and checked:
   - no pad in water or an exclusion zone;
   - every lane no shorter than the great-circle distance;
   - every road time no faster than the straight line at the fastest road speed;
   - a finite matrix;
   - the spec 9 reader rules.
6. **Scenario.** `scenario.toml` comes from `pipeline/scenario.toml.in` plus three things the
   pipeline decides by rule (A52):
   - **Demand:** 4 requests/h from each warehouse, split over the pads its drones can reach.
     The four flows to Clairton need 2.61 to 2.67 MJ against 2.43 MJ available and are left out.
   - **Pods:** they belong to the warehouses and every pad sends one home once it is unloaded.
   - **Fleet:** ceil(offered drone-hours per hour / 0.5) = 8 drones, where an order's drone time
     includes recharging both legs.
7. **Variants.** Steps 2 to 6 run once per entry of `params.yaml` `scenarios`, each into its own
   `configs/<name>/`:
   - `parts_logistics_pittsburgh`: the anchor, with the flight profile fitted to PX4 SITL
     (`docs/calibration.md`).
   - `parts_logistics_pittsburgh_placeholder`: Phase 3's placeholder profile. Its catalog, lanes
     and truck matrix are byte-identical to Phase 3's.
   - `parts_logistics_pittsburgh_battery_1370wh`: the fitted profile with a 1370 Wh battery,
     sized for the longest legal lane at 5 kg (spec A66).
   - `parts_logistics_pittsburgh_vtol`: the VTOL profile of Phase 7 with a 620 Wh battery
     coupled to its mass (spec A68), and the Phase 6a pad keys (`docs/vtol.md`).

   A variant can select a flight profile, override drone keys, couple the battery's mass,
   set or remove other scenario keys and raise the schema version. Its lane range, energy guard,
   flows and fleet all follow its own model. If its lanes no longer reach a pad cluster, the
   scenario keeps the hub's side (spec A65).
8. **Plot.** `pipeline/out/pittsburgh_overview.png` shows the pads, the kept lanes coloured by
   ground risk, the rejected lanes, the exclusion cells and the rivers.

## Running it

```
pipeline/run.sh          # fetch or reuse the cache, build, check, plot
pipeline/run.sh fetch    # downloads only
pipeline/run.sh test     # unit tests for the binary formats and tag parsers
```

`run.sh` creates `pipeline/.venv` with uv: Python 3.12 and pinned, hashed binary wheels from
`requirements.txt`. The system Python lacks `ensurepip`, so `python3 -m venv` would need
`sudo apt install python3.10-venv`; uv avoids that.

A full build takes 53 s and 1.3 GB peak RSS, most of it one pass over the Pennsylvania extract.
The parsed extract is cached in `pipeline/data/cache/`, so a rebuild takes a few seconds.
Deleting the cache changes no output. Two runs give byte-identical outputs, the second with
networking switched off (`unshare -rn pipeline/run.sh`).

## Limits

- One data snapshot (2026-10-07). A fresh download moves pads and lanes slightly.
- Lanes are straight. A detour around an exclusion is not a lane; two lanes through a third pad
  are.
- Ground risk is a density average, not a casualty model. Terrain is carried, but the core
  does not use it yet.
- No traffic. Truck times are free-flow, scaled by the scenario's placeholder time-of-day
  factors.
