# /// script
# requires-python = ">=3.10"
# dependencies = ["jsonschema==4.23.0", "referencing==0.35.1", "tomli==2.0.1"]
# ///
"""Phase-1 contract checks: configs against schemas, config cross-field rules (spec 8.3),
and docs/spec.md + docs/paper-run.md against the schemas. Exits non-zero on any failure."""

import json
import math
import re
import sys
from pathlib import Path

import tomli
from jsonschema import Draft202012Validator
from referencing import Registry, Resource

ROOT = Path(__file__).resolve().parent.parent
SCHEMAS = ROOT / "schemas"
SPEC = ROOT / "docs" / "spec.md"
PAPER_RUN = ROOT / "docs" / "paper-run.md"
EARTH_RADIUS_M = 6371000.0
FORBIDDEN_WORDS = re.compile(r"\b(TBD|TODO|later)\b", re.IGNORECASE)

errors: list[str] = []


def fail(msg: str) -> None:
    errors.append(msg)


def load_schemas() -> tuple[dict, Registry]:
    schemas = {p.name: json.loads(p.read_text()) for p in SCHEMAS.glob("*.schema.json")}
    registry = Registry().with_resources(
        (s["$id"], Resource.from_contents(s)) for s in schemas.values()
    )
    for name, schema in schemas.items():
        Draft202012Validator.check_schema(schema)
    return schemas, registry


def q(node: dict) -> float:
    return node["value"]


# ---- config checks -------------------------------------------------------------------------


def iter_quantities(node, path=""):
    if isinstance(node, dict):
        if set(node) == {"value", "unit", "assumption"}:
            yield path, node
            return
        for k, v in node.items():
            yield from iter_quantities(v, f"{path}.{k}" if path else k)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from iter_quantities(v, f"{path}[{i}]")
    elif isinstance(node, (int, float)) and not isinstance(node, bool):
        yield path, None


def enu_m(lat_deg, lon_deg, lat0_deg, lon0_deg):
    x = (
        EARTH_RADIUS_M
        * math.cos(math.radians(lat0_deg))
        * math.radians(lon_deg - lon0_deg)
    )
    y = EARTH_RADIUS_M * math.radians(lat_deg - lat0_deg)
    return x, y


def check_dispatch_energy(cfg: dict, name: str) -> None:
    """R12/R16: every directed lane and every demand flow passes dispatch_ok (spec 6)."""
    d, po, pol = cfg["drone"], cfg["pad_ops"], cfg["policy"]
    lat0, lon0 = q(cfg["geo"]["origin_lat_deg"]), q(cfg["geo"]["origin_lon_deg"])
    pos = {
        p["name"]: enu_m(q(p["lat_deg"]), q(p["lon_deg"]), lat0, lon0)
        for p in cfg["pads"]
    }
    payload_kg = q(cfg["payload"]["pod_gross_mass_kg"])

    def k(m):
        return (
            (q(d["empty_mass_kg"]) + q(d["latch_mass_kg"]) + m)
            / q(d["power_ref_mass_kg"])
        ) ** 1.5

    adj: dict[str, dict[str, float]] = {p: {} for p in pos}
    for lane in cfg["lanes"]:
        a, b = lane["pad_a"], lane["pad_b"]
        adj[a][b] = adj[b][a] = math.dist(pos[a], pos[b])

    def route_len(src):
        dist = {src: 0.0}
        todo = set(pos)
        while todo:
            u = min(
                (p for p in todo if p in dist), key=lambda p: (dist[p], p), default=None
            )
            if u is None:
                break
            todo.discard(u)
            for v, w in adj[u].items():
                if dist[u] + w < dist.get(v, math.inf):
                    dist[v] = dist[u] + w
        return dist

    lens = {p: route_len(p) for p in pos}

    def e_route(a, b, m):
        return q(d["cruise_power_ref_w"]) * k(m) * lens[a][b] / q(d["cruise_speed_mps"])

    def p_hover(m):
        return q(d["hover_power_ref_w"]) * k(m)

    ngo = q(po["max_go_arounds"])
    t_cycle = (ngo + 1) * q(d["approach_s"]) + ngo * q(d["go_around_s"])
    t_cont = (q(pol["marker_max_hold_cycles"]) + 1) * t_cycle + max(
        q(pol["max_hold_s"]), q(pol["marker_max_hold_cycles"]) * q(pol["marker_hold_s"])
    )
    hub_only = cfg["charging"]["mode"] == "HUB_ONLY"
    chargers = [p["name"] for p in cfg["pads"] if not hub_only or p["is_hub"]]
    cap = q(d["battery_capacity_j"])
    pairs = [(a, b, "R12 lane") for a in pos for b in adj[a]]
    flows = cfg["demand"].get("flows", [])
    pairs += [(f["origin"], f["destination"], "R16 flow") for f in flows]
    for a, b, rule in pairs:
        alts = [e_route(b, qq, payload_kg) for qq in pos if qq != b]
        e_div = min(alts) + p_hover(payload_kg) * t_cycle
        e_chg = 0.0 if b in chargers else min(e_route(b, h, 0.0) for h in chargers)
        need = (
            p_hover(payload_kg) * q(d["takeoff_s"])
            + e_route(a, b, payload_kg)
            + p_hover(payload_kg) * t_cont
            + e_div
            + e_chg
        )
        have = (q(d["charge_target_soc"]) - q(d["reserve_soc"])) * cap
        if need > have:
            fail(
                f"{name}: {rule} {a}->{b} needs {need:.0f} J, has {have:.0f} J above reserve"
            )


def parking_slots(pad: dict) -> int:
    """slots(p) from spec 6.4: parking capacity, or 1 for the OUT spot when it is 0."""
    capacity = q(pad["parking_capacity"]) if "parking_capacity" in pad else 0
    return capacity if capacity > 0 else 1


def check_synthetic_config(cfg: dict, name: str) -> None:
    """Rules that need no generated pads; the core checks the rest after generation (8.3)."""
    net = cfg["synthetic_network"]
    n = q(net["n_pads"])
    generated = {f"P{i}" for i in range(n)}
    refs = list(cfg["drone"].get("initial_pads", [])) + [
        f["pad"] for f in cfg["faults"]
    ]
    for flow in cfg["demand"].get("flows", []):
        refs += [flow["origin"], flow["destination"]]
    for ref in refs:
        if ref not in generated:
            fail(f"{name}: pad name {ref} is not one of the generated P0..P{n - 1}")
    if cfg["charging"]["mode"] == "HUB_ONLY" and net["topology"] != "HUB_SPOKE":
        fail(f"{name}: R11 HUB_ONLY requires a hub (HUB_SPOKE topology)")


def check_complete_config(cfg: dict, name: str, scenario_dir: Path, validator) -> None:
    check_common_rules(cfg, name)
    if "synthetic_network" in cfg:
        check_synthetic_config(cfg, name)
    elif "network_files" in cfg:
        check_files_network(cfg, name, scenario_dir, validator)
    else:
        check_explicit_network(cfg, name)
    if cfg["truck"]["matrix_source"] == "FILE" and "network_files" not in cfg:
        fail(f"{name}: R17 truck.matrix_source FILE without network_files")


def check_files_network(cfg: dict, name: str, scenario_dir: Path, validator) -> None:
    """The pipeline's pad catalog against the pads[] schema, and the rules that need no lanes.
    The core checks the binary files and the lane rules at load (spec 8.3)."""
    path = scenario_dir / cfg["network_files"]["dir"] / "pads.toml"
    try:
        catalog = tomli.loads(path.read_text())
    except (OSError, tomli.TOMLDecodeError) as err:
        fail(f"{name}: {path}: {err}")
        return
    if set(catalog) != {"pads"}:
        fail(f"{name}: {path}: expected only a pads array")
        return
    # Same base URI as the config schema, so its relative $refs resolve in the registry.
    pad_schema = {
        "$id": validator.schema["$id"].replace("config.schema.json", "pads-catalog.json"),
        **validator.schema["properties"]["pads"],
    }
    for err in validator.evolve(schema=pad_schema).iter_errors(catalog["pads"]):
        fail(f"{name}: pads.toml: {'/'.join(map(str, err.absolute_path))}: {err.message[:200]}")
    for qpath, node in iter_quantities(catalog):
        coordinate = qpath.endswith(".lat_deg") or qpath.endswith(".lon_deg")
        if node is None:
            fail(f"{name}: pads.toml: bare number at {qpath}")
        elif not coordinate and node["assumption"] is not True:
            fail(f"{name}: pads.toml: R15 {qpath} not flagged assumption = true")
    check_pad_rules(cfg, name, catalog["pads"])


def check_common_rules(cfg: dict, name: str) -> None:
    d = cfg["drone"]
    pl = cfg["payload"]
    margin = q(d["mtow_kg"]) - q(d["empty_mass_kg"]) - q(d["latch_mass_kg"])
    if q(pl["pod_gross_mass_kg"]) > margin or q(pl["pod_empty_mass_kg"]) >= q(
        pl["pod_gross_mass_kg"]
    ):
        fail(f"{name}: R6 payload does not fit")
    if not (
        q(d["reserve_soc"]) < q(d["charge_target_soc"])
        and q(d["reserve_soc"]) < q(d["initial_soc"])
    ):
        fail(f"{name}: R7 soc ordering")
    if q(cfg["sim"]["warmup_s"]) >= q(cfg["sim"]["duration_s"]):
        fail(f"{name}: R8 warmup >= duration")
    if q(cfg["metrics"]["hist_max_s"]) <= q(cfg["metrics"]["hist_bin_s"]):
        fail(f"{name}: R13 histogram range")


def check_explicit_network(cfg: dict, name: str) -> None:
    pads = [p["name"] for p in cfg["pads"]]
    pad_set = set(pads)
    pairs = set()
    for lane in cfg["lanes"]:
        a, b = lane["pad_a"], lane["pad_b"]
        if a not in pad_set or b not in pad_set or a == b:
            fail(f"{name}: R2 bad lane {a}-{b}")
        if frozenset((a, b)) in pairs:
            fail(f"{name}: R2 duplicate lane {a}-{b}")
        pairs.add(frozenset((a, b)))
    seen, todo = {pads[0]}, [pads[0]]
    while todo:
        u = todo.pop()
        for pr in pairs:
            if u in pr:
                for v in pr - {u}:
                    if v not in seen:
                        seen.add(v)
                        todo.append(v)
    if seen != pad_set:
        fail(f"{name}: R3 lane graph not connected")
    check_pad_rules(cfg, name, cfg["pads"])
    check_dispatch_energy(cfg, name)


def check_pad_rules(cfg: dict, name: str, pad_list: list) -> None:
    """R1, R4, R5 and R9 to R11: the rules that need pads but no lanes."""
    pads = [p["name"] for p in pad_list]
    pad_set = set(pads)
    if len(pad_set) != len(pads):
        fail(f"{name}: R1 duplicate pad names")
    d = cfg["drone"]
    slots = {p["name"]: parking_slots(p) for p in pad_list}
    init = d.get("initial_pads", [])
    if init and (
        len(init) != q(d["fleet_size"])
        or not set(init) <= pad_set
        or any(init.count(p) > slots[p] for p in set(init))
    ):
        fail(
            f"{name}: R4 drone.initial_pads inconsistent with fleet_size, pads or slots"
        )
    if q(d["fleet_size"]) > sum(slots.values()):
        fail(f"{name}: R5 fleet_size exceeds total parking slots")
    duration = q(cfg["sim"]["duration_s"])
    for f in cfg["demand"].get("flows", []):
        if (
            f["origin"] not in pad_set
            or f["destination"] not in pad_set
            or f["origin"] == f["destination"]
        ):
            fail(f"{name}: R9 bad flow {f['origin']}->{f['destination']}")
    windows: dict[str, list[tuple[float, float]]] = {}
    for f in cfg["faults"]:
        if f["pad"] not in pad_set or q(f["start_s"]) >= duration:
            fail(f"{name}: R10 bad fault {f}")
        windows.setdefault(f["pad"], []).append(
            (q(f["start_s"]), q(f["start_s"]) + q(f["duration_s"]))
        )
    for pad, ws in windows.items():
        ws.sort()
        if any(ws[i][1] > ws[i + 1][0] for i in range(len(ws) - 1)):
            fail(f"{name}: R10 overlapping fault windows on {pad}")
    if cfg["charging"]["mode"] == "HUB_ONLY" and not any(p["is_hub"] for p in pad_list):
        fail(f"{name}: R11 HUB_ONLY without hub")


def check_configs(schemas: dict, registry: Registry) -> int:
    validator = Draft202012Validator(schemas["config.schema.json"], registry=registry)
    n = 0
    for path in sorted((ROOT / "configs").glob("*/scenario.toml")):
        n += 1
        name = path.parent.name
        try:
            cfg = tomli.loads(path.read_text())
        except tomli.TOMLDecodeError as err:
            fail(f"{name}: not valid TOML: {err}")
            continue
        for err in validator.iter_errors(cfg):
            fail(
                f"{name}: schema: {'/'.join(map(str, err.absolute_path))}: {err.message[:200]}"
            )
        if cfg.get("meta", {}).get("name") != name:
            fail(f"{name}: R14 meta.name != directory name")
        for qpath, node in iter_quantities(cfg):
            if node is None:
                fail(f"{name}: bare number at {qpath} (must be a quantity)")
            elif node["assumption"] is not True:
                fail(f"{name}: R15 {qpath} not flagged assumption = true")
        if cfg.get("meta", {}).get("status") == "COMPLETE" and not errors:
            check_complete_config(cfg, name, path.parent, validator)
    return n


# ---- spec cross-checks ---------------------------------------------------------------------


def md_sections(text: str) -> dict[str, list[list[list[str]]]]:
    """Heading text -> list of tables; each table is a list of rows (header row first)."""
    out: dict[str, list] = {}
    heading = ""
    table: list[list[str]] | None = None
    for line in text.splitlines():
        stripped = line.strip()
        if line.startswith("#"):
            heading = line.lstrip("#").strip()
            out.setdefault(heading, [])
            table = None
            continue
        if stripped.startswith("|"):
            cells = [c.strip() for c in stripped.strip("|").split("|")]
            if set("".join(cells)) <= set("-: "):
                continue
            if table is None:
                table = []
                out.setdefault(heading, []).append(table)
            table.append(cells)
        else:
            table = None
    return out


def find_table(sections, heading_prefix: str, index: int = 0):
    for h, tables in sections.items():
        if h.startswith(heading_prefix):
            if len(tables) <= index:
                fail(f"spec: no table #{index} under '{heading_prefix}'")
                return []
            return tables[index]
    fail(f"spec: heading '{heading_prefix}' not found")
    return []


def resolve(prop: dict, common: dict) -> tuple[str, str, str | None]:
    enum_name = None
    target = {}
    if "$ref" in prop:
        ref_name = prop["$ref"].split("/")[-1]
        target = common["$defs"][ref_name]
        if "enum" in target:
            enum_name = ref_name
    x_type = prop.get("x-type", target.get("x-type"))
    x_unit = prop.get("x-unit", target.get("x-unit"))
    return x_type, x_unit, enum_name


def strip_ticks(s: str) -> str:
    return s.replace("`", "").strip()


def check_entity_tables(sections, schemas) -> None:
    common = schemas["common.schema.json"]
    entities = schemas["entities.schema.json"]["$defs"]
    for heading in sections:
        m = re.match(r"(\d+\.\d+(?:\.\d+)?) (\w+)$", heading)
        if not m or not heading.startswith("2.") or m.group(2) not in entities:
            continue
        name = m.group(2)
        table = sections[heading][0] if sections[heading] else []
        rows = {r[0]: r for r in table[1:]}
        props = entities[name]["properties"]
        if set(rows) != set(props):
            fail(
                f"spec 2 {name}: fields differ from schema: spec-only {set(rows) - set(props)}, schema-only {set(props) - set(rows)}"
            )
        for field, row in rows.items():
            if field not in props:
                continue
            x_type, x_unit, enum_name = resolve(props[field], common)
            parts = row[1].split()
            if (
                parts[0] != x_type
                or (parts[1] if len(parts) > 1 else None) != enum_name
            ):
                fail(
                    f"spec 2 {name}.{field}: type '{row[1]}' != schema '{x_type} {enum_name or ''}'"
                )
            if row[2] != x_unit:
                fail(f"spec 2 {name}.{field}: unit '{row[2]}' != schema '{x_unit}'")
    for name in entities:
        if not any(re.match(rf"2\.\d+(\.\d+)? {name}$", h) for h in sections):
            fail(f"spec 2: entity {name} has no table")


def check_events(sections, schemas) -> None:
    ev = schemas["events.schema.json"]
    defs = ev["$defs"]
    kinds = ev["properties"]["kind"]["enum"]
    seen = set()
    for row in find_table(sections, "4.2 Messages")[1:]:
        name, direction, fields = row[0], row[1], strip_ticks(row[2])
        seen.add(name)
        d = defs.get(name)
        if d is None or d["x-class"] != "MESSAGE":
            fail(f"spec 4.2: {name} is not a MESSAGE kind in schema")
            continue
        if direction != f"{d['x-from']} → {d['x-to']}":
            fail(
                f"spec 4.2: {name} direction '{direction}' != {d['x-from']} → {d['x-to']}"
            )
        if [f.strip() for f in fields.split(",")] != d["required"]:
            fail(f"spec 4.2: {name} fields '{fields}' != {d['required']}")
    for row in find_table(sections, "5.1 Events")[1:]:
        name, cls, fields = row[0], row[1], strip_ticks(row[2])
        seen.add(name)
        d = defs.get(name)
        if d is None or d["x-class"] != cls:
            fail(f"spec 5.1: {name} class '{cls}' does not match schema")
            continue
        if [f.strip() for f in fields.split(",")] != d["required"]:
            fail(f"spec 5.1: {name} fields '{fields}' != {d['required']}")
    if seen != set(kinds):
        fail(
            f"spec 4.2/5.1 vs schema kinds: spec-only {seen - set(kinds)}, schema-only {set(kinds) - seen}"
        )


def check_state_machine(sections, schemas) -> None:
    enum = schemas["common.schema.json"]["$defs"]["VisitState"]["enum"]
    states = [r[0] for r in find_table(sections, "3.3 Visit states")[1:]]
    if states != enum:
        fail(f"spec 3.3 states {states} != VisitState {enum}")
    ids = set()
    for heading, min_cols in (
        ("3.1 Pad mode", 8),
        ("3.2 Pods on spots", 6),
        ("3.4 Visit transitions", 8),
        ("4.4 Drone side", 7),
    ):
        table = find_table(sections, heading)
        header = table[0] if table else []
        for row in table[1:]:
            if len(row) != len(header) or len(row) < min_cols:
                fail(
                    f"spec {heading}: row {row[0]} has {len(row)} cells, header has {len(header)}"
                )
                continue
            if row[0] in ids:
                fail(f"spec {heading}: duplicate transition id {row[0]}")
            ids.add(row[0])
            for col, cell in zip(header, row):
                if not cell:
                    fail(f"spec {heading}: {row[0]} empty '{col}'")
            if "Owner" in header and not row[header.index("Owner")]:
                fail(f"spec {heading}: {row[0]} has no owner")
            msg = row[header.index("Messages")]
            if not re.search(r"`[A-Z_]+`|silence|re-sent|none needed", msg):
                fail(f"spec {heading}: {row[0]} names no message: '{msg}'")
        if heading.startswith("3.") and table:
            for row in table[1:]:
                for state in re.findall(r"\b[A-Z_]{4,}\b", row[1] + " " + row[2]):
                    if (
                        state not in enum
                        and state
                        not in ("ONLINE", "CONTROLLER_FAULT", "OUT_OF_SERVICE")
                        and heading.startswith("3.4")
                    ):
                        fail(f"spec 3.4: {row[0]} uses unknown state {state}")
    branches = [r[0] for r in find_table(sections, "3.6 Failure branches")[1:]]
    if len(branches) != 7:
        fail(f"spec 3.6: expected 7 failure branches, found {len(branches)}")


def config_keys(schemas) -> dict[str, str]:
    common = schemas["common.schema.json"]["$defs"]
    keys: dict[str, str] = {}

    def unit_of(node):
        ref = node.get("$ref", "").split("/")[-1]
        if ref.startswith("q_"):
            return common[ref]["properties"]["unit"]["const"]
        return "-"

    def walk(node, path):
        if node.get("type") == "object" and "properties" in node:
            for k, v in node["properties"].items():
                walk(v, f"{path}.{k}" if path else k)
        elif node.get("type") == "array" and "items" in node:
            item = node["items"]
            if item.get("type") == "object":
                walk(item, f"{path}[]")
            elif "$ref" in item and item["$ref"].split("/")[-1].startswith("q_"):
                keys[f"{path}[]"] = unit_of(item)
            else:
                keys[path] = "-"
        else:
            keys[path] = unit_of(node)

    walk(schemas["config.schema.json"], "")
    return keys


def inline_code(text: str) -> list[str]:
    """Inline `code` spans, ignoring fenced blocks whose triple backticks break pairing."""
    return re.findall(r"`([^`\n]+)`", re.sub(r"```.*?```", "", text, flags=re.DOTALL))


def check_config_keys(sections, schemas, texts: dict[str, str]) -> None:
    keys = config_keys(schemas)
    table = {r[0]: r[1] for r in find_table(sections, "8.2 Keys")[1:]}
    if set(table) != set(keys):
        fail(
            f"spec 8.2 vs config schema: spec-only {set(table) - set(keys)}, schema-only {set(keys) - set(table)}"
        )
    for k, unit in table.items():
        if k in keys and keys[k] != unit:
            fail(f"spec 8.2: {k} unit '{unit}' != schema '{keys[k]}'")
    sections_re = r"(?:meta|sim|geo|network_files|pads\[\]|lanes\[\]|drone|payload|charging|pad_ops|handshake|policy|demand|truck|metrics|faults\[\])\."
    token_re = re.compile(
        r"(?<![/\w.])" + sections_re + r"[a-z_]+(?:\[\])?(?:\.[a-z_]+)*"
    )
    for doc, text in texts.items():
        for span in inline_code(text):
            for token in token_re.findall(span):
                table_prefix = token.endswith("[]") and any(
                    k.startswith(token + ".") for k in keys
                )
                if token not in keys and not table_prefix:
                    fail(f"{doc}: config key reference `{token}` does not exist")


def check_tokens(schemas, texts: dict[str, str]) -> None:
    common = schemas["common.schema.json"]["$defs"]
    known = {"NONE", "ALERT"}
    for d in common.values():
        known.update(d.get("enum", []))
    known.update(schemas["events.schema.json"]["properties"]["kind"]["enum"])
    for doc, text in texts.items():
        for ref in (
            c for c in inline_code(text) if re.fullmatch(r"[A-Z][A-Z0-9_]{2,}", c)
        ):
            if ref not in known:
                fail(f"{doc}: `{ref}` is not a known enum value, event or message")


def check_paper_run(schemas, text: str) -> None:
    kinds = set(schemas["events.schema.json"]["properties"]["kind"]["enum"])
    sections = md_sections(text)
    steps = 0
    for heading, tables in sections.items():
        for table in tables:
            header = table[0]
            if "Owner" not in header or "Message / event" not in header:
                continue
            for row in table[1:]:
                steps += 1
                owner = row[header.index("Owner")]
                msg = row[header.index("Message / event")]
                if not owner or not msg:
                    fail(f"paper-run {heading} step {row[0]}: missing owner or message")
                names = re.findall(r"`([A-Z_]+)`", msg)
                if not names:
                    fail(f"paper-run {heading} step {row[0]}: no message/event named")
                for n in names:
                    if n not in kinds and n != "ALERT":
                        fail(
                            f"paper-run {heading} step {row[0]}: unknown message/event {n}"
                        )
    if steps == 0:
        fail("paper-run: no step tables found")


def check_forbidden_words() -> None:
    for path in (
        list((ROOT / "docs").rglob("*.md"))
        + list(SCHEMAS.glob("*.json"))
        + list((ROOT / "configs").rglob("*.toml"))
    ):
        for i, line in enumerate(path.read_text().splitlines(), 1):
            if FORBIDDEN_WORDS.search(line):
                fail(
                    f"{path.relative_to(ROOT)}:{i}: forbidden placeholder word: {line.strip()[:80]}"
                )


def main() -> int:
    schemas, registry = load_schemas()
    n_cfg = check_configs(schemas, registry)
    spec_text = SPEC.read_text()
    paper_text = PAPER_RUN.read_text() if PAPER_RUN.exists() else ""
    if not paper_text:
        fail("docs/paper-run.md missing")
    sections = md_sections(spec_text)
    check_entity_tables(sections, schemas)
    check_events(sections, schemas)
    check_state_machine(sections, schemas)
    texts = {"docs/spec.md": spec_text, "docs/paper-run.md": paper_text}
    check_config_keys(sections, schemas, texts)
    check_tokens(schemas, texts)
    if paper_text:
        check_paper_run(schemas, paper_text)
    check_forbidden_words()
    if errors:
        print(f"FAIL: {len(errors)} problem(s)")
        for e in errors:
            print("  " + e)
        return 1
    print(f"OK: {n_cfg} configs valid, spec and paper-run consistent with schemas")
    return 0


if __name__ == "__main__":
    sys.exit(main())
