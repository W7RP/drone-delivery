"""Step 1: download every source in sources.yaml into data/raw/ once, and keep a manifest of
what was fetched (URL, retrieval date, license, sha256). Cached files are re-hashed on every run
and never fetched again, so a rebuild needs no network."""

import datetime
import hashlib
import json
import logging
import urllib.request
from pathlib import Path

import yaml

from .paths import MANIFEST, PIPELINE, RAW
from .provenance import pipeline_revision

log = logging.getLogger(__name__)

RAW_BUDGET_BYTES = 2 * 1024**3
CHUNK_BYTES = 1 << 20
TIMEOUT_S = 120


def load_sources() -> dict:
    with open(PIPELINE / "sources.yaml") as f:
        return yaml.safe_load(f)


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(CHUNK_BYTES):
            h.update(chunk)
    return h.hexdigest()


# Leading bytes each downloaded file type must start with; catches an HTML error page saved under
# a data file's name.
MAGIC = {
    ".pbf": b"\x00\x00\x00",
    ".tif": b"II*\x00",
    ".zip": b"PK\x03\x04",
    ".geojson": b"{",
    ".csv": b"station,",
}

# The only CSV source is the IEM ASOS archive, with the fields its URL asks for.
CSV_HEADER = "station,valid,sknt,drct,gust,tmpf,vsby,p01i,wxcodes"


def _check_content(path: Path, suffix: str) -> None:
    with open(path, "rb") as f:
        head = f.read(8)
    if not head.startswith(MAGIC[suffix]):
        raise RuntimeError(f"{path.name}: starts with {head!r}, not a {suffix} file")
    if suffix == ".geojson":
        doc = json.loads(path.read_text())
        truncated = doc.get("exceededTransferLimit") or doc.get("properties", {}).get(
            "exceededTransferLimit"
        )
        if doc.get("type") != "FeatureCollection" or truncated:
            raise RuntimeError(f"{path.name}: not a complete FeatureCollection")
    if suffix == ".csv":
        with open(path) as f:
            header = f.readline().strip()
        if header != CSV_HEADER:
            raise RuntimeError(f"{path.name}: header {header!r}, expected {CSV_HEADER!r}")


def _download(url: str, dest: Path) -> None:
    part = dest.with_suffix(dest.suffix + ".part")
    req = urllib.request.Request(url, headers={"User-Agent": "drone-delivery-pipeline"})
    with urllib.request.urlopen(req, timeout=TIMEOUT_S) as resp, open(part, "wb") as out:
        if resp.status != 200:
            raise RuntimeError(f"{url}: HTTP {resp.status}")
        while chunk := resp.read(CHUNK_BYTES):
            out.write(chunk)
    _check_content(part, dest.suffix)
    part.rename(dest)


def _read_manifest() -> dict:
    if MANIFEST.exists():
        return json.loads(MANIFEST.read_text())
    return {"files": {}}


def fetch_all() -> dict:
    """Return the manifest after making sure every source is present and matches it."""
    sources = load_sources()["sources"]
    RAW.mkdir(parents=True, exist_ok=True)
    manifest = _read_manifest()
    files = manifest["files"]
    for s in sources:
        path = RAW / s["file"]
        entry = files.get(s["name"])
        if entry and path.exists():
            digest = sha256_of(path)
            _check_content(path, path.suffix)
            if digest != entry["sha256"]:
                raise RuntimeError(
                    f"{path}: sha256 {digest} does not match the manifest; delete the file "
                    "and its manifest entry to fetch it again"
                )
            log.info("cached %s", s["file"])
            continue
        log.info("fetching %s", s["url"])
        _download(s["url"], path)
        files[s["name"]] = {
            "file": s["file"],
            "url": s["url"],
            "license": s["license"],
            "use": s["use"],
            "retrieved_utc": datetime.datetime.now(datetime.UTC).date().isoformat(),
            "bytes": path.stat().st_size,
            "sha256": sha256_of(path),
            "pipeline_git": pipeline_revision(),
        }
        MANIFEST.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    total = sum(e["bytes"] for e in files.values())
    if total > RAW_BUDGET_BYTES:
        raise RuntimeError(f"raw data is {total} bytes, over the 2 GB budget")
    log.info("raw data: %d files, %.0f MB", len(files), total / 1e6)
    return manifest
