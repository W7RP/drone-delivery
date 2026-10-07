"""Sample the resident memory of a process tree (the run script and everything it started:
Gazebo, PX4, the XRCE agent, the bridge, the nodes) every half second, and on SIGTERM write the
peak of the sum, and each process's own peak, to a JSON file. Reads /proc only; standard
library, so it runs with any python3.

Usage: rss_monitor.py <root_pid> <out.json>"""

import json
import os
import signal
import sys
import time
from pathlib import Path

PERIOD_S = 0.5
PAGE_KB = os.sysconf("SC_PAGE_SIZE") // 1024


def children() -> dict[int, list[int]]:
    out: dict[int, list[int]] = {}
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            ppid = int((d / "stat").read_text().rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            continue
        out.setdefault(ppid, []).append(int(d.name))
    return out


def tree(root: int) -> list[int]:
    kids = children()
    todo, seen = [root], []
    while todo:
        p = todo.pop()
        seen.append(p)
        todo.extend(kids.get(p, []))
    return seen


def rss_kb(pid: int) -> tuple[str, int]:
    try:
        name = Path(f"/proc/{pid}/comm").read_text().strip()
        pages = int(Path(f"/proc/{pid}/statm").read_text().split()[1])
        return name, pages * PAGE_KB
    except (OSError, IndexError, ValueError):
        return "", 0


def main() -> int:
    root, out = int(sys.argv[1]), Path(sys.argv[2])
    peak_total = 0
    peak_by_name: dict[str, int] = {}
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    while running and Path(f"/proc/{root}").exists():
        total = 0
        by_name: dict[str, int] = {}
        for pid in tree(root):
            if pid == os.getpid():
                continue
            name, kb = rss_kb(pid)
            total += kb
            by_name[name] = by_name.get(name, 0) + kb
        peak_total = max(peak_total, total)
        for name, kb in by_name.items():
            peak_by_name[name] = max(peak_by_name.get(name, 0), kb)
        time.sleep(PERIOD_S)
    out.write_text(json.dumps({"peak_total_mb": round(peak_total / 1024, 1),
                               "peak_by_process_mb": {k: round(v / 1024, 1) for k, v in
                                                      sorted(peak_by_name.items(), key=lambda kv: -kv[1])}},
                              indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
