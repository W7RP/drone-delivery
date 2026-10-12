#!/usr/bin/env python3
"""Write a copy of a network_files scenario with every demand flow rate multiplied by a factor,
into <out_root>/<scenario name>/scenario.toml (spec R14 needs that directory name). The copy's
network_files.dir points back at the original derived/ files, so both share one network.

Usage: scale_demand.py <scenario.toml> <factor> <out_root>"""

import re
import sys
from pathlib import Path

RATE = re.compile(r"^(rate_per_h = \{ value = )([0-9.eE+-]+)(,.*)$", re.MULTILINE)
DIR = re.compile(r'^dir = "([^"]+)"$', re.MULTILINE)


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__, file=sys.stderr)
        return 2
    src, factor, out_root = Path(sys.argv[1]).resolve(), float(sys.argv[2]), Path(sys.argv[3])
    text = src.read_text()
    text, n_rates = RATE.subn(lambda m: f"{m[1]}{float(m[2]) * factor!r}{m[3]}", text)
    text, n_dirs = DIR.subn(lambda m: f'dir = "{(src.parent / m[1]).resolve()}"', text)
    if n_rates == 0 or n_dirs != 1:
        print(f"{src}: expected demand flows and one network_files.dir", file=sys.stderr)
        return 1
    out = out_root / src.parent.name / "scenario.toml"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
