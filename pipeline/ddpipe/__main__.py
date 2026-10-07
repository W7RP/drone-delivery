"""Entry point: `python -m ddpipe [fetch|build|check|all]` (default all)."""

import argparse
import logging
import sys

from . import fetch


def main() -> int:
    parser = argparse.ArgumentParser(prog="ddpipe")
    parser.add_argument("step", nargs="?", default="all", choices=["fetch", "all"])
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    fetch.fetch_all()
    return 0


if __name__ == "__main__":
    sys.exit(main())
