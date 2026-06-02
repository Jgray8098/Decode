#!/usr/bin/env python3
"""
Convert Pedro visualizer JSON path arrays into exported .pp project files.

Usage:
  python json_to_pedro_pp.py pedro_paths_blue_far.json
  python json_to_pedro_pp.py pedro_paths_blue_far.json pedro_paths_red_far.json
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

VERSION = "1.2.1"
PALETTE = [
    "#ffc516",
    "#5B9C9A",
    "#857B75",
    "#C9DB67",
    "#60a5fa",
    "#ff6b6b",
    "#f97316",
]


def _require_waypoint(path_name: str, waypoint: dict[str, Any], key: str) -> float:
    if key not in waypoint:
        raise ValueError(f"Path '{path_name}' waypoint is missing '{key}'")
    return float(waypoint[key])


def convert(paths: list[dict[str, Any]], chain_name: str) -> dict[str, Any]:
    if not paths:
        raise ValueError("Input JSON has no paths")

    first = paths[0]
    first_wps = first.get("waypoints", [])
    if len(first_wps) < 2:
        raise ValueError("First path must have at least 2 waypoints")

    first_start = first_wps[0]
    first_end = first_wps[-1]

    lines: list[dict[str, Any]] = []
    sequence: list[dict[str, str]] = []
    line_ids: list[str] = []

    for idx, path in enumerate(paths, start=1):
        name = str(path.get("name", f"Path {idx}"))
        wps = path.get("waypoints", [])
        if len(wps) < 2:
            raise ValueError(f"Path '{name}' must have at least 2 waypoints")

        start_wp = wps[0]
        end_wp = wps[-1]
        line_id = f"line-{idx:03d}"

        line = {
            "id": line_id,
            "name": name,
            "endPoint": {
                "x": _require_waypoint(name, end_wp, "x"),
                "y": _require_waypoint(name, end_wp, "y"),
                "heading": "linear",
                "startDeg": _require_waypoint(name, start_wp, "heading"),
                "endDeg": _require_waypoint(name, end_wp, "heading"),
            },
            "controlPoints": [],
            "color": PALETTE[(idx - 1) % len(PALETTE)],
            "locked": False,
            "waitBeforeMs": 0,
            "waitAfterMs": 0,
            "waitBeforeName": "",
            "waitAfterName": "",
        }

        lines.append(line)
        sequence.append({"kind": "path", "lineId": line_id})
        line_ids.append(line_id)

    return {
        "startPoint": {
            "x": _require_waypoint(str(first.get("name", "Path 1")), first_start, "x"),
            "y": _require_waypoint(str(first.get("name", "Path 1")), first_start, "y"),
            "heading": "linear",
            "startDeg": _require_waypoint(str(first.get("name", "Path 1")), first_start, "heading"),
            "endDeg": _require_waypoint(str(first.get("name", "Path 1")), first_end, "heading"),
            "locked": False,
        },
        "lines": lines,
        "shapes": [],
        "sequence": sequence,
        "pathChains": [
            {
                "id": f"chain-{chain_name}",
                "name": chain_name,
                "color": "#C9DB67",
                "lineIds": line_ids,
            }
        ],
        "version": VERSION,
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
    }


def convert_file(json_path: Path) -> Path:
    with json_path.open("r", encoding="utf-8") as f:
        paths = json.load(f)

    if not isinstance(paths, list):
        raise ValueError(f"Expected top-level list in {json_path.name}")

    chain_name = json_path.stem.replace("_", " ").title()
    project = convert(paths, chain_name)

    out_path = json_path.with_suffix(".pp")
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(project, f, indent=2)
        f.write("\n")

    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert pedro .json paths to .pp format")
    parser.add_argument("json_files", nargs="+", type=Path, help="Input JSON files")
    args = parser.parse_args()

    for json_file in args.json_files:
        out_file = convert_file(json_file)
        print(f"Wrote {out_file}")


if __name__ == "__main__":
    main()

