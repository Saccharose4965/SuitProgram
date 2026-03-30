from __future__ import annotations

import argparse
from pathlib import Path

from .layout_io import generated_layout, load_layout_file
from .preview_model import build_preview_frame
from .preview_svg import render_preview_svg
from .project_model import load_project


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render one simulated show frame to SVG.")
    parser.add_argument("project", type=Path, help="project JSON file")
    parser.add_argument("output", type=Path, help="output SVG path")
    parser.add_argument("--layout", type=Path, default=None, help="optional led_layout.txt file")
    parser.add_argument("--role", choices=("A", "B", "C"), default="A")
    parser.add_argument("--time-ms", type=int, default=0, help="timeline position in milliseconds")
    parser.add_argument("--projection", choices=("flat", "iso"), default="flat")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    project = load_project(args.project)
    layout = load_layout_file(args.layout) if args.layout else generated_layout()
    frame = build_preview_frame(project, layout, args.role, args.time_ms)
    output = render_preview_svg(layout, frame, args.output, projection=args.projection)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
