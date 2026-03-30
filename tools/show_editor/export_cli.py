from __future__ import annotations

import argparse
from pathlib import Path

from .export_show import export_show
from .project_model import load_project


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a compiled music show package.")
    parser.add_argument("project", type=Path, help="path to the source project JSON")
    parser.add_argument("output_dir", type=Path, help="output directory for show.bin and track.wav")
    parser.add_argument(
        "--copy-audio",
        action="store_true",
        help="copy the referenced audio file to track.wav when it exists",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    project = load_project(args.project)
    result = export_show(project, args.output_dir, copy_audio=args.copy_audio)
    print(result.show_bin_path)
    for role, clip_count in sorted(result.role_clip_counts.items()):
        print(f"{role}: clips={clip_count}")
    if result.track_path:
        print(result.track_path)


if __name__ == "__main__":
    main()
