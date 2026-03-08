from __future__ import annotations

import argparse
from pathlib import Path

from layout_model import (
    DEFAULT_BACK_PROFILE,
    DEFAULT_CHESTPLATE_PROFILE,
    DEFAULT_FRONT_PROFILE,
    build_back_layout,
    build_chestplate_layout,
    build_front_layout,
    make_profile,
    parse_count_overrides,
    save_layout_file,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the LED layout file for the SD card."
    )
    parser.add_argument(
        "--scope",
        choices=("front", "back", "chestplate"),
        default="chestplate",
        help="which geometry to export",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="layout text output path",
    )
    parser.add_argument(
        "--profile-name",
        default=None,
        help="profile name written into the layout metadata",
    )
    parser.add_argument(
        "--density",
        type=float,
        default=None,
        help="fallback LEDs-per-unit for sections without explicit overrides",
    )
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="SECTION=COUNT",
        help="override a section LED count",
    )
    return parser.parse_args()


def default_output_path(scope: str) -> Path:
    script_dir = Path(__file__).resolve().parent
    return script_dir / f"led_layout_{scope}_v2.txt"


def default_profile_name(scope: str) -> str:
    if scope == "front":
        return DEFAULT_FRONT_PROFILE.name
    if scope == "back":
        return DEFAULT_BACK_PROFILE.name
    return DEFAULT_CHESTPLATE_PROFILE.name


def default_density(scope: str) -> float:
    if scope == "front":
        return DEFAULT_FRONT_PROFILE.default_density
    if scope == "back":
        return DEFAULT_BACK_PROFILE.default_density
    return DEFAULT_CHESTPLATE_PROFILE.default_density


def default_base_profile(scope: str):
    if scope == "front":
        return DEFAULT_FRONT_PROFILE
    if scope == "back":
        return DEFAULT_BACK_PROFILE
    return DEFAULT_CHESTPLATE_PROFILE


def build_sections(scope: str, profile):
    if scope == "front":
        return build_front_layout(profile)
    if scope == "back":
        return build_back_layout(profile)
    return build_chestplate_layout(profile)


def main() -> None:
    args = parse_args()
    overrides = parse_count_overrides(args.overrides)
    profile = make_profile(
        base_profile=default_base_profile(args.scope),
        name=args.profile_name or default_profile_name(args.scope),
        density=args.density if args.density is not None else default_density(args.scope),
        overrides=overrides,
    )
    sections = build_sections(args.scope, profile)
    output_path = save_layout_file(args.output or default_output_path(args.scope), sections, profile)
    print(output_path)


if __name__ == "__main__":
    main()
