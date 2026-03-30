from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .main_window import ShowEditorMainWindow
from .project_model import load_project
from .session_state import load_editor_session
from .qt_compat import IMPORT_ERROR, PYSIDE_AVAILABLE, QtCore, QtWidgets


def default_project_path() -> Path:
    return Path(__file__).resolve().parent / "example_project.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the Suit Show Editor scaffold.")
    parser.add_argument(
        "--project",
        type=Path,
        default=None,
        help="project JSON to load at startup; if omitted, resumes the last autosaved session or falls back to the bundled example project",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not PYSIDE_AVAILABLE:
        print(
            "PySide6 is required to run the editor UI scaffold.\n"
            f"Current import error: {IMPORT_ERROR}\n"
            "Install it with: python -m pip install PySide6",
            file=sys.stderr,
        )
        return 1

    QtCore.QLocale.setDefault(QtCore.QLocale.c())
    app = QtWidgets.QApplication(sys.argv)
    startup_session = None
    if args.project is not None:
        project = load_project(args.project)
    else:
        try:
            startup_session = load_editor_session()
        except Exception:
            startup_session = None
        project = startup_session.project if startup_session is not None else load_project(default_project_path())
    window = ShowEditorMainWindow(project, startup_session=startup_session)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
