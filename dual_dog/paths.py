"""Resolve resources relative to the installed project, not the working directory."""

from pathlib import Path
import sys

BUNDLE_ROOT = Path(__file__).resolve().parents[1]
APP_ROOT = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else BUNDLE_ROOT
LOCAL_ROOT = APP_ROOT / "_local"
DEMO_CONFIG = BUNDLE_ROOT / "config" / "robots.json"
REAL_CONFIG = LOCAL_ROOT / "config" / "robots.json"


def asset_path(name: str) -> Path:
    """Prefer a local asset when present and fall back to the bundled demo."""
    for directory in (LOCAL_ROOT / "assets", APP_ROOT / "assets"):
        candidate = directory / name
        if candidate.is_file():
            return candidate
    return BUNDLE_ROOT / "assets" / name
