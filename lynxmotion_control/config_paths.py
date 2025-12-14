"""Helpers for locating and seeding user configuration files."""
from __future__ import annotations

import logging
import os
import platform
import shutil
import subprocess
from pathlib import Path

_LOGGER = logging.getLogger(__name__)

PACKAGE_ROOT = Path(__file__).resolve().parent
CONFIG_BUNDLE_DIR = PACKAGE_ROOT / "config_defaults"


def get_config_root() -> Path:
    """Return the OS-appropriate root folder for user configuration.

    macOS: ``~/Library/Application Support/lynxmotion_al5a``
    Windows: ``%APPDATA%\\lynxmotion_al5a`` (falls back to
    ``~/AppData/Roaming/lynxmotion_al5a``)
    Other platforms: ``~/.config/lynxmotion_al5a``
    """

    env_override = os.environ.get("LYNXMOTION_CONFIG_DIR")
    if env_override:
        return Path(env_override)

    system = platform.system().lower()
    if system == "windows":
        appdata = os.environ.get("APPDATA")
        base = Path(appdata) if appdata else Path.home() / "AppData" / "Roaming"
    elif system == "darwin":
        base = Path.home() / "Library" / "Application Support"
    else:
        base = Path.home() / ".config"
    return base / "lynxmotion_al5a"


CONFIG_ROOT = get_config_root()


def seed_default_configs(
    *, bundle_dir: Path | None = None, target_root: Path | None = None
) -> None:
    """Copy bundled defaults into the user's config folder if missing.

    The copy is idempotent: existing user files are never overwritten.
    """

    bundle = bundle_dir or CONFIG_BUNDLE_DIR
    target = target_root or CONFIG_ROOT
    if not bundle.exists() or not bundle.is_dir():
        return

    target.mkdir(parents=True, exist_ok=True)
    for item in bundle.iterdir():
        destination = target / item.name
        if destination.exists():
            continue
        try:
            if item.is_dir():
                shutil.copytree(item, destination)
            elif item.is_file():
                shutil.copy2(item, destination)
        except Exception:  # pragma: no cover - robustness during startup
            _LOGGER.warning(
                "Failed to seed default config %s to %s", item, destination, exc_info=True
            )


def reveal_config_folder(path: Path | None = None) -> None:
    """Open the given folder in the OS file manager."""

    folder = path or CONFIG_ROOT
    folder.mkdir(parents=True, exist_ok=True)
    system = platform.system()
    try:
        if system == "Windows":
            os.startfile(folder)  # type: ignore[attr-defined]
        elif system == "Darwin":
            subprocess.Popen(["open", str(folder)])
        else:
            subprocess.Popen(["xdg-open", str(folder)])
    except Exception:  # pragma: no cover - depends on local OS utilities
        _LOGGER.warning("Could not open config folder %s", folder, exc_info=True)
