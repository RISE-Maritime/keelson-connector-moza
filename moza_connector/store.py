"""The settings file: what the operator set, kept between runs (``model.saved_state``).

I/O only; what is kept and how it is checked on the way back is ``model.saved_state`` /
``model.load_state``. A file that cannot be read or written is logged and otherwise
ignored: the wheel works without it.
"""

from __future__ import annotations

import json
import logging
import os
import sys
import tempfile
from pathlib import Path

log = logging.getLogger(__name__)


def default_path(realm: str, entity_id: str, source_id: str) -> Path:
    """One file per realm, entity and source, in the platform's place for app settings."""
    if sys.platform == "darwin":
        base = Path.home() / "Library" / "Application Support" / "moza2keelson"
    else:
        base = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config") / "moza2keelson"
    name = "-".join(part.replace("/", "_") for part in (realm, entity_id, source_id))
    return base / f"{name}.json"


def read(path: Path) -> dict | None:
    """The file's document, or None when there is none or it cannot be read."""
    try:
        return json.loads(path.read_text())
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as e:
        log.warning("Settings file %s unreadable (%s): starting from the command line", path, e)
        return None


def write(path: Path, doc: dict) -> bool:
    """Write atomically (a temporary file renamed over the old one). Never raises."""
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
        with os.fdopen(fd, "w") as f:
            json.dump(doc, f, indent=2, sort_keys=True)
        os.replace(tmp, path)
        return True
    except OSError as e:
        log.warning("Settings not saved to %s: %s", path, e)
        return False
