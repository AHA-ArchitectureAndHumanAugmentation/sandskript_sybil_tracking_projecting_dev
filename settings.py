import json
from pathlib import Path

_SETTINGS_FILE = Path("settings.json")


def load_settings() -> dict:
    try:
        with open(_SETTINGS_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_settings(data: dict) -> None:
    existing = load_settings()
    existing.update(data)
    with open(_SETTINGS_FILE, "w") as f:
        json.dump(existing, f, indent=2)
