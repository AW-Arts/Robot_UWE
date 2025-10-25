"""Flask application exposing motor inversion controls via HTTP."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

from flask import Flask, jsonify, request, send_from_directory

from motor_control import MotorInversionManager

DEFAULT_CONFIG = Path("config/motors.json")
DEFAULT_MOTORS: List[Dict[str, Any]] = [
    {"id": "base", "label": "Base rotation", "inverted": False},
    {"id": "shoulder", "label": "Shoulder joint", "inverted": False},
    {"id": "elbow", "label": "Elbow joint", "inverted": False},
    {"id": "wrist_pitch", "label": "Wrist pitch", "inverted": False},
    {"id": "wrist_roll", "label": "Wrist roll", "inverted": False},
    {"id": "gripper", "label": "Gripper", "inverted": False},
]


def create_app(config_path: Path | str = DEFAULT_CONFIG) -> Flask:
    config_path = Path(config_path)
    defaults = DEFAULT_MOTORS if not config_path.exists() else None
    manager = MotorInversionManager(config_path, default_motors=defaults)

    app = Flask(__name__, static_folder="static", static_url_path="/static")

    @app.get("/")
    def index() -> Any:
        return send_from_directory(app.static_folder, "index.html")

    @app.get("/api/motors")
    def get_motors() -> Any:
        return jsonify(manager.as_dict())

    @app.post("/api/motors/<motor_id>/toggle")
    def toggle_motor(motor_id: str) -> Any:
        inverted = manager.toggle(motor_id)
        return jsonify({"id": motor_id, "inverted": inverted})

    @app.post("/api/motors/<motor_id>/set")
    def set_motor(motor_id: str) -> Any:
        payload: Dict[str, Any] = request.get_json(force=True, silent=True) or {}
        if "inverted" not in payload:
            return jsonify({"error": "Missing 'inverted' attribute"}), 400
        manager.set_inverted(motor_id, bool(payload["inverted"]))
        return jsonify({"id": motor_id, "inverted": manager.is_inverted(motor_id)})

    @app.get("/api/motors/<motor_id>/apply")
    def apply_to_motor(motor_id: str) -> Any:
        try:
            value = float(request.args.get("value", "0"))
        except ValueError:
            return jsonify({"error": "Query parameter 'value' must be numeric"}), 400
        applied = manager.apply(motor_id, value)
        return jsonify({"id": motor_id, "input": value, "output": applied})

    return app


if __name__ == "__main__":  # pragma: no cover - manual execution helper
    app = create_app()
    app.run(host="0.0.0.0", port=5000, debug=True)
