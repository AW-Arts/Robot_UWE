"""Utilities for managing motor inversion state.

The :class:`MotorInversionManager` provides an abstraction around a
configuration file that stores the inversion state of each motor.  It is
intended to be used by both the hardware control loop and any visualisation
layer so that both systems agree on the sign convention of each joint.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional


@dataclass
class MotorState:
    """Representation of a single motor and its inversion state."""

    identifier: str
    label: str
    inverted: bool = False

    @classmethod
    def from_mapping(cls, data: Mapping[str, object]) -> "MotorState":
        try:
            identifier = str(data["id"])
        except KeyError as exc:  # pragma: no cover - guard clause
            raise KeyError("Motor configuration entry missing 'id' field") from exc
        label = str(data.get("label", identifier))
        inverted = bool(data.get("inverted", False))
        return cls(identifier=identifier, label=label, inverted=inverted)

    def to_dict(self) -> Dict[str, object]:
        return {"id": self.identifier, "label": self.label, "inverted": self.inverted}


class MotorInversionManager:
    """Manage the inversion state of a set of motors.

    Parameters
    ----------
    config_path:
        Path to the JSON configuration file that stores the motor
        definitions.  The file is created automatically if it does not exist
        and ``default_motors`` is provided.
    default_motors:
        Optional iterable of dictionaries with ``id`` and ``label`` keys. The
        ``inverted`` flag defaults to ``False``.  Used only when the
        configuration file does not exist yet.
    autosave:
        When ``True`` (default) any mutation persists the configuration back
        to disk immediately.  This keeps the visualisation and hardware in
        sync even if the process terminates unexpectedly.
    """

    def __init__(
        self,
        config_path: Path | str,
        *,
        default_motors: Optional[Iterable[Mapping[str, object]]] = None,
        autosave: bool = True,
    ) -> None:
        self._config_path = Path(config_path)
        self._autosave = autosave
        self._motors: Dict[str, MotorState] = {}

        if not self._config_path.exists():
            if default_motors is None:
                raise FileNotFoundError(
                    f"Motor configuration file '{self._config_path}' does not exist"
                )
            self._motors = {
                entry["id"]: MotorState.from_mapping(entry) for entry in default_motors
            }
            self.save()
        else:
            self._load()

    # ------------------------------------------------------------------
    # Private helpers
    def _load(self) -> None:
        data = json.loads(self._config_path.read_text())
        motors = data.get("motors", [])
        if not isinstance(motors, list):
            raise ValueError("The 'motors' entry in the configuration must be a list")
        self._motors = {}
        for entry in motors:
            if not isinstance(entry, Mapping):
                raise ValueError("Motor configuration entries must be objects")
            state = MotorState.from_mapping(entry)
            self._motors[state.identifier] = state

    def _autosave_if_enabled(self) -> None:
        if self._autosave:
            self.save()

    # ------------------------------------------------------------------
    # Public API
    @property
    def config_path(self) -> Path:
        """Return the path to the backing configuration file."""

        return self._config_path

    def motors(self) -> List[MotorState]:
        """Return the list of motors sorted by their identifier."""

        return [self._motors[key] for key in sorted(self._motors)]

    def get(self, motor_id: str) -> MotorState:
        try:
            return self._motors[motor_id]
        except KeyError as exc:
            raise KeyError(f"Unknown motor '{motor_id}'") from exc

    def is_inverted(self, motor_id: str) -> bool:
        return self.get(motor_id).inverted

    def set_inverted(self, motor_id: str, inverted: bool) -> None:
        state = self.get(motor_id)
        state.inverted = bool(inverted)
        self._autosave_if_enabled()

    def toggle(self, motor_id: str) -> bool:
        state = self.get(motor_id)
        state.inverted = not state.inverted
        self._autosave_if_enabled()
        return state.inverted

    def apply(self, motor_id: str, value: float) -> float:
        """Apply the inversion state to the provided control value."""

        if self.is_inverted(motor_id):
            return -value
        return value

    def save(self) -> None:
        data = {"motors": [state.to_dict() for state in self._motors.values()]}
        self._config_path.parent.mkdir(parents=True, exist_ok=True)
        self._config_path.write_text(json.dumps(data, indent=2))

    def as_dict(self) -> Dict[str, object]:
        return {"motors": [state.to_dict() for state in self.motors()]}
