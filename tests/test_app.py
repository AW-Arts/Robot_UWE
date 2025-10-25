from pathlib import Path

import pytest

from app import create_app, DEFAULT_MOTORS
from motor_control import MotorInversionManager


@pytest.fixture()
def client(tmp_path: Path):
    config_path = tmp_path / "motors.json"
    MotorInversionManager(config_path, default_motors=DEFAULT_MOTORS)
    app = create_app(config_path)
    app.config.update(TESTING=True)
    with app.test_client() as client:
        yield client


def test_get_motors(client):
    response = client.get("/api/motors")
    assert response.status_code == 200
    data = response.get_json()
    assert isinstance(data["motors"], list)
    assert {motor["id"] for motor in data["motors"]} >= {"base", "gripper"}


def test_toggle_endpoint(client):
    response = client.post("/api/motors/base/toggle")
    assert response.status_code == 200
    payload = response.get_json()
    assert payload == {"id": "base", "inverted": True}

    response = client.get("/api/motors")
    states = {motor["id"]: motor["inverted"] for motor in response.get_json()["motors"]}
    assert states["base"] is True


def test_apply_validation(client):
    response = client.get("/api/motors/base/apply", query_string={"value": "abc"})
    assert response.status_code == 400
