"""
Unit tests for docker/docker-compose.yml — the committed ROS bridge definition.

Nothing here starts Docker. The point is that the compose file and config.py
describe the SAME bridge: the app connects to a port and a namespace, the
containers publish a port and a namespace, and a silent disagreement between
them looks exactly like a robot that is switched off.

Plain text checks rather than a YAML parse, so the tests add no dependency.
"""
from pathlib import Path

import pytest

from config import RRC_NAMESPACE, RRC_ROS_PORT

COMPOSE = Path(__file__).resolve().parents[1] / "docker" / "docker-compose.yml"


@pytest.fixture(scope="module")
def compose_text() -> str:
    assert COMPOSE.is_file(), f"missing {COMPOSE}"
    return COMPOSE.read_text(encoding="utf-8")


class TestBridgeCompose:

    def test_the_three_services_are_defined(self, compose_text):
        for service in ("ros-master:", "ros-bridge:", "abb-driver:"):
            assert service in compose_text

    def test_the_driver_image_is_pinned(self, compose_text):
        # The whole reason this file is committed instead of borrowed from the
        # compas_rrc repo: an upstream release must not be able to change what
        # the installation runs.
        assert "compasrrc/compas_rrc_driver:v1.1.2" in compose_text
        assert "compas_rrc_driver:latest" not in compose_text

    def test_the_websocket_port_matches_config(self, compose_text):
        assert f'"{RRC_ROS_PORT}:{RRC_ROS_PORT}"' in compose_text

    def test_the_namespace_matches_config(self, compose_text):
        # config says "/rob1"; roslaunch takes it without the slash.
        assert f"namespace:={RRC_NAMESPACE.lstrip('/')}" in compose_text

    def test_rosbridge_keeps_a_quiet_connection_alive(self, compose_text):
        # The ~10 s default unregisters an idle connection and the link dies on
        # its own — which for an installation waiting between visitors is the
        # normal case. 28800 s = 8 h.
        assert "unregister_timeout:=28800" in compose_text

    def test_the_robot_ip_is_the_only_robot_address(self, compose_text):
        # The app never sees the robot's IP; it connects to the bridge. This is
        # the one place the address is entered.
        assert "robot_ip:=192.168.125.1" in compose_text
