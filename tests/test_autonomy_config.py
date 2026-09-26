"""Unit tests for autonomy configuration subsystem."""

import os
from unittest.mock import patch
from autonomy.config import AutonomyConfig, load_autonomy_config


def test_default_config():
    config = AutonomyConfig()
    assert not config.enabled
    assert config.mode == "hybrid"
    assert config.safety.command_ttl_ms == 500.0
    assert config.safety.control_rate_hz == 10.0
    assert config.safety.max_slope_deg == 18.0
    assert config.network.disconnect_timeout_s == 3.0
    assert config.sensors.camera_fps == 30


def test_load_from_yaml(tmp_path):
    yaml_content = """
enabled: true
mode: edge
safety:
  command_ttl_ms: 300.0
  max_slope_deg: 15.0
network:
  disconnect_timeout_s: 2.0
"""
    yaml_file = tmp_path / "test_autonomy.yaml"
    yaml_file.write_text(yaml_content, encoding="utf-8")

    config = load_autonomy_config(str(yaml_file))
    assert config.enabled
    assert config.mode == "edge"
    assert config.safety.command_ttl_ms == 300.0
    assert config.safety.max_slope_deg == 15.0
    assert config.network.disconnect_timeout_s == 2.0
    # non-overridden fields retain defaults
    assert config.safety.control_rate_hz == 10.0


def test_env_var_overrides(tmp_path):
    yaml_file = tmp_path / "autonomy.yaml"
    yaml_file.write_text("enabled: false\nmode: remote\n", encoding="utf-8")

    env_vars = {
        "AUTONOMY_ENABLED": "true",
        "AUTONOMY_MODE": "edge",
        "AUTONOMY_COMMAND_TTL_MS": "250.0",
        "AUTONOMY_MAX_SLOPE_DEG": "16.5",
    }
    with patch.dict(os.environ, env_vars):
        config = load_autonomy_config(str(yaml_file))
        assert config.enabled is True
        assert config.mode == "edge"
        assert config.safety.command_ttl_ms == 250.0
        assert config.safety.max_slope_deg == 16.5
