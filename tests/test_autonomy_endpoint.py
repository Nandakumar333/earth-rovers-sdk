"""Integration test for /autonomy/status HTTP endpoint."""

import json
import pytest
import main


@pytest.mark.asyncio
async def test_autonomy_status_endpoint_returns_json():
    response = await main.get_autonomy_status()
    assert response.status_code == 200
    data = json.loads(response.body)
    assert "enabled" in data
    assert "state" in data
    assert "mode" in data
    assert "metrics" in data
    assert "recent_events" in data
