"""Тесты объединения аварийных событий."""
from uav_telemetry.alerts_logic import merge_alerts


def test_merge_without_lidar():
    assert merge_alerts(['LINK_WEAK'], [], None) == ['LINK_WEAK']


def test_merge_fresh_lidar():
    assert merge_alerts(['LINK_WEAK'], ['OBSTACLE_NEAR'], 0.3) == ['LINK_WEAK', 'OBSTACLE_NEAR']


def test_no_duplicates():
    assert merge_alerts(['OBSTACLE_NEAR'], ['OBSTACLE_NEAR'], 0.1) == ['OBSTACLE_NEAR']


def test_stale_lidar_ignored():
    assert merge_alerts([], ['OBSTACLE_CLOSE'], 5.0) == []
    assert merge_alerts([], ['OBSTACLE_CLOSE'], None) == []
