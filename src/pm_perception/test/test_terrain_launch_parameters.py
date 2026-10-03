"""launchのYAML優先順・引数上書き・表示閾値整合を確認する。"""
from pathlib import Path

import pytest
from launch import LaunchContext
from pm_perception.terrain_launch_parameters import TUNING_PARAMETERS, terrain_tuning_overrides


def resolve(**kwargs):
    context = LaunchContext()
    context.launch_configurations.update({name: "" for name in TUNING_PARAMETERS})
    context.launch_configurations.update(kwargs)
    config = Path(__file__).resolve().parents[1] / "config"
    return terrain_tuning_overrides(context, [
        config / "depth_elevation_mapper_hazard_0p05_baseline.yaml",
        config / "depth_elevation_mapper_old_bag.yaml",
        config / "mapper_tf_trace.yaml"])


def test_yaml_defaults():
    values = resolve()
    assert values["pixel_stride"] == 4
    assert values["resolution"] == 0.1
    assert values["debug_obstacle_height_max"] == 0.08


def test_arguments_override_and_display_sync():
    values = resolve(pixel_stride="2", hazard_slope_limit_deg="15.0",
                     hazard_step_limit="0.05", hazard_roughness_limit="0.02")
    assert values["pixel_stride"] == 2
    assert values["debug_slope_max_deg"] == 15.0
    assert values["debug_step_height_max"] == 0.05
    assert values["debug_roughness_max"] == 0.02


@pytest.mark.parametrize("args", [
    {"pixel_stride": "0"}, {"pixel_stride": "1.5"},
    {"hazard_step_limit": "-0.1"}, {"hazard_slope_limit_deg": "nan"}])
def test_invalid_settings_rejected(args):
    with pytest.raises(ValueError):
        resolve(**args)
