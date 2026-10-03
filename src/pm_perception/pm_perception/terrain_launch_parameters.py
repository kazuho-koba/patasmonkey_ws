"""目視確認・TF採取launchで共有する、起動時だけのterrain設定解決。

YAMLの優先順を維持し、非空のlaunch引数だけで上書きする。component表示100を
hazard限界に一致させるため、表示スケールは解決後の限界へ自動追従する。
"""
import math

import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


TUNING_PARAMETERS = {
    "resolution": (float, "セル幅[m]"),
    "pixel_stride": (int, "両pixel軸の間引き幅。1は全pixel、4は約1/16"),
    "obstacle_min_height": (float, "obstacle証拠の最低記録高さ[m]"),
    "hazard_slope_limit_deg": (float, "slopeによるhazard限界[deg]"),
    "hazard_roughness_limit": (float, "roughnessによるhazard限界[m]"),
    "hazard_step_limit": (float, "stepによるhazard限界[m]"),
    "hazard_obstacle_height_limit": (float, "obstacleによるhazard限界[m]"),
}


def declare_tuning_arguments():
    """空の引数はYAMLを尊重する。既存の起動コマンドの設定を変えない。"""
    return [DeclareLaunchArgument(name, default_value="", description=description+"。空ならYAML値")
            for name, (_, description) in TUNING_PARAMETERS.items()]


def terrain_tuning_overrides(context, config_paths):
    """基本→old bag→専用YAML→launch引数の順に、7つの設定を解決する。"""
    values = {}
    for path in config_paths:
        with open(path) as stream:
            config = yaml.safe_load(stream)
        node = (config.get("depth_elevation_mapper") or config.get("/depth_elevation_mapper")
                or config.get("/**"))
        if node is None:
            raise ValueError("depth_elevation_mapper設定がありません: "+str(path))
        values.update(node["ros__parameters"])
    resolved = {}
    for name, (value_type, _) in TUNING_PARAMETERS.items():
        text = LaunchConfiguration(name).perform(context).strip()
        raw = text if text else values[name]
        value = value_type(raw)
        # YAMLの小数strideを整数へ切り捨てず、設定ミスとして起動時に拒否する。
        if name == "pixel_stride" and float(raw) != value:
            raise ValueError("pixel_strideは整数で指定してください")
        if not math.isfinite(value) or (value < 0 if name == "obstacle_min_height" else value <= 0):
            raise ValueError("不正なterrain設定: "+name)
        resolved[name] = value
    # YAMLでhazard限界だけを編集しても、componentの黒表示を同じ限界に揃える。
    for limit, display in [("hazard_slope_limit_deg", "debug_slope_max_deg"),
                           ("hazard_roughness_limit", "debug_roughness_max"),
                           ("hazard_step_limit", "debug_step_height_max"),
                           ("hazard_obstacle_height_limit", "debug_obstacle_height_max")]:
        resolved[display] = resolved[limit]
    # Foxyではnode名付きYAMLとdict由来の/**設定を混在させると、YAML側が
    # 優先される場合がある。全設定を一つのdictへ統合して上書きを確実にする。
    values.update(resolved)
    return values
