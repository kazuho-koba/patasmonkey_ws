"""RViz用の単一画像grid。投影・TF処理は通常mapperを共用する。"""

import numpy as np

from pm_perception.rolling_elevation_grid import RollingElevationGrid


class SingleFrameGrid(RollingElevationGrid):
    """画像間の状態を捨て、同一画像内のXY集約だけを行う。"""

    def __init__(self, *args, single_frame_obstacle=True, **kwargs):
        super().__init__(*args, **kwargs)
        self.single_frame_obstacle = single_frame_obstacle

    def clear_frame(self):
        """未観測セルにも過去の高さ・障害物を残さず、確保済み配列を再利用する。"""
        slots = np.arange(self.cell_count)
        self._reset_slots(slots, self.world_x.copy(), self.world_y.copy())

    def fuse_points(self, *args, **kwargs):
        """1画像のmin/maxを集約する。confidence待ちの解除は明示時だけ行う。"""
        self.clear_frame()
        count = super().fuse_points(*args, **kwargs)
        if self.single_frame_obstacle:
            # 空gridへの入力なのでheightは現在画像のmax−min[m]になる。
            # 記録閾値未満のセルには証拠を作らず、通常の0.1加算による待ちだけを省く。
            self.obstacle_confidence[self.obstacle_confidence > 0] = 1.0
        return count

    def stage2_layers(self, measurement_variance, current_stamp_ns,
                      obstacle_confidence_min, observation_decay_time,
                      include_accepted_ground_age=False):
        """one-shot時はofflineの--single-frame-obstacle同様confidence gateを省く。"""
        layers = super().stage2_layers(
            measurement_variance, current_stamp_ns,
            0.0 if self.single_frame_obstacle else obstacle_confidence_min,
            observation_decay_time, include_accepted_ground_age,
        )
        if self.single_frame_obstacle:
            # confidence閾値をゼロにしても、記録閾値未満のゼロ高さを既知へ変えない。
            layers["obstacle_height"][layers["obstacle_confidence"] <= 0] = np.nan
        return layers
