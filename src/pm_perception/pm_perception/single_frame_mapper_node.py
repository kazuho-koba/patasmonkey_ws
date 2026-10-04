"""bagを通常mapperと同じTF・intrinsicsで投影する独立フレーム診断node。"""

from pm_perception.depth_elevation_mapper_node import DepthElevationMapper, main, stamp_to_ns
from pm_perception.single_frame_grid import SingleFrameGrid


class SingleFrameMapper(DepthElevationMapper):
    """通常nodeの入出力を共用し、gridの時間方向の記憶だけを無効化する。"""

    def __init__(self):
        super().__init__()
        enabled = self.declare_parameter("single_frame_obstacle", True).value
        grid = self.grid
        self.grid = SingleFrameGrid(
            grid.width * grid.resolution, grid.height * grid.resolution,
            grid.resolution, forensic=self.forensic_enabled,
            single_frame_obstacle=enabled,
        )

    def process_frame(self, message, intrinsics, camera_tf, base_tf,
                      *args, **kwargs):
        # rate gateで棄却される画像は地図を変更しない。受理画像に有効depthがゼロでも
        # 過去画像を残さないため、通常のfuse_points呼び出しより前に空にする。
        stamp_ns = stamp_to_ns(message.header.stamp)
        if (self.last_processed_stamp_ns < 0 or
                stamp_ns - self.last_processed_stamp_ns >= self.minimum_period_ns):
            self.grid.clear_frame()
        return super().process_frame(message, intrinsics, camera_tf, base_tf,
                                     *args, **kwargs)


def run(args=None):
    """通常mainのTF worker・診断・SIGINT終了処理をそのまま利用する。"""
    main(args=args, node_factory=SingleFrameMapper)
