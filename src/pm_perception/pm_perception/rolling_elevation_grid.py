"""固定allocationのrobot-centric rolling elevation grid。

array indexはring bufferを構成する。各slotは現在対応する絶対odom-grid座標を保持するため、
map移動時は論理originだけを更新する。window外へ出たcellはslot再利用時にlazy resetする。
"""

import math

import numpy as np


UNASSIGNED = np.iinfo(np.int64).min


class RollingElevationGrid:
    """ground統計量と将来拡張可能な最小obstacle fieldを保持する。"""

    def __init__(self, size_x, size_y, resolution):
        if size_x <= 0.0 or size_y <= 0.0 or resolution <= 0.0:
            raise ValueError("map dimensions and resolution must be positive")
        self.resolution = float(resolution)
        self.width = int(round(size_x / resolution))
        self.height = int(round(size_y / resolution))
        self.cell_count = self.width * self.height
        self.origin_cell_x = 0
        self.origin_cell_y = 0

        # Structure-of-arraysによりhot updateを連続配置し、frameごとのfusion処理を
        # NumPyでvectorize可能にする。
        self.world_x = np.full(self.cell_count, UNASSIGNED, dtype=np.int64)
        self.world_y = np.full(self.cell_count, UNASSIGNED, dtype=np.int64)
        self.elevation = np.zeros(self.cell_count, dtype=np.float32)
        self.elevation_m2 = np.zeros(self.cell_count, dtype=np.float32)
        # 観測分散の逆数和。observation_countとは意図的に分離する。countは解釈しやすい
        # 生の観測数のままとし、weightだけがcellの古さに応じて減衰する。
        self.elevation_weight = np.zeros(self.cell_count, dtype=np.float32)
        # このlayerは各画像timestampにおけるcameraの期待ground高さを差し引く。odomに
        # common-mode z offsetがあっても、暫定camera高さでの水平面は概ねゼロになる。
        self.relative_elevation = np.zeros(self.cell_count, dtype=np.float32)
        self.relative_elevation_m2 = np.zeros(self.cell_count, dtype=np.float32)
        self.relative_elevation_weight = np.zeros(
            self.cell_count, dtype=np.float32
        )
        self.observation_count = np.zeros(self.cell_count, dtype=np.uint32)
        self.last_observed_ns = np.zeros(self.cell_count, dtype=np.int64)

        # Stage 1では単純なvertical extentだけを埋める。ここでfieldを持たせることで、
        # Stage 2でrolling-gridのownership modelを変更せずに済む。
        self.obstacle_height = np.zeros(self.cell_count, dtype=np.float32)
        self.obstacle_confidence = np.zeros(self.cell_count, dtype=np.float32)
        self.last_obstacle_observed_ns = np.zeros(
            self.cell_count, dtype=np.int64)

        self._frame_min = np.full(self.cell_count, np.inf, dtype=np.float32)
        self._frame_max = np.full(self.cell_count, -np.inf, dtype=np.float32)
        self._frame_variance = np.zeros(self.cell_count, dtype=np.float32)
        self._frame_world_x = np.zeros(self.cell_count, dtype=np.int64)
        self._frame_world_y = np.zeros(self.cell_count, dtype=np.int64)

    @property
    def origin_x(self):
        return self.origin_cell_x * self.resolution

    @property
    def origin_y(self):
        return self.origin_cell_y * self.resolution

    def recenter(self, center_x, center_y):
        center_cell_x = math.floor(center_x / self.resolution)
        center_cell_y = math.floor(center_y / self.resolution)
        self.origin_cell_x = center_cell_x - self.width // 2
        self.origin_cell_y = center_cell_y - self.height // 2

    def _reset_slots(self, slots, world_x, world_y):
        self.world_x[slots] = world_x
        self.world_y[slots] = world_y
        self.elevation[slots] = 0.0
        self.elevation_m2[slots] = 0.0
        self.elevation_weight[slots] = 0.0
        self.relative_elevation[slots] = 0.0
        self.relative_elevation_m2[slots] = 0.0
        self.relative_elevation_weight[slots] = 0.0
        self.observation_count[slots] = 0
        self.last_observed_ns[slots] = 0
        self.obstacle_height[slots] = 0.0
        self.obstacle_confidence[slots] = 0.0
        self.last_obstacle_observed_ns[slots] = 0

    def fuse_points(
        self,
        x,
        y,
        z,
        stamp_ns,
        ground_merge_threshold,
        obstacle_min_height,
        observation_variance=None,
        relative_elevation_offset=0.0,
        observation_decay_time=0.0,
    ):
        """frameをXY cellごとに集約し、cellごとにground sampleを1個fusionする。

        cell内の最小sampleを暫定ground観測、最大sampleを最小限のobstacle-height cueに
        用いる。完全なground classifierではないが、垂直方向のreturnすべてを意味のない1高さに
        平均することを防ぐ。観測分散はcellごとに保守的に縮約（最大point分散を使用）し、
        逆分散fusion weightに変換する。
        """
        world_x = np.floor(x / self.resolution).astype(np.int64)
        world_y = np.floor(y / self.resolution).astype(np.int64)
        inside = (
            (world_x >= self.origin_cell_x)
            & (world_x < self.origin_cell_x + self.width)
            & (world_y >= self.origin_cell_y)
            & (world_y < self.origin_cell_y + self.height)
        )
        if not np.any(inside):
            return 0

        world_x = world_x[inside]
        world_y = world_y[inside]
        z = np.asarray(z[inside], dtype=np.float32)
        if observation_variance is None:
            observation_variance = np.ones(z.shape, dtype=np.float32)
        else:
            observation_variance = np.asarray(
                observation_variance[inside], dtype=np.float32
            )
        # moduloで無限に広がるodom-cell座標を固定arrayへ写像する。下の`world_x/world_y`
        # tagにより、使用中slotと偶然同じmodulo indexを共有する古いcellを区別する。
        slots = (
            np.mod(world_y, self.height) * self.width
            + np.mod(world_x, self.width)
        ).astype(np.intp)

        # まずこの画像のsample点をXY cellごとのmin/max一組へ縮約する。これによりfusionは
        # samplingした全depth pixelではなく、touchしたcell数に対してO()で実行できる。
        self._frame_min.fill(np.inf)
        self._frame_max.fill(-np.inf)
        self._frame_variance.fill(0.0)
        np.minimum.at(self._frame_min, slots, z)
        np.maximum.at(self._frame_max, slots, z)
        np.maximum.at(self._frame_variance, slots, observation_variance)
        self._frame_world_x[slots] = world_x
        self._frame_world_y[slots] = world_y
        observed = np.flatnonzero(np.isfinite(self._frame_min))
        observed_x = self._frame_world_x[observed]
        observed_y = self._frame_world_y[observed]

        # rolling originの移動後、slotが別の絶対odom cellへ対応していることがある。古い
        # elevationが新しい物理位置へ漏れないよう、使用前にresetする。
        reused = (
            (self.world_x[observed] != observed_x)
            | (self.world_y[observed] != observed_y)
        )
        if np.any(reused):
            self._reset_slots(
                observed[reused], observed_x[reused], observed_y[reused]
            )

        sample = self._frame_min[observed]
        sample_variance = np.maximum(
            self._frame_variance[observed], np.finfo(np.float32).eps
        )
        sample_weight = 1.0 / sample_variance
        count = self.observation_count[observed]
        current = self.elevation[observed]
        empty = count == 0
        # 明確に低いminimumは、2つのsurfaceを平均せずground hypothesisを置き換える。垂直差は
        # weak obstacle evidenceとして残すが、視点やdepthのartifactである可能性もある。
        lower = (~empty) & (sample < current - ground_merge_threshold)
        merge = (~empty) & (np.abs(sample - current) <= ground_merge_threshold)

        initialize = empty | lower
        existing = ~empty
        # confidence/weightは今回touchしたcellだけで減衰する。publish時にもageを評価するため、
        # 周期的な全map mutationなしに古いevidenceの影響を弱められる。
        if observation_decay_time > 0.0 and np.any(existing):
            existing_slots = observed[existing]
            age_seconds = np.maximum(
                0.0,
                (stamp_ns - self.last_observed_ns[existing_slots]).astype(
                    np.float64
                ) / 1e9,
            )
            decay = np.exp(-age_seconds / observation_decay_time).astype(
                np.float32
            )
            self.elevation_weight[existing_slots] *= decay
            self.elevation_m2[existing_slots] *= decay
            self.relative_elevation_weight[existing_slots] *= decay
            self.relative_elevation_m2[existing_slots] *= decay
            self.obstacle_confidence[existing_slots] *= decay
        if np.any(lower):
            lower_slots = observed[lower]
            self.obstacle_height[lower_slots] = np.maximum(
                self.obstacle_height[lower_slots], current[lower] - sample[lower]
            )
            self.obstacle_confidence[lower_slots] = np.minimum(
                1.0, self.obstacle_confidence[lower_slots] + 0.2
            )
            self.last_obstacle_observed_ns[lower_slots] = stamp_ns
        if np.any(initialize):
            init_slots = observed[initialize]
            self.elevation[init_slots] = sample[initialize]
            self.elevation_m2[init_slots] = 0.0
            self.elevation_weight[init_slots] = sample_weight[initialize]
            self.relative_elevation[init_slots] = (
                sample[initialize] + relative_elevation_offset
            )
            self.relative_elevation_m2[init_slots] = 0.0
            self.relative_elevation_weight[init_slots] = sample_weight[initialize]
            self.observation_count[init_slots] = 1

        if np.any(merge):
            # weighted Welford型update。画像やdepth点の履歴を保存せず、meanとweighted
            # second momentを保持する。
            merge_slots = observed[merge]
            old_weight = self.elevation_weight[merge_slots]
            weight = sample_weight[merge]
            new_weight = old_weight + weight
            delta = sample[merge] - self.elevation[merge_slots]
            new_mean = self.elevation[merge_slots] + delta * weight / new_weight
            self.elevation_m2[merge_slots] += (
                weight * delta * (sample[merge] - new_mean)
            )
            self.elevation[merge_slots] = new_mean
            self.elevation_weight[merge_slots] = new_weight

            relative_sample = sample[merge] + relative_elevation_offset
            relative_old_weight = self.relative_elevation_weight[merge_slots]
            relative_new_weight = relative_old_weight + weight
            relative_delta = (
                relative_sample - self.relative_elevation[merge_slots]
            )
            relative_new_mean = self.relative_elevation[merge_slots] + (
                relative_delta * weight / relative_new_weight
            )
            self.relative_elevation_m2[merge_slots] += (
                weight * relative_delta * (relative_sample - relative_new_mean)
            )
            self.relative_elevation[merge_slots] = relative_new_mean
            self.relative_elevation_weight[merge_slots] = relative_new_weight
            self.observation_count[merge_slots] += np.uint32(1)

        # この画像の最大returnをfusion済みground estimateと比較する。これはvertical extent
        # cueにすぎず、semantic判定やray-occlusion推論を含まない。
        ground = self.elevation[observed]
        height = self._frame_max[observed] - ground
        obstacle = height >= obstacle_min_height
        if np.any(obstacle):
            obstacle_slots = observed[obstacle]
            self.obstacle_height[obstacle_slots] = np.maximum(
                self.obstacle_height[obstacle_slots], height[obstacle]
            )
            self.obstacle_confidence[obstacle_slots] = np.minimum(
                1.0, self.obstacle_confidence[obstacle_slots] + 0.1
            )
            self.last_obstacle_observed_ns[obstacle_slots] = stamp_ns

        self.last_observed_ns[observed] = stamp_ns
        return int(observed.size)

    def logical_layers(self, measurement_variance):
        """publish用にcopyした、odom向きrow-major viewを返す。"""
        xs = self.origin_cell_x + np.arange(self.width, dtype=np.int64)
        ys = self.origin_cell_y + np.arange(self.height, dtype=np.int64)
        world_x, world_y = np.meshgrid(xs, ys)
        # odom-cell昇順で連続した論理mapを再構成する。これは低rate publishのためだけに行う
        # 意図的な全grid copyである。
        slots = (
            np.mod(world_y, self.height) * self.width
            + np.mod(world_x, self.width)
        ).astype(np.intp)
        valid = (
            (self.world_x[slots] == world_x)
            & (self.world_y[slots] == world_y)
            & (self.observation_count[slots] > 0)
        )
        elevation = np.full((self.height, self.width), np.nan, dtype=np.float32)
        variance = np.full_like(elevation, np.nan)
        count = np.zeros((self.height, self.width), dtype=np.uint32)
        age_ns = np.zeros((self.height, self.width), dtype=np.int64)
        elevation[valid] = self.elevation[slots][valid]
        count[valid] = self.observation_count[slots][valid]
        age_ns[valid] = self.last_observed_ns[slots][valid]
        weight = self.elevation_weight[slots]
        spread = np.zeros_like(weight, dtype=np.float32)
        weighted = weight > np.finfo(np.float32).eps
        spread[weighted] = self.elevation_m2[slots][weighted] / weight[weighted]
        variance[valid] = np.maximum(
            measurement_variance,
            1.0 / np.maximum(weight[valid], np.finfo(np.float32).eps)
            + spread[valid],
        )
        return elevation, variance, count, age_ns

    def stage2_layers(
        self,
        measurement_variance,
        current_stamp_ns,
        obstacle_confidence_min,
        observation_decay_time,
    ):
        """論理map順でpublish可能なStage 2 layerを返す。

        `age_seconds`とobstacle confidenceはpublish時に評価する。このため、周期的な全grid
        mutationをせずに未観測mapの信頼度を自然に下げられる。
        """
        elevation, variance, count, last_observed_ns = self.logical_layers(
            measurement_variance
        )
        xs = self.origin_cell_x + np.arange(self.width, dtype=np.int64)
        ys = self.origin_cell_y + np.arange(self.height, dtype=np.int64)
        world_x, world_y = np.meshgrid(xs, ys)
        slots = (
            np.mod(world_y, self.height) * self.width
            + np.mod(world_x, self.width)
        ).astype(np.intp)
        valid = count > 0
        relative = np.full_like(elevation, np.nan)
        relative[valid] = self.relative_elevation[slots][valid]
        age_seconds = np.full_like(elevation, np.nan)
        age_seconds[valid] = np.maximum(
            0.0,
            (current_stamp_ns - last_observed_ns[valid]).astype(np.float64) / 1e9,
        )
        obstacle_age = np.maximum(
            0.0,
            (current_stamp_ns - self.last_obstacle_observed_ns[slots]).astype(
                np.float64
            ) / 1e9,
        )
        obstacle_confidence = self.obstacle_confidence[slots] * np.exp(
            -obstacle_age / max(1e-6, observation_decay_time)
        )
        obstacle_height = np.full_like(elevation, np.nan)
        obstacle_valid = (
            valid & (obstacle_confidence >= obstacle_confidence_min)
        )
        obstacle_height[obstacle_valid] = self.obstacle_height[slots][
            obstacle_valid
        ]
        return {
            "elevation": elevation,
            "relative_elevation": relative,
            "variance": variance,
            "count": count,
            "age_seconds": age_seconds,
            "obstacle_height": obstacle_height,
            "obstacle_confidence": obstacle_confidence,
        }
