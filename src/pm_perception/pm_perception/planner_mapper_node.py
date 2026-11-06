"""従来nodeのdepth購読・撮像時刻TFを再利用する、明示選択式planner producer。"""
from nav_msgs.msg import OccupancyGrid
from pm_msgs.msg import TerrainAssessment
from pm_perception.depth_elevation_mapper_node import DepthElevationMapper, main, yaw_from_quaternion
from pm_perception.rolling_elevation_grid import RollingElevationGrid
from pm_perception.independent_obstacles import IndependentObstacles
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
import math


class PlannerGrid(RollingElevationGrid):
    """親の投影経路だけ再利用し、高さの時間融合は行わない。"""
    def fuse_points(self,x,y,z,stamp,*args,**kwargs):
        import numpy as np
        return self.model.update(np.column_stack((x,y,z)),self.camera,self.center,stamp,self.yaw,
                                 *self.limits)


class PlannerMapper(DepthElevationMapper):
    def __init__(self):
        super().__init__()
        if self.forensic_enabled:
            raise ValueError('新producerのforensic保存は未対応です')
        options={name:self.declare_parameter(name,default).value for name,default in
                 (('confirmation_window',3),('clear_min_points',3),
                  ('clear_count_ratio',.75),('clear_ground_tolerance',.03))}
        grid=self.grid
        self.grid=PlannerGrid(grid.width*grid.resolution,grid.height*grid.resolution,grid.resolution)
        if grid.width!=grid.height:raise ValueError('新producerは正方形gridを使用してください')
        self.grid.model=IndependentObstacles(grid.resolution,grid.width*grid.resolution,
            self.hazard_obstacle_height_limit,options['confirmation_window'],options['clear_min_points'],
            options['clear_count_ratio'],options['clear_ground_tolerance'])
        self.grid.limits=(self.hazard_slope_limit_deg,self.hazard_roughness_limit,self.hazard_step_limit)
        self.assessment=self.create_publisher(TerrainAssessment,'/depth_elevation_mapper/planner_costmap',1)
        self.obstacle_map=self.create_publisher(OccupancyGrid,'~/obstacle_costmap',1)
        self.terrain_map=self.create_publisher(OccupancyGrid,'~/terrain_costmap',1)
        self.last_published=-1
        # GUIの原子的変更だけを受け付ける。TF・QoSの変更は購読の再生成が必要なため
        # ここでは対象外。地形の意味が変わる変更では既存mapを全消去する。
        self.add_on_set_parameters_callback(self.reconfigure)
        self.create_service(Trigger, '~/reset_map', self.reset_map)

    def reconfigure(self, changes):
        """単位・組合せを検証後、同じexecutor内でgridと処理条件を切り替える。"""
        bounds = dict(map_size_x=(1.,30.), map_size_y=(1.,30.), resolution=(.05,.5),
            min_depth=(.1,10.), max_depth=(.2,20.), pixel_stride=(1,16),
            hazard_obstacle_height_limit=(.01,2.), confirmation_window=(1,30),
            clear_min_points=(1,1000), clear_count_ratio=(.01,1.),
            clear_ground_tolerance=(.001,1.), hazard_slope_limit_deg=(1.,80.),
            hazard_roughness_limit=(.001,1.), hazard_step_limit=(.001,1.))
        relevant = [p for p in changes if p.name != 'use_sim_time']
        if not relevant:return SetParametersResult(successful=True)
        values = {name:self.get_parameter(name).value for name in bounds}
        try:
            for p in relevant:
                if p.name not in bounds:raise ValueError(p.name+'は実行中変更に未対応')
                lo, hi = bounds[p.name]
                if isinstance(p.value,bool) or not isinstance(p.value,(int,float)) or not math.isfinite(p.value) or not lo<=p.value<=hi:
                    raise ValueError(p.name+'が許容範囲外')
                if isinstance(values[p.name],int) and not isinstance(p.value,int):
                    raise ValueError(p.name+'は整数が必要')
                values[p.name] = p.value
            if values['min_depth']>=values['max_depth']:raise ValueError('min_depth < max_depthが必要')
            if abs(values['map_size_x']-values['map_size_y'])>1e-9:raise ValueError('mapは正方形が必要')
            grid = self.make_grid(values)
        except (ValueError,TypeError) as exc:
            return SetParametersResult(successful=False,reason=str(exc))
        for name in ('min_depth','max_depth','pixel_stride','hazard_obstacle_height_limit',
                     'hazard_slope_limit_deg','hazard_roughness_limit','hazard_step_limit'):
            setattr(self,name,values[name])
        self.grid=grid; self.pending.clear(); self.last_published=-1
        self.get_logger().info('認識設定変更: mapをリセットしました')
        return SetParametersResult(successful=True)

    def make_grid(self, values):
        """実効設定から新しい空mapを生成する。寸法の丸めは既存gridに揃える。"""
        grid=PlannerGrid(values['map_size_x'], values['map_size_y'], values['resolution'])
        grid.model=IndependentObstacles(grid.resolution,grid.width*grid.resolution,
            values['hazard_obstacle_height_limit'],values['confirmation_window'],
            values['clear_min_points'],values['clear_count_ratio'],values['clear_ground_tolerance'])
        grid.limits=(values['hazard_slope_limit_deg'],values['hazard_roughness_limit'],values['hazard_step_limit'])
        return grid

    def reset_map(self, request, response):
        """自己位置原点は変えず、観測結果だけを破棄する。次の画像から再生成する。"""
        names=('map_size_x','map_size_y','resolution','hazard_obstacle_height_limit',
               'confirmation_window','clear_min_points','clear_count_ratio','clear_ground_tolerance',
               'hazard_slope_limit_deg','hazard_roughness_limit','hazard_step_limit')
        self.grid=self.make_grid({name:self.get_parameter(name).value for name in names})
        self.pending.clear();self.last_published=-1
        response.success=True;response.message='mapを消去。次のdepthを待ちます'
        return response

    def process_frame(self,message,intrinsics,camera_tf,base_tf,*args,**kwargs):
        """TFは親が画像stampで取得済み。latestへの置換はしない。"""
        t=camera_tf.transform.translation;b=base_tf.transform.translation
        self.grid.camera=(t.x,t.y,t.z);self.grid.center=(b.x,b.y)
        self.grid.yaw=yaw_from_quaternion(base_tf.transform.rotation)
        result=super().process_frame(message,intrinsics,camera_tf,base_tf,*args,**kwargs)
        stamp=message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        if self.last_processed_stamp_ns==stamp and self.grid.model.last_stamp<stamp:
            import numpy as np
            self.grid.model.update(np.empty((0,3)),self.grid.camera,self.grid.center,stamp,self.grid.yaw,
                                   *self.grid.limits)
        return result

    def publish_debug(self,stamp):
        """plannerとRVizへ同じgrid・stampの独立channelを送る。"""
        model=self.grid.model
        if model.last_stamp<=self.last_published:return
        obstacle,terrain=model.costs()
        # 観測停止中に新headerで古いmapを再送し、timeoutを延命させない。
        from builtin_interfaces.msg import Time
        stamp=Time(sec=int(model.last_stamp//1000000000),nanosec=int(model.last_stamp%1000000000))
        maps=[]
        for values,publisher in ((obstacle,self.obstacle_map),(terrain,self.terrain_map)):
            msg=self.make_debug_grid(stamp,values,values>=0,0.,100.)
            msg.info.origin.position.x=float(model.origin[0]);msg.info.origin.position.y=float(model.origin[1])
            publisher.publish(msg);maps.append(msg)
        msg=TerrainAssessment();msg.schema_version=1;msg.header=maps[0].header;msg.info=maps[0].info
        msg.obstacle_cost=maps[0].data;msg.terrain_cost=maps[1].data
        self.assessment.publish(msg);self.last_published=model.last_stamp


def run(args=None):
    main(args=args,node_factory=PlannerMapper)
