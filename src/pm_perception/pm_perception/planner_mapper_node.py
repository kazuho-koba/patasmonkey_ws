"""従来nodeのdepth購読・撮像時刻TFを再利用する、明示選択式planner producer。"""
from nav_msgs.msg import OccupancyGrid
from pm_msgs.msg import TerrainAssessment
from pm_perception.depth_elevation_mapper_node import DepthElevationMapper, main, yaw_from_quaternion
from pm_perception.rolling_elevation_grid import RollingElevationGrid
from pm_perception.independent_obstacles import IndependentObstacles


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
