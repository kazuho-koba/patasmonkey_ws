"""表示用MPPIの実入力を保存する独立診断node。通常node・認識閾値は変えない。

solver.stepの直前状態を保存し、障害物だけを除いた反実仮想等を同じ入力・乱数状態で
比較できるようにする。保存を有効化したoffline実行だけに使用する。
"""
import argparse
from dataclasses import asdict
import hashlib
import inspect
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from pm_planner import perception_preview as preview
from pm_planner.mppi_poc import MppiPoc, MppiConfig
from pm_planner.terrain_cost import TerrainCost


def restore_solver(data,config=None):
    """最適化の直前状態を復元。長さを変更した比較だけnominalを同じ末尾値で延長する。"""
    settings=dict(json.loads(str(data['config'])) if config is None else config)
    # 追加前に保存したprobeは当時の評価を維持する。新しいsnapshotには明示値を保存。
    settings.setdefault('clearance_distance_m',0.)
    settings.setdefault('clearance_cost_weight',0.)
    solver=MppiPoc(MppiConfig(**settings));solver.set_path(data['reference'])
    solver.station=float(data['station'])
    nominal=data['nominal'];n=settings['steps']
    solver.nominal=np.vstack((nominal[:n],np.repeat(nominal[-1:],max(0,n-len(nominal)),axis=0)))
    solver.rng.bit_generator.state=json.loads(str(data['rng']))
    solver.required_station=json.loads(str(data['required_station']))
    return solver


def environment(data,values=None):
    """保存時と同じ膨張・map外処理を使用する。unknownを都合よくfreeにしない。"""
    from pm_planner.perception_input import select_costs
    h,w=data['obstacle'].shape
    if values is None:
        values=select_costs(data['obstacle'].ravel(),data['terrain'].ravel(),h*w,str(data['mode']))
    return TerrainCost(np.asarray(values).ravel(),w,h,float(data['resolution']),
                       tuple(data['origin']),float(data['radius']),
                       margin_m=float(data['footprint_margin_m']) if 'footprint_margin_m' in data else None)


class Recorder:
    def __init__(self,path):
        self.path=Path(path);self.path.mkdir(parents=True,exist_ok=False)
        self.stream=(self.path/'events.jsonl').open('x',buffering=1)
        self.node=None;self.count=0

    def before(self,solver,pose,velocity,previous,env):
        """image・ROS接続は複製せず、実際に採用されたmapとsolver状態を対応付ける。"""
        msg=self.node.assessment;h,w=msg.info.height,msg.info.width
        return dict(obstacle=np.asarray(msg.obstacle_cost,dtype=np.int8).reshape(h,w),
            terrain=np.asarray(msg.terrain_cost,dtype=np.int8).reshape(h,w),
            expanded=env.grid.copy(),resolution=env.res,origin=np.asarray(env.origin),
            radius=self.node.get_parameter('footprint_radius_m').value,
            footprint_margin_m=env.margin_m,
            mode=self.node.get_parameter('mode').value,reference=solver.path.copy(),
            nominal=solver.nominal.copy(),station=solver.station,
            config=json.dumps(asdict(solver.c)),rng=json.dumps(solver.rng.bit_generator.state),
            required_station=json.dumps(solver.required_station),pose=np.asarray(pose),
            velocity=np.asarray(velocity),previous=np.asarray(previous if previous is not None else velocity),
            map_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
            pose_stamp_ns=self.node.pose_stamp.sec*10**9+self.node.pose_stamp.nanosec,
            tick_stamp_ns=self.node.get_clock().now().nanoseconds)

    def after(self,data,result,error):
        self.count+=1;data['trajectory']=result['trajectory'] if result else np.empty((0,3))
        file='frame_%04d.npz'%self.count;np.savez_compressed(self.path/file,**data)
        self.stream.write(json.dumps(dict(index=self.count,file=file,map_stamp_ns=int(data['map_stamp_ns']),
            success=result is not None,error=error),ensure_ascii=False)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    args=parser.parse_args(remove_ros_args()[1:]);recorder=Recorder(args.output)
    original=preview.MppiPoc
    class ProbedMppi(MppiPoc):
        def step(self,pose,velocity,previous_command=None,environment=None):
            data=recorder.before(self,pose,velocity,previous_command,environment)
            try:result=super().step(pose,velocity,previous_command,environment)
            except ValueError as exc:
                recorder.after(data,None,str(exc));raise
            recorder.after(data,result,None);return result
    preview.MppiPoc=ProbedMppi
    rclpy.init();node=preview.PerceptionPreview();recorder.node=node
    sources={cls.__name__:inspect.getfile(cls) for cls in (MppiPoc,TerrainCost,preview.PerceptionPreview)}
    manifest=dict(parameters={name:node.get_parameter(name).value for name in
        ('reference_mode','target_speed','prediction_steps','prediction_dt','footprint_radius_m','footprint_margin_m','reference_length_m','use_sim_time')},
        source_sha256={name:hashlib.sha256(Path(path).read_bytes()).hexdigest() for name,path in sources.items()})
    (recorder.path/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        recorder.stream.close();preview.MppiPoc=original;node.destroy_node();rclpy.shutdown()
