"""独立ROS domainでmission/debug記録とmetadata確定を確認する手動smoke。

カメラ・IMU・motor等は起動せず、synthetic RobotModel/debug messageだけを発行する。
Foxy overlayをsourceした開発コンテナ内でpython3 recording_smoke.pyを実行する。
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def fixture(root):
    """core manifestを模擬し、遅延参加のtransient_local geometryも発行する。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid
    rclpy.init()
    node = Node('pm_core_manifest')
    node.declare_parameter('history_depth',3)
    node.declare_parameter('manifest_json',json.dumps({
        'launch':'synthetic_fixture','arguments':{'motor':'false'},
        'runtime_directory':str(root/'runtime'),'configuration_paths':[str(root/'config.yaml')]}))
    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    model = node.create_publisher(String,'/robot_description',qos)
    grid = node.create_publisher(OccupancyGrid,'/depth_elevation_mapper/elevation_debug',10)
    model.publish(String(data='<robot name="synthetic"/>'))
    def tick():
        message = OccupancyGrid()
        message.info.width = message.info.height = 1
        message.info.resolution = .1
        message.data = [0]
        grid.publish(message)
    node.create_timer(.2,tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    """記録だけを実行し、Ctrl-C正常停止とprovenanceを検証する。"""
    root = Path(tempfile.mkdtemp(prefix='pm_recording_smoke_'))
    (root/'runtime').mkdir()
    (root/'runtime'/'fixture.txt').write_text('synthetic artifact\n')
    (root/'config.yaml').write_text('history_depth: 3\n')
    env = dict(os.environ,ROS_DOMAIN_ID='197')
    publisher = subprocess.Popen([sys.executable,__file__,'--fixture',str(root)],env=env)
    launches = []
    try:
        time.sleep(2)
        for profile in ('mission','debug'):
            stream = (root/(profile+'.log')).open('w')
            launch = subprocess.Popen(['ros2','launch','pm_bringup',profile+'_bag.launch.py',
                'bag_directory:='+str(root), 'bag_name:='+profile, 'storage:=mcap',
                'workspace:=/workspaces/patasmonkey_ws','external_workspace:=/workspaces/ros2_ws'],
                env=env,stdout=stream,stderr=subprocess.STDOUT)
            launches.append((profile,launch,stream))
        deadline = time.monotonic()+90
        while time.monotonic()<deadline:
            file = root/'mission/provenance/effective_parameters.json'
            if file.is_file() and '/pm_core_manifest' in json.loads(file.read_text())['snapshots']:
                break
            if any(p.poll() is not None for _,p,_ in launches):
                raise RuntimeError('recorder exited early: '+str(root))
            time.sleep(1)
        else:
            raise RuntimeError('parameter snapshot timeout: '+str(root))
        time.sleep(3)
    finally:
        for profile,launch,stream in launches:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
            launch.wait(timeout=120)
            stream.close()
        publisher.send_signal(signal.SIGINT)
        publisher.wait(timeout=10)
    for profile,_,_ in launches:
        completion = json.loads((root/profile/'completion.json').read_text())
        assert completion['verified'],completion
    import yaml
    topics = {}
    for profile in ('mission','debug'):
        metadata = yaml.safe_load((root/profile/'metadata.yaml').read_text())
        topics[profile] = {row['topic_metadata']['name']:row['message_count'] for row in
                          metadata['rosbag2_bagfile_information']['topics_with_message_count']}
    assert topics['mission']['/robot_description']==1
    assert topics['debug']['/depth_elevation_mapper/elevation_debug']>0
    assert '/depth_elevation_mapper/elevation_debug' not in topics['mission']
    params = json.loads((root/'mission/provenance/effective_parameters.json').read_text())
    assert params['snapshots']['/pm_core_manifest']['values']['history_depth']==3
    assert (root/'mission/provenance/runtime_artifacts/fixture.txt').is_file()
    assert (root/'mission/provenance/versions.json').is_file()
    print('SMOKE PASSED: '+str(root))


if __name__ == '__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--fixture':
        fixture(Path(sys.argv[2]))
    else:
        main()
