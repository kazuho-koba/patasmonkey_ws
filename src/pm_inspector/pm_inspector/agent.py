"""Jetson上のbag解析受付。停止済みbagへの明示要求だけを実行する。"""
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pm_msgs.srv import InspectBags
from pm_inspector.jobs import Jobs


class Agent(Node):
    """解析を子processへ分離し、CoreやRobot Managerには依存しない。"""
    def __init__(self):
        super().__init__('pm_inspection_agent')
        self.declare_parameter('bag_root', str(Path.home()/'patasmonkey_ws/bags'))
        self.declare_parameter('result_root', str(Path.home()/'.ros/pm_inspector'))
        self.declare_parameter('service', '/pm/inspector/request')
        self.jobs = Jobs(self.get_parameter('bag_root').value,
                         self.get_parameter('result_root').value)
        self.active = {}
        for profile in ('mission', 'debug'):
            self.create_subscription(String, '/pm/robot_manager/'+profile+'_bag/status',
                                     self.recording_status, 10)
        self.create_service(InspectBags, self.get_parameter('service').value, self.request)

    def recording_status(self, message):
        try:
            data = json.loads(message.data)
            self.active[data['profile']] = data
        except (ValueError, KeyError):
            pass

    def request(self, request, response):
        try:
            if len(request.payload) > 65536:
                raise ValueError('要求が大きすぎます')
            payload = json.loads(request.payload or '{}')
            if not isinstance(payload, dict):
                raise ValueError('payloadはJSON objectが必要です')
            if request.operation == 'start':
                targets = {str(self.jobs.resolve(b)) for b in payload['bags']}
                for data in self.active.values():
                    if (data.get('state', '').upper() not in ('STOPPED', 'ERROR')
                            and data.get('output') in targets):
                        raise ValueError('記録中または保存処理中のbagは解析できません')
            result = self.jobs.dispatch(request.operation, payload)
            encoded = json.dumps(result, ensure_ascii=False, allow_nan=False)
            if len(encoded.encode('utf-8')) > 512*1024:
                raise ValueError('結果が大きすぎます。対象bag/topicを絞ってください')
            response.success, response.payload = True, encoded
        except Exception as error:
            response.success = False
            response.payload = json.dumps(dict(error=str(error)), ensure_ascii=False)
        return response


def main(args=None):
    # 管理要求と集計結果だけをUDPで送り、画像用SHMの状態に依存させない。
    from ament_index_python.packages import get_package_share_directory
    os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(
        Path(get_package_share_directory('pm_inspector'))/'config/control_dds_udp.xml'))
    rclpy.init(args=args)
    node = Agent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.jobs.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
