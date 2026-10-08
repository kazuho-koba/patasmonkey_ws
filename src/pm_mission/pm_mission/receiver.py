"""Jetson用ミッション受領・保存node。走行関連interfaceは呼び出さない。"""
import hashlib
import json
from pathlib import Path
import uuid

import rclpy
from rclpy.node import Node
from pm_msgs.srv import UploadMission
from .model import from_message, load_document, save_document


class MissionReceiver(Node):
    """永続保存完了後にだけ受領成功を応答する。executorとは独立した受信箱。"""

    def __init__(self):
        super().__init__('mission_receiver')
        self.declare_parameter('upload_service', '/pm/mission/upload')
        self.declare_parameter('output_directory', '~/.local/share/pm_mission/received')
        self.directory = Path(self.get_parameter('output_directory').value).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.create_service(UploadMission, self.get_parameter('upload_service').value, self.upload)
        self.get_logger().info('ミッション受信待機（走行は開始しません）: '+str(self.directory))

    def upload(self, request, response):
        """request IDと内容hashで再送を識別し、同じIDで違う経路の上書きを拒否する。"""
        response.request_id = request.request_id
        response.mission_id = request.mission.mission_id
        try:
            request_id = str(uuid.UUID(request.request_id))
            document = from_message(request.mission)
            digest = hashlib.sha256(json.dumps(document, sort_keys=True).encode()).hexdigest()
            destination = self.directory / (request_id+'_'+digest+'.yaml')
            conflicts = list(self.directory.glob(request_id+'_*.yaml'))
            if conflicts and destination not in conflicts:
                raise ValueError('同じrequest IDで異なるミッションが送られました')
            if destination.exists():
                if load_document(destination) != document:
                    raise ValueError('保存済みミッションの内容が一致しません')
            else:
                save_document(destination, document)
            response.accepted = True
            response.stored_path = str(destination)
            response.message = 'UGV側への保存完了。走行は開始していません。'
            self.get_logger().info('ミッション受領: '+str(destination))
        except (OSError, ValueError, KeyError, TypeError) as error:
            response.accepted = False
            response.message = '受領失敗: '+str(error)
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
