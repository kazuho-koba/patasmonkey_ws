"""Jetson用ミッション受領・保存node。走行関連interfaceは呼び出さない。"""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import uuid
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from pm_msgs.srv import UploadMission
from std_msgs.msg import String
from .model import from_message, load_document, save_document


class MissionReceiver(Node):
    """永続保存完了後にだけ受領成功を応答する。executorとは独立した受信箱。"""

    def __init__(self, parameter_overrides=None):
        super().__init__('mission_receiver', parameter_overrides=parameter_overrides)
        self.declare_parameter('upload_service', '/pm/mission/upload')
        self.declare_parameter('output_directory', '~/.local/share/pm_mission/received')
        self.directory = Path(self.get_parameter('output_directory').value).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        # 保存済みsnapshotを後から起動した実行管理にも届ける。走行開始は伴わない。
        self.accepted_pub = self.create_publisher(String, '/pm/mission/accepted',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_service(UploadMission, self.get_parameter('upload_service').value, self.upload)
        self.restore_latest()
        self.get_logger().info('ミッション受信待機（走行は開始しません）: '+str(self.directory))

    def publish_saved(self, request_id, digest, destination, document):
        """検証・保存後のsnapshotを遅参加する実行管理とGUIへ渡す。"""
        self.accepted_pub.publish(String(data=json.dumps({
            'request_id': request_id, 'sha256': digest, 'stored_path': str(destination),
            'mission': document}, ensure_ascii=False)))

    def save_latest(self, request_id, digest, destination):
        """最後に受領したfileの参照を同一directoryで原子的に保存する。"""
        pointer = self.directory/'latest_mission.json'
        temporary = None
        try:
            with tempfile.NamedTemporaryFile('w', dir=str(self.directory), delete=False,
                                             encoding='utf-8') as stream:
                temporary = stream.name
                json.dump(dict(request_id=request_id, sha256=digest, filename=destination.name), stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, str(pointer))
            temporary = None
            descriptor = os.open(str(self.directory), os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        finally:
            if temporary is not None:
                os.unlink(temporary)

    def restore_latest(self):
        """再起動時は保存fileとhashを再検証し、成立した場合だけ復元する。"""
        pointer = self.directory/'latest_mission.json'
        if not pointer.exists():
            return
        try:
            data = json.loads(pointer.read_text(encoding='utf-8'))
            request_id = str(uuid.UUID(data['request_id']))
            filename = data['filename']
            digest = data['sha256']
            if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
                raise ValueError('受領fileのhash形式が不正です')
            if filename != request_id+'_'+digest+'.yaml':
                raise ValueError('受領file名が不正です')
            destination = self.directory/filename
            document = load_document(destination)
            # 読込時にも送信用条件と内容hashを検証する。
            from .model import validate
            document = validate(document, for_submission=True)
            if hashlib.sha256(json.dumps(document, sort_keys=True).encode()).hexdigest() != digest:
                raise ValueError('受領fileのhashが一致しません')
            self.publish_saved(request_id, digest, destination, document)
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            self.get_logger().error('保存済みミッションを復元できません: '+str(error))

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
            self.save_latest(request_id, digest, destination)
            self.publish_saved(request_id, digest, destination, document)
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
