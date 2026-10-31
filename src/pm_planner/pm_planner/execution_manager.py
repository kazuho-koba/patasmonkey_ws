"""保存済みミッションと開始・終了要求の受口。MPPI接続前は自律有効化しない。"""
import hashlib
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from pm_mission.model import validate


class ExecutionManager(Node):
    """GUIと実車の間で受領状態を管理し、未完成controllerへの開始を拒否する。"""

    def __init__(self):
        super().__init__('execution_manager', namespace='/pm/planner')
        self.mission = None
        self.mission_document = None
        self.reason = 'ミッション未受領'
        self.vehicle = None
        self.vehicle_received = None
        self.stop_future = None
        self.preview = None
        self.preview_received = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/pm/mission/accepted', self.on_mission, qos)
        self.create_subscription(String, '/pm/vehicle/status', self.on_vehicle, 10)
        self.create_subscription(String, '/pm/planner/preview/status', self.on_preview, 10)
        # 検証済みsnapshotを保持し、previewが後から起動しても同じ経路を受け取れる。
        self.mission_publisher = self.create_publisher(String, '~/mission', qos)
        self.publisher = self.create_publisher(String, '~/status', 10)
        self.vehicle_client = self.create_client(SetBool, '/pm/vehicle/set_autonomy_enabled')
        self.create_service(Trigger, '~/start', self.start)
        self.create_service(Trigger, '~/stop', self.stop)
        self.create_timer(0.5, self.publish_status)

    def on_mission(self, message):
        """受信箱が保存に成功したdocumentだけを検証し、運用対象として保持する。"""
        try:
            payload = json.loads(message.data)
            mission = validate(payload['mission'], for_submission=True)
            digest = hashlib.sha256(json.dumps(mission, sort_keys=True).encode()).hexdigest()
            if digest != payload['sha256']:
                raise ValueError('mission hashが一致しません')
            self.mission = dict(mission_id=mission['mission_id'], revision=mission['revision'],
                                name=mission['name'], sha256=digest,
                                waypoint_count=len(mission['waypoints']),
                                stored_path=payload['stored_path'])
            self.mission_document = mission
            self.preview = None
            self.preview_received = None
            self.mission_publisher.publish(String(data=json.dumps(
                {'mission': mission, 'sha256': digest}, ensure_ascii=False)))
            self.reason = '経路受領済み。参照Path生成を待機中'
        except (ValueError, TypeError, KeyError) as exc:
            self.mission = None
            self.mission_document = None
            self.preview = None
            self.preview_received = None
            self.mission_publisher.publish(String(data=''))
            self.reason = '経路の検証失敗: '+str(exc)

    def on_preview(self, message):
        """受領snapshotと一致するpreview状態だけを現在のPathとして採用する。"""
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                raise ValueError('preview状態が不正です')
            provenance = status.get('coordinate_provenance') or {}
            if not self.mission or status.get('mission_id') != self.mission['mission_id'] or \
                    status.get('revision') != self.mission['revision'] or \
                    provenance.get('mission_sha256') != self.mission['sha256']:
                return
            self.preview = status
            self.preview_received = time.monotonic()
            if self.reason.startswith(('経路受領済み', '参照Path')):
                state = status.get('load_state', 'UNKNOWN')
                if state == 'READY' and status.get('geometry_valid'):
                    self.reason = '参照Path生成済み。MPPI未接続のため走行開始できません'
                elif state == 'ERROR':
                    self.reason = '参照Path生成失敗: '+status.get('load_error', '')
                else:
                    self.reason = '参照Path生成待機: '+state
        except (ValueError, TypeError, AttributeError):
            self.preview = None
            self.preview_received = None

    def on_vehicle(self, message):
        """実際の車両側許可状態と通信鮮度を表示する。"""
        try:
            data = json.loads(message.data)
            if not isinstance(data.get('autonomy_enabled'), bool):
                raise ValueError('自律許可状態がありません')
            self.vehicle = data
            self.vehicle_received = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.vehicle = None
            self.vehicle_received = None

    def start(self, request, response):
        """controller・経路検証が未完成のため、常に失敗応答し車両を許可しない。"""
        response.success = False
        response.message = ('ミッション未受領' if self.mission is None else
                            'MPPI controller未接続のため開始できません')
        self.reason = response.message
        return response

    def stop(self, request, response):
        """車両側へ非同期で無効化を要求。受付と実際の許可解除は分けて表示する。"""
        if not self.vehicle_client.service_is_ready():
            response.success = False
            response.message = '車両側の自律無効化サービスが未接続です'
            self.reason = response.message
            return response
        if self.stop_future and not self.stop_future.done():
            response.success = False
            response.message = '無効化応答待ちです'
            return response
        command = SetBool.Request()
        command.data = False
        self.stop_future = self.vehicle_client.call_async(command)
        self.stop_future.add_done_callback(self.stop_completed)
        self.reason = '車両側の自律無効化応答待ち'
        response.success = True
        response.message = '無効化要求を送信しました。車両状態で結果を確認してください'
        return response

    def stop_completed(self, future):
        try:
            result = future.result()
            self.reason = ('自律無効化応答: ' if result.success else '自律無効化失敗: ')+result.message
        except Exception as exc:
            self.reason = '自律無効化応答エラー: '+str(exc)

    def publish_status(self):
        fresh = self.vehicle_received is not None and time.monotonic()-self.vehicle_received < 2.0
        preview_fresh = self.preview_received is not None and time.monotonic()-self.preview_received < 2.0
        preview = self.preview if preview_fresh else None
        status = dict(state='BLOCKED', reason=self.reason, mission=self.mission,
                      controller_ready=False, drive_ready=False, vehicle_fresh=fresh,
                      path_state=(preview.get('load_state', 'UNKNOWN') if preview else
                                  'WAITING_FOR_PREVIEW' if self.mission else 'NO_MISSION'),
                      path_error=preview.get('load_error', '') if preview else '',
                      path_frame=preview.get('frame_id') if preview else None,
                      path_geometry_valid=bool(preview and preview.get('geometry_valid')),
                      path_preview_fresh=preview_fresh,
                      vehicle_autonomy_enabled=(self.vehicle['autonomy_enabled'] if fresh else None))
        self.publisher.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = ExecutionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
