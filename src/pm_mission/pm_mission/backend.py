"""ROS callbackをQtのqueued signalへ渡し、画面のthreadで受信状態を扱う。"""
import math
import threading

from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from pm_msgs.srv import UploadMission
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
from .model import to_message


class Events(QObject):
    """executor threadからviewへ渡すimmutableな通知。"""
    position = pyqtSignal(float, float)
    receipt = pyqtSignal(str, bool, str)


class MissionBackend(Node):
    """非同期serviceのみを使い、Qt event loopでROS応答を待たない。"""
    def __init__(self, config, events):
        super().__init__('mission_planner_backend')
        self.events = events
        self.client = self.create_client(UploadMission, config['upload_service'])
        self.create_subscription(NavSatFix, config['gnss_topic'], self.fix,
                                 qos_profile_sensor_data)
        self.thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.thread.start()

    def fix(self, message):
        if (message.status.status >= 0 and math.isfinite(message.latitude)
                and math.isfinite(message.longitude) and abs(message.latitude) <= 90
                and abs(message.longitude) <= 180):
            self.events.position.emit(message.latitude, message.longitude)

    def submit(self, document, request_id):
        """同期送信はせずfuture完了を通知する。timeout管理はview側timerが行う。"""
        if not self.client.service_is_ready():
            raise RuntimeError('UGVのmission_receiverが見つかりません')
        request = UploadMission.Request()
        request.request_id = request_id
        request.mission = to_message(document)
        future = self.client.call_async(request)

        def finished(future):
            try:
                result = future.result()
                if result.request_id != request_id or result.mission_id != document['mission_id']:
                    raise RuntimeError('受領応答のIDが一致しません')
                detail = result.message
                if result.stored_path:
                    detail += '\n'+result.stored_path
                self.events.receipt.emit(request_id, result.accepted, detail)
            except Exception as error:
                self.events.receipt.emit(request_id, False, str(error))
        future.add_done_callback(finished)

    def close(self):
        """callbackが終了してからnodeを破棄し、GUI終了はRobot Coreへ影響させない。"""
        if rclpy.ok():
            rclpy.shutdown()
        self.thread.join(timeout=3)
        self.destroy_node()
