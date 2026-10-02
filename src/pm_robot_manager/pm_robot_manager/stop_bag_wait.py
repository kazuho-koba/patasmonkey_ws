#!/usr/bin/env python3
"""systemd ExecStopからbag recorderの保存完了を待つ。"""

import json
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class BagStopWaiter(Node):
    """停止serviceを呼び、出力bagのcompletion.jsonを検証する。"""

    PROFILES = ('mission', 'debug')

    def __init__(self, profile, status_file, invocation_id):
        if profile not in self.PROFILES:
            raise ValueError('未知のbag profile: {}'.format(profile))
        super().__init__('pm_{}_bag_stop_waiter'.format(profile))
        self.profile = profile
        self.status_file = Path(status_file)
        self.invocation_id = invocation_id
        prefix = '/pm/robot_manager/{}_bag'.format(profile)
        self.output = ''
        self.recorder_state = 'UNKNOWN'
        self.verified = None
        self.error = ''
        self._client = self.create_client(Trigger, prefix+'/request_stop')
        self.create_subscription(
            String, prefix+'/status', self._status_callback, 10)

    def _status_callback(self, message):
        try:
            data = json.loads(message.data)
            if data.get('profile') != self.profile:
                return
            if (self.invocation_id
                    and data.get('invocation_id') != self.invocation_id):
                return
            self.output = str(data.get('output', self.output))
            self.recorder_state = str(data.get('state', 'UNKNOWN')).upper()
            self.verified = data.get('verified')
            self.error = str(data.get('error', ''))
        except (TypeError, ValueError):
            self.get_logger().warning('recorder status JSONを解釈できません')

    def _refresh_status_file(self):
        """unitごとの起動IDが一致する状態だけを採用し、古いbagを誤認しない。"""
        try:
            data = json.loads(self.status_file.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return
        if data.get('profile') != self.profile:
            return
        if (self.invocation_id
                and data.get('invocation_id') != self.invocation_id):
            return
        self.output = str(data.get('output', self.output))
        self.recorder_state = str(data.get('state', self.recorder_state)).upper()
        self.verified = data.get('verified', self.verified)
        self.error = str(data.get('error', self.error))

    def _completion_verified(self):
        self._refresh_status_file()
        if not self.output:
            return False
        try:
            data = json.loads(
                (Path(self.output)/'completion.json').read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return False
        return bool(
            data.get('verified') is True
            and data.get('metadata_exists') is True
            and data.get('bag_info_returncode') == 0
            and (not self.invocation_id
                 or data.get('invocation_id') == self.invocation_id))

    def wait_until_saved(self):
        """停止受付後はタイムアウトで打ち切らず、保存確認まで待ち続ける。"""
        last_notice = time.monotonic()
        last_request = 0.0
        stop_sent = False
        while True:
            rclpy.spin_once(self, timeout_sec=.2)
            if self._completion_verified():
                return 0
            now = time.monotonic()
            if (not stop_sent and now-last_request >= 2.0
                    and self._client.service_is_ready()):
                last_request = now
                future = self._client.call_async(Trigger.Request())
                while not future.done():
                    rclpy.spin_once(self, timeout_sec=.2)
                    if self._completion_verified():
                        return 0
                try:
                    response = future.result()
                except Exception as error:
                    self.get_logger().warning(
                        '停止serviceの応答待ちに失敗しました: {}'.format(error))
                    continue
                if response is not None and response.success:
                    stop_sent = True
                    self.get_logger().info(
                        '停止を受け付けました。bagの保存検証を待っています')
                else:
                    detail = response.message if response is not None else '応答なし'
                    self.get_logger().warning(
                        '停止要求を受け付けませんでした。保存確認を続けて再要求します: '
                        '{}'.format(detail))
            if now - last_notice >= 30.0:
                if self.recorder_state == 'ERROR' and self.verified is False:
                    self.get_logger().error(
                        'bagの保存検証に失敗しました。unitを終了させず待機します: {}'
                        .format(self.error))
                else:
                    self.get_logger().warning(
                        'bagの保存完了を待っています。強制終了は行いません')
                last_notice = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        helper = Node('pm_bag_stop_waiter_arguments_{}'.format(os.getpid()))
        helper.declare_parameter('profile', '')
        helper.declare_parameter('status_file', '')
        profile = str(helper.get_parameter('profile').value)
        status_file = str(helper.get_parameter('status_file').value)
        helper.destroy_node()
        node = BagStopWaiter(
            profile, status_file, os.environ.get('INVOCATION_ID', ''))
        return node.wait_until_saved()
    except (OSError, RuntimeError, ValueError) as error:
        print('bag停止確認に失敗しました: {}'.format(error), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
