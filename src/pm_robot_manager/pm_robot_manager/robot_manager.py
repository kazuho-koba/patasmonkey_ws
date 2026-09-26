#!/usr/bin/env python3
"""systemd unitを介してRobot Coreを開始・停止する管理node。"""

import json
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class RobotManager(Node):
    """Robot Core状態をpublishし、非同期にsystemd操作を受け付ける。"""

    STATES = ('STOPPED', 'STARTING', 'RUNNING', 'STOPPING', 'ERROR')

    def __init__(self):
        super().__init__('pm_robot_manager')
        self.declare_parameter('unit_name', 'start-pm.service')
        self.declare_parameter('use_sudo', True)
        self.declare_parameter('startup_timeout_sec', 45.0)
        self.declare_parameter('shutdown_timeout_sec', 35.0)
        self.declare_parameter('status_topic', '/pm/robot_manager/status')
        self.declare_parameter('start_service', '/pm/robot_manager/start')
        self.declare_parameter('stop_service', '/pm/robot_manager/stop')
        self.declare_parameter('get_status_service', '/pm/robot_manager/get_status')

        self.unit_name = str(self.get_parameter('unit_name').value)
        self.use_sudo = bool(self.get_parameter('use_sudo').value)
        self.startup_timeout = float(
            self.get_parameter('startup_timeout_sec').value)
        self.shutdown_timeout = float(
            self.get_parameter('shutdown_timeout_sec').value)
        self._lock = threading.Lock()
        self._state = 'STOPPED'
        self._error = ''
        self._operation_thread = None
        self._monitor_running = False
        self._publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_service(
            Trigger, str(self.get_parameter('start_service').value),
            self._start_request)
        self.create_service(
            Trigger, str(self.get_parameter('stop_service').value),
            self._stop_request)
        self.create_service(
            Trigger, str(self.get_parameter('get_status_service').value),
            self._get_status)
        self.create_timer(1.0, self._publish_status)
        self.create_timer(1.0, self._check_unit_state)
        self._refresh_initial_state()
        self.get_logger().info(
            'Robot Manager ready; it does not start Robot Core automatically')

    def _systemctl(self, *arguments, timeout=5.0):
        command = ['systemctl', *arguments]
        if self.use_sudo:
            command.insert(0, 'sudo')
            command.insert(1, '-n')
        return subprocess.run(
            command, check=False, capture_output=True, text=True,
            timeout=timeout)

    def _is_active(self):
        result = self._systemctl('is-active', self.unit_name)
        return result.returncode == 0 and result.stdout.strip() == 'active'

    def _unit_state(self):
        result = self._systemctl('is-active', self.unit_name)
        return result.stdout.strip()

    def _refresh_initial_state(self):
        try:
            result = self._systemctl('is-active', self.unit_name)
            unit_state = result.stdout.strip()
            if unit_state == 'active':
                self._state = 'RUNNING'
            elif unit_state == 'inactive':
                self._state = 'STOPPED'
            elif unit_state in ('activating', 'deactivating'):
                self._state = 'STARTING' if unit_state == 'activating' else 'STOPPING'
            elif unit_state == 'failed':
                self._state = 'ERROR'
                self._error = '既存のRobot Core unitがfailed状態です'
            else:
                self._state = 'ERROR'
                self._error = result.stderr.strip() or 'systemd状態を判定できません'
        except (OSError, subprocess.TimeoutExpired) as exc:
            self._state = 'ERROR'
            self._error = 'systemd状態を取得できません: {}'.format(exc)

    def _state_json(self):
        with self._lock:
            return json.dumps({
                'state': self._state,
                'error': self._error,
                'unit': self.unit_name,
                'stamp_unix_sec': time.time(),
            }, ensure_ascii=False)

    def _publish_status(self):
        message = String()
        message.data = self._state_json()
        self._publisher.publish(message)

    def _check_unit_state(self):
        with self._lock:
            if self._monitor_running:
                return
            self._monitor_running = True
        threading.Thread(target=self._monitor_unit, daemon=True).start()

    def _monitor_unit(self):
        try:
            unit_state = self._unit_state()
            with self._lock:
                current_state = self._state
                if unit_state == 'failed':
                    self._state = 'ERROR'
                    self._error = 'Robot Core systemd unitがfailed状態になりました'
                elif unit_state == 'active' and current_state not in (
                        'STARTING', 'STOPPING'):
                    self._state = 'RUNNING'
                    self._error = ''
                elif unit_state == 'active' and current_state == 'STARTING':
                    self._state = 'RUNNING'
                    self._error = ''
                elif unit_state == 'inactive' and current_state == 'STOPPING':
                    self._state = 'STOPPED'
                    self._error = ''
                elif unit_state == 'inactive' and current_state == 'RUNNING':
                    self._state = 'ERROR'
                    self._error = 'Robot Core unitが管理外で停止しました'
                elif unit_state == 'activating' and current_state in (
                        'STOPPED', 'ERROR'):
                    self._state = 'STARTING'
                    self._error = ''
                elif unit_state == 'deactivating' and current_state in (
                        'RUNNING', 'STARTING'):
                    self._state = 'STOPPING'
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warning('systemd unit監視に失敗しました: {}'.format(exc))
        finally:
            with self._lock:
                self._monitor_running = False

    def _get_status(self, _request, response):
        response.success = True
        response.message = self._state_json()
        return response

    def _accept_transition(self, target, response):
        with self._lock:
            if self._state in ('STARTING', 'STOPPING'):
                response.success = False
                response.message = '遷移中です: {}'.format(self._state)
                return False
            if target == 'STARTING' and self._state == 'RUNNING':
                response.success = True
                response.message = 'Robot Coreは既にRUNNINGです'
                return False
            if target == 'STOPPING' and self._state == 'STOPPED':
                response.success = True
                response.message = 'Robot Coreは既にSTOPPEDです'
                return False
            self._state = target
            self._error = ''
            response.success = True
            response.message = '{}要求を受け付けました'.format(target)
            return True

    def _start_request(self, _request, response):
        if self._accept_transition('STARTING', response):
            self._start_worker('start')
        return response

    def _stop_request(self, _request, response):
        if self._accept_transition('STOPPING', response):
            self._start_worker('stop')
        return response

    def _start_worker(self, operation):
        worker = threading.Thread(
            target=self._run_operation, args=(operation,), daemon=True)
        self._operation_thread = worker
        worker.start()

    def _run_operation(self, operation):
        try:
            if operation == 'start':
                command_timeout = 10.0
                result = self._systemctl(
                    'start', '--no-block', self.unit_name,
                    timeout=command_timeout)
                if result.returncode != 0:
                    raise RuntimeError(result.stderr.strip() or 'systemctl start失敗')
                deadline = time.monotonic() + self.startup_timeout
                while time.monotonic() < deadline:
                    unit_state = self._unit_state()
                    if unit_state == 'active':
                        self._set_state('RUNNING', '')
                        return
                    if unit_state == 'failed':
                        raise RuntimeError('systemd unitがfailed状態になりました')
                    time.sleep(0.25)
                raise RuntimeError('Robot Coreの起動がtimeoutしました')

            try:
                result = self._systemctl(
                    'stop', self.unit_name,
                    timeout=self.shutdown_timeout + 5.0)
            except subprocess.TimeoutExpired:
                result = None
            if result is None or result.returncode != 0:
                # unit側のSIGINT猶予処理が終わらない場合だけsystemdへSIGTERMを依頼する。
                self._systemctl(
                    'kill', '--signal=SIGTERM', '--kill-who=all',
                    self.unit_name, timeout=5.0)
                raise RuntimeError('正常停止timeout後にsystemdへSIGTERM fallbackを依頼しました')
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and self._is_active():
                time.sleep(0.2)
            if self._is_active():
                raise RuntimeError('停止要求後もRobot Core unitがactiveです')
            self._set_state('STOPPED', '')
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self._set_state('ERROR', str(exc))
            self.get_logger().error('Robot Core {} failed: {}'.format(operation, exc))

    def _set_state(self, state, error):
        with self._lock:
            self._state = state
            self._error = error


def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
