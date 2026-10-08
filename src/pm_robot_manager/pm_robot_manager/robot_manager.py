#!/usr/bin/env python3
"""systemd管理下のCore、走行許可、bag launchを独立して操作するnode。"""

import json
from pathlib import Path
import shutil
import subprocess
import threading
import time
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class RobotManager(Node):
    """Jetson上のlaunch unitを監視し、Qtを止めずに操作を受け付ける。"""

    STATES = ('STOPPED', 'STARTING', 'RUNNING', 'STOPPING', 'ERROR')
    PROFILES = ('mission', 'debug')

    def __init__(self):
        super().__init__('pm_robot_manager')
        self.declare_parameter('unit_name', 'start-pm.service')
        self.declare_parameter('mission_bag_unit_name', 'pm-mission-bag.service')
        self.declare_parameter('debug_bag_unit_name', 'pm-debug-bag.service')
        self.declare_parameter('vehicle_unit_name', 'pm-vehicle-control.service')
        self.declare_parameter('vehicle_start_service', '/pm/robot_manager/vehicle/start')
        self.declare_parameter('vehicle_stop_service', '/pm/robot_manager/vehicle/stop')
        self.declare_parameter('vehicle_status_topic', '/pm/vehicle/status')
        self.declare_parameter('use_sudo', True)
        self.declare_parameter('startup_timeout_sec', 60.0)
        self.declare_parameter('shutdown_timeout_sec', 45.0)
        self.declare_parameter('bag_stop_request_timeout_sec', 20.0)
        self.declare_parameter('bag_stop_warning_sec', 120.0)
        self.declare_parameter('status_topic', '/pm/robot_manager/status')
        self.declare_parameter('start_service', '/pm/robot_manager/start')
        self.declare_parameter('stop_service', '/pm/robot_manager/stop')
        self.declare_parameter('get_status_service', '/pm/robot_manager/get_status')
        self.declare_parameter(
            'mission_bag_start_service', '/pm/robot_manager/mission_bag/start')
        self.declare_parameter(
            'mission_bag_stop_service', '/pm/robot_manager/mission_bag/stop')
        self.declare_parameter(
            'debug_bag_start_service', '/pm/robot_manager/debug_bag/start')
        self.declare_parameter(
            'debug_bag_stop_service', '/pm/robot_manager/debug_bag/stop')
        self.declare_parameter(
            'mission_bag_recorder_stop_service',
            '/pm/robot_manager/mission_bag/request_stop')
        self.declare_parameter(
            'debug_bag_recorder_stop_service',
            '/pm/robot_manager/debug_bag/request_stop')
        self.declare_parameter(
            'mission_bag_status_topic', '/pm/robot_manager/mission_bag/status')
        self.declare_parameter(
            'debug_bag_status_topic', '/pm/robot_manager/debug_bag/status')

        self.units = {
            'core': str(self.get_parameter('unit_name').value),
            'vehicle': str(self.get_parameter('vehicle_unit_name').value),
            'mission': str(self.get_parameter('mission_bag_unit_name').value),
            'debug': str(self.get_parameter('debug_bag_unit_name').value),
        }
        self.use_sudo = bool(self.get_parameter('use_sudo').value)
        self.startup_timeout = float(self.get_parameter('startup_timeout_sec').value)
        self.shutdown_timeout = float(self.get_parameter('shutdown_timeout_sec').value)
        self.bag_stop_request_timeout = float(
            self.get_parameter('bag_stop_request_timeout_sec').value)
        self.bag_stop_warning = float(self.get_parameter('bag_stop_warning_sec').value)

        self._lock = threading.RLock()
        self._states = {key: 'UNKNOWN' for key in self.units}
        self._errors = {key: '' for key in self.units}
        self._recording_runtime = {
            profile: {
                'recorder_state': 'UNKNOWN',
                'output': '',
                'started_at_unix_sec': None,
                'verified': None,
                'invocation_id': '',
                'error': '',
                'stamp_unix_sec': None,
            }
            for profile in self.PROFILES
        }
        self._vehicle_operation = None
        self._vehicle_detail = {}
        self._vehicle_stamp = None
        self._vehicle_unit_state = 'unknown'
        self._operation = None
        self._operation_targets = set()
        self._monitor_running = False

        self._publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('vehicle_status_topic').value),
            self._vehicle_status_callback, 10)
        for operation in ('start', 'stop'):
            self.create_service(
                Trigger, str(self.get_parameter('vehicle_'+operation+'_service').value),
                lambda request, response, action=operation:
                    self._vehicle_request(action, request, response))
        self._bag_stop_clients = {}
        for profile in self.PROFILES:
            start_name = str(self.get_parameter(
                profile+'_bag_start_service').value)
            stop_name = str(self.get_parameter(
                profile+'_bag_stop_service').value)
            recorder_stop_name = str(self.get_parameter(
                profile+'_bag_recorder_stop_service').value)
            status_name = str(self.get_parameter(
                profile+'_bag_status_topic').value)
            self.create_service(
                Trigger, start_name,
                lambda request, response, bag=profile:
                    self._bag_start_request(bag, request, response))
            self.create_service(
                Trigger, stop_name,
                lambda request, response, bag=profile:
                    self._bag_stop_request(bag, request, response))
            self._bag_stop_clients[profile] = self.create_client(
                Trigger, recorder_stop_name)
            self.create_subscription(
                String, status_name,
                lambda message, bag=profile:
                    self._recording_status_callback(bag, message), 10)

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
            'Robot Manager ready; Core and bag launch processes remain independent')

    def _systemctl(self, *arguments, timeout=5.0):
        command = ['systemctl', *arguments]
        if self.use_sudo:
            command[0:0] = ['sudo', '-n']
        try:
            return subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=timeout)
        except OSError as error:
            # errnoだけでは保存先・process起動・systemd接続を区別できない。
            raise RuntimeError('systemctl process起動失敗: {}: {}'.format(
                ' '.join(command), error)) from error

    def _unit_state(self, component):
        result = self._systemctl('is-active', self.units[component])
        state = result.stdout.strip()
        if component == 'vehicle':
            with self._lock:
                self._vehicle_unit_state = state or 'unknown'
        return state or 'unknown'

    def _refresh_initial_state(self):
        for component in self.units:
            try:
                unit_state = self._unit_state(component)
                self._states[component] = self._state_from_unit(unit_state)
                if unit_state == 'failed':
                    self._errors[component] = 'systemd unitがfailed状態です'
                elif unit_state == 'unknown':
                    self._errors[component] = 'systemd unit状態を取得できません'
            except (OSError, subprocess.TimeoutExpired) as exc:
                self._states[component] = 'ERROR'
                self._errors[component] = 'systemd状態を取得できません: {}'.format(exc)

    @staticmethod
    def _state_from_unit(unit_state):
        return {
            'inactive': 'STOPPED',
            'active': 'RUNNING',
            'activating': 'STARTING',
            'deactivating': 'STOPPING',
            'failed': 'ERROR',
        }.get(unit_state, 'ERROR')

    def _state_json(self):
        with self._lock:
            now = time.time()
            recordings = {}
            for profile in self.PROFILES:
                runtime = dict(self._recording_runtime[profile])
                started = runtime.get('started_at_unix_sec')
                elapsed_end = now
                if self._states[profile] != 'RUNNING':
                    elapsed_end = float(runtime.get('stamp_unix_sec') or now)
                runtime.update({
                    'state': self._states[profile],
                    'unit': self.units[profile],
                    'manager_error': self._errors[profile],
                    'elapsed_sec': (
                        max(0.0, elapsed_end-float(started))
                        if started is not None
                        else 0.0),
                })
                output = runtime.get('output', '')
                try:
                    free_bytes = shutil.disk_usage(Path(output).parent).free if output else None
                except OSError:
                    free_bytes = None
                runtime['free_bytes'] = free_bytes
                recordings[profile] = runtime
            payload = {
                'state': self._states['core'],
                'error': self._errors['core'],
                'unit': self.units['core'],
                'vehicle': {
                    'state': self._states['vehicle'],
                    'error': self._errors['vehicle'],
                    'unit_state': self._vehicle_unit_state,
                    'detail': dict(self._vehicle_detail),
                    'fresh': (self._vehicle_stamp is not None
                              and time.monotonic()-self._vehicle_stamp <= 3.0),
                },
                'recordings': recordings,
                'operation': self._operation or '',
                'stamp_unix_sec': now,
            }
        return json.dumps(payload, ensure_ascii=False)

    def _publish_status(self):
        message = String()
        message.data = self._state_json()
        self._publisher.publish(message)

    def _recording_status_callback(self, profile, message):
        try:
            data = json.loads(message.data)
            if data.get('profile') != profile:
                return
            with self._lock:
                runtime = self._recording_runtime[profile]
                # 取消済みの起動から遅れて届くERROR/RECORDINGを採用しない。
                if (runtime.get('cancelled_before_output') is True
                        and runtime.get('invocation_id') == data.get('invocation_id')):
                    return
                runtime.update({
                    'recorder_state': str(data.get('state', 'UNKNOWN')).upper(),
                    'output': str(data.get('output', '')),
                    'started_at_unix_sec': data.get('started_at_unix_sec'),
                    'verified': data.get('verified'),
                    'cancelled_before_output': data.get('cancelled_before_output', False),
                    'invocation_id': str(data.get('invocation_id', '')),
                    'error': str(data.get('error', '')),
                    'stamp_unix_sec': data.get('stamp_unix_sec', time.time()),
                })
        except (TypeError, ValueError):
            self.get_logger().warning(
                '{} bag status JSONを解釈できません'.format(profile))

    def _check_unit_state(self):
        with self._lock:
            if self._monitor_running or self._operation is not None:
                return
            self._monitor_running = True
        threading.Thread(target=self._monitor_units, daemon=True).start()

    def _monitor_units(self):
        try:
            for component in self.units:
                with self._lock:
                    if component == 'vehicle' and self._vehicle_operation is not None:
                        continue
                unit_state = self._unit_state(component)
                if component in self.PROFILES:
                    if self._cancel_missing_stopped_bag(component, unit_state):
                        continue
                with self._lock:
                    previous = self._states[component]
                if component in self.PROFILES and unit_state == 'inactive':
                    if previous == 'RUNNING':
                        if self._verify_bag_completion(component):
                            self._set_component(component, 'STOPPED', '')
                        else:
                            self._set_component(
                                component, 'ERROR',
                                'bag終了後の保存検証に成功していません')
                        continue
                state = self._state_from_unit(unit_state)
                # 正常終了を確認できなかった走行STOPのエラーは、active監視で消さない。
                with self._lock:
                    error = self._errors[component] if component == 'vehicle' else ''
                if unit_state == 'failed':
                    error = 'systemd unitがfailed状態です'
                elif unit_state == 'unknown':
                    error = 'systemd unit状態を取得できません'
                self._set_component(component, state, error)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warning(
                'systemd unit監視に失敗しました: {}\n{}'.format(exc, traceback.format_exc()))
        finally:
            with self._lock:
                self._monitor_running = False

    def _set_component(self, component, state, error):
        with self._lock:
            self._states[component] = state
            self._errors[component] = error

    def _verify_bag_completion(self, profile):
        with self._lock:
            output = self._recording_runtime[profile].get('output', '')
        if not output:
            return False
        # 保存済みbagと起動直後の取消を区別し、古い起動の証明は採用しない。
        with self._lock:
            cancellation_id = self._recording_runtime[profile].get('invocation_id', '')
        if cancellation_id and not Path(output).exists():
            try:
                cancellation = json.loads(Path(output+'.cancelled.json').read_text())
            except (OSError, ValueError):
                cancellation = {}
            if (cancellation.get('invocation_id') == cancellation_id
                    and cancellation.get('cancelled_before_output') is True
                    and cancellation.get('verified') is True
                    and cancellation.get('recorder_returncode') in (0, 2, -2, 130)):
                return True
        completion_path = Path(output) / 'completion.json'
        try:
            data = json.loads(completion_path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return False
        with self._lock:
            invocation_id = self._recording_runtime[profile].get('invocation_id', '')
        return bool(
            data.get('verified') is True
            and data.get('metadata_exists') is True
            and data.get('bag_info_returncode') == 0
            and (not invocation_id
                 or data.get('invocation_id') == invocation_id))

    def _get_status(self, _request, response):
        response.success = True
        response.message = self._state_json()
        return response

    def _begin_operation(self, operation, transitions, response):
        with self._lock:
            if (self._operation is not None
                    or operation in ('start_core', 'stop_core')
                    and self._vehicle_operation is not None):
                response.success = False
                response.message = '別の操作が進行中です: {}'.format(
                    self._operation or 'vehicle '+str(self._vehicle_operation))
                return False
            self._operation = operation
            self._operation_targets = set(transitions)
            for component, state in transitions.items():
                self._states[component] = state
                self._errors[component] = ''
            response.success = True
            response.message = '{}要求を受け付けました'.format(operation)
        threading.Thread(
            target=self._run_operation, args=(operation,), daemon=True).start()
        return True

    def _start_request(self, _request, response):
        with self._lock:
            state = self._states['core']
        if state == 'RUNNING':
            response.success, response.message = True, 'Robot Coreは既にRUNNINGです'
            return response
        self._begin_operation('start_core', {'core': 'STARTING'}, response)
        return response

    def _stop_request(self, _request, response):
        with self._lock:
            core_state = self._states['core']
            bag_states = {p: self._states[p] for p in (*self.PROFILES, 'vehicle')}
        if core_state == 'STOPPED' and all(
                state == 'STOPPED' for state in bag_states.values()):
            response.success, response.message = True, 'Coreとbagは既にSTOPPEDです'
            return response
        transitions = {'core': 'STOPPING'}
        for profile, state in bag_states.items():
            if state in ('RUNNING', 'STARTING', 'STOPPING'):
                transitions[profile] = 'STOPPING'
        self._begin_operation('stop_core', transitions, response)
        return response

    def _vehicle_status_callback(self, message):
        try:
            detail = json.loads(message.data)
            if not isinstance(detail, dict):
                raise ValueError('statusがobjectではありません')
            with self._lock:
                self._vehicle_detail = detail
                self._vehicle_stamp = time.monotonic()
        except (ValueError, TypeError) as exc:
            self.get_logger().warning('車両statusを解釈できません: {}'.format(exc))

    def _vehicle_request(self, action, _request, response):
        """bagの保存待ちで走行STOPを遅らせないよう、独立workerで処理する。"""
        with self._lock:
            state, core = self._states['vehicle'], self._states['core']
            if self._vehicle_operation is not None:
                response.success, response.message = False, '走行許可は状態遷移中です'
                return response
            if action == 'start' and (core != 'RUNNING' or self._operation is not None):
                response.success, response.message = False, 'Core稼働中かつ他の操作の完了後に許可してください'
                return response
            if (action == 'start' and state == 'RUNNING'
                    or action == 'stop' and state == 'STOPPED'):
                response.success, response.message = True, '走行許可は要求された状態です'
                return response
            self._vehicle_operation = action
            self._states['vehicle'] = 'STARTING' if action == 'start' else 'STOPPING'
            self._errors['vehicle'] = ''
        response.success, response.message = True, 'vehicle '+action+'要求を受け付けました'
        threading.Thread(target=self._run_vehicle_operation, args=(action,), daemon=True).start()
        return response

    def _run_vehicle_operation(self, action):
        try:
            if action == 'start':
                self._start_vehicle()
            else:
                self._stop_vehicle()
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self._set_component('vehicle', 'ERROR', str(exc))
            self.get_logger().error('走行許可の操作に失敗: {}'.format(exc))
        finally:
            with self._lock:
                self._vehicle_operation = None

    def _start_vehicle(self):
        if self._unit_state('core') != 'active':
            raise RuntimeError('Coreがactiveではありません')
        if self._unit_state('vehicle') == 'active':
            self._set_component('vehicle', 'RUNNING', '')
            return
        # 古いCoreや手動launchとの重複起動を拒否する。既存processは勝手に停止しない。
        nodes = {name for name, namespace in self.get_node_names_and_namespaces()}
        if nodes.intersection({'vehicle_interface_node', 'joy_teleop'}):
            raise RuntimeError('管理外の走行nodeが存在します。Coreの旧設定や手動launchを確認してください')
        started = time.monotonic()
        with self._lock:
            self._vehicle_stamp = None
            self._vehicle_detail = {}
        result = self._systemctl('start', '--no-block', self.units['vehicle'], timeout=10.0)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or '走行用unitの開始に失敗しました')
        try:
            self._wait_for_unit('vehicle', 'active', self.startup_timeout)
            # systemd activeだけではUSB接続成功を保証しない。新しい接続statusを待つ。
            while time.monotonic()-started < self.startup_timeout:
                if self._unit_state('vehicle') != 'active':
                    raise RuntimeError('走行用launchが終了しました')
                with self._lock:
                    stamp, detail = self._vehicle_stamp, dict(self._vehicle_detail)
                if (stamp is not None and stamp >= started
                        and time.monotonic()-stamp <= 3.0 and detail.get('connected')):
                    self._set_component('vehicle', 'RUNNING', '')
                    return
                time.sleep(0.25)
            raise RuntimeError('ODrive接続statusの待受がtimeoutしました')
        except Exception:
            # 起動失敗後に遅れてモーター制御が始まるprocessを残さない。
            self._stop_vehicle()
            raise

    def _stop_vehicle(self):
        """速度変換とvehicle interfaceへSIGINTを送り、IDLE処理後の終了を待つ。"""
        state = self._unit_state('vehicle')
        if state == 'inactive':
            self._set_component('vehicle', 'STOPPED', '')
            return
        if state in ('failed', 'unknown'):
            # failedでも停止timeoutでprocessが残る場合があるため、OFF成功に見せない。
            raise RuntimeError('走行unitの正常停止を確認できません: '+state)
        result = self._systemctl('stop', self.units['vehicle'],
                                 timeout=self.shutdown_timeout+5.0)
        if result.returncode != 0 or self._unit_state('vehicle') != 'inactive':
            raise RuntimeError(result.stderr.strip() or '走行用launchの正常終了を確認できません')
        self._set_component('vehicle', 'STOPPED', '')

    def _bag_start_request(self, profile, _request, response):
        with self._lock:
            core_state = self._states['core']
            bag_state = self._states[profile]
        if core_state != 'RUNNING':
            response.success = False
            response.message = 'Robot Core稼働中にだけbagを開始できます'
            return response
        if bag_state == 'RUNNING':
            response.success, response.message = True, 'bagは既に記録中です'
            return response
        self._begin_operation(
            'start_'+profile+'_bag', {profile: 'STARTING'}, response)
        return response

    def _bag_stop_request(self, profile, _request, response):
        with self._lock:
            state = self._states[profile]
        if state == 'STOPPED':
            response.success, response.message = True, 'bagは既にSTOPPEDです'
            return response
        self._begin_operation(
            'stop_'+profile+'_bag', {profile: 'STOPPING'}, response)
        return response

    def _start_core(self):
        result = self._systemctl(
            'start', '--no-block', self.units['core'], timeout=10.0)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or 'Coreのsystemctl startに失敗しました')
        self._wait_for_unit('core', 'active', self.startup_timeout)
        self._set_component('core', 'RUNNING', '')

    def _start_bag(self, profile):
        if self._unit_state('core') != 'active':
            raise RuntimeError('Robot Core unitがactiveではありません')
        with self._lock:
            self._recording_runtime[profile] = {
                'recorder_state': 'STARTING',
                'output': '',
                'started_at_unix_sec': None,
                'verified': None,
                'invocation_id': '',
                'error': '',
                'stamp_unix_sec': time.time(),
            }
        result = self._systemctl(
            'start', '--no-block', self.units[profile], timeout=10.0)
        if result.returncode != 0:
            raise RuntimeError(
                result.stderr.strip() or '{} bagのsystemctl startに失敗しました'.format(profile))
        self._wait_for_unit(profile, 'active', self.startup_timeout)
        self._set_component(profile, 'RUNNING', '')

    def _wait_for_unit(self, component, expected, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            state = self._unit_state(component)
            if state == expected:
                return
            if state == 'failed':
                raise RuntimeError(
                    '{} unitがfailed状態になりました'.format(component))
            time.sleep(.25)
        raise RuntimeError(
            '{} unitが{}にならずtimeoutしました'.format(component, expected))

    def _request_recorder_stop(self, profile):
        client = self._bag_stop_clients[profile]
        try:
            ready = client.wait_for_service(timeout_sec=self.bag_stop_request_timeout)
        except Exception as exc:
            raise RuntimeError(
                '{} bag stop serviceの検索に失敗しました: {}'.format(profile, exc))
        if not ready:
            raise RuntimeError(
                '{} bag recorderの正常停止interfaceに接続できません。'
                'データ保護のため強制停止しません'.format(profile))
        try:
            future = client.call_async(Trigger.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(timeout=self.bag_stop_request_timeout):
                raise RuntimeError(
                    '{} bag recorderから停止受付応答がありません。強制停止しません'.format(profile))
            response = future.result()
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(
                '{} bag stop serviceとの通信に失敗しました: {}'.format(profile, exc))
        if response is None or not response.success:
            detail = response.message if response is not None else '応答なし'
            raise RuntimeError(
                '{} bag recorderが停止要求を拒否しました: {}'.format(profile, detail))

    def _cancel_missing_stopped_bag(self, profile, state):
        """終了済みで保存先のないbagを取消し、Core停止を妨げない。

        取消済みの起動、または現在の起動IDと保存先不在を確認できる場合
        だけを対象とする。存在する未検証bagや確認不能な起動は解除しない。
        """
        if state not in ('inactive', 'failed'):
            return False
        with self._lock:
            runtime = dict(self._recording_runtime[profile])
        already_cancelled = runtime.get('cancelled_before_output') is True
        output = runtime.get('output', '')
        if not already_cancelled:
            if not output or not Path(output).is_absolute() or not runtime.get('invocation_id'):
                return False
            try:
                Path(output).lstat()
                return False
            except FileNotFoundError:
                pass
        result = subprocess.run(
            ['systemctl', 'show', self.units[profile], '--property=InvocationID',
             '--property=MainPID', '--property=ControlPID', '--property=ActiveState'],
            check=False, capture_output=True, text=True, timeout=5.0)
        if result.returncode != 0:
            raise RuntimeError('終了済みbagの状態取得に失敗しました')
        values = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
        if (values.get('ActiveState') not in ('inactive', 'failed')
                or values.get('MainPID') != '0' or values.get('ControlPID') != '0'):
            return False
        if not already_cancelled and values.get('InvocationID') != runtime['invocation_id']:
            return False
        if already_cancelled and values.get('InvocationID') not in ('', runtime.get('invocation_id')):
            return False
        if values['ActiveState'] == 'failed':
            result = self._systemctl('reset-failed', self.units[profile])
            if result.returncode != 0:
                raise RuntimeError(result.stderr.strip() or 'bag取消のfailed解除に失敗しました')
        if self._unit_state(profile) != 'inactive':
            raise RuntimeError('bag取消後の停止状態を確認できませんでした')
        with self._lock:
            self._recording_runtime[profile].update({
                'recorder_state': 'CANCELLED', 'output': '', 'verified': False,
                'cancelled_before_output': True, 'error': '',
                'started_at_unix_sec': None, 'stamp_unix_sec': time.time(),
            })
        self._set_component(profile, 'STOPPED', '')
        return True

    def _force_stop_missing_bag(self, profile):
        """保存先不在の起動だけを取消す。保存成功とは区別する。

        判定中のdirectory作成をSIGSTOPで防止し、systemdの起動IDと
        recorder statusを照合する。確認不能・保存先ありの場合は再開する。
        """
        with self._lock:
            runtime = dict(self._recording_runtime[profile])
        output = runtime.get('output', '')
        invocation_id = runtime.get('invocation_id', '')
        if not output or not invocation_id or not Path(output).is_absolute():
            return False
        try:
            Path(output).lstat()
            return False
        except FileNotFoundError:
            pass
        unit = self.units[profile]
        killed = False
        try:
            # 一部processだけ停止してからsystemctlが失敗する場合もfinallyで再開する。
            result = self._systemctl('kill', '--signal=SIGSTOP', '--kill-who=all', unit)
            if result.returncode != 0:
                raise RuntimeError(result.stderr.strip() or 'bagの一時停止に失敗しました')
            # 読み取り専用showにはsudoを使わず、権限をkill対象unitだけに限定する。
            result = subprocess.run(
                ['systemctl', 'show', unit, '--property=InvocationID', '--value'],
                check=False, capture_output=True, text=True, timeout=5.0)
            if result.returncode != 0 or result.stdout.strip() != invocation_id:
                raise RuntimeError('bagの起動IDを確認できないため強制停止しません')
            try:
                Path(output).lstat()
                return False
            except FileNotFoundError:
                pass
            result = self._systemctl('kill', '--signal=SIGKILL', '--kill-who=all', unit)
            if result.returncode != 0:
                raise RuntimeError(result.stderr.strip() or 'bagの強制停止に失敗しました')
            killed = True
        finally:
            if not killed:
                result = self._systemctl('kill', '--signal=SIGCONT', '--kill-who=all', unit)
                if result.returncode != 0:
                    raise RuntimeError(result.stderr.strip() or 'bagを再開できませんでした')
        result = self._systemctl('stop', unit, timeout=30.0)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or '強制停止後のunit停止に失敗しました')
        # SIGKILL完了後の実状態だけを正常化する。既にinactiveなら
        # reset-failedは不要で、未ロードunitへの失敗も回避できる。
        final_state = self._unit_state(profile)
        if final_state == 'failed':
            result = self._systemctl('reset-failed', unit)
            if result.returncode != 0:
                raise RuntimeError(result.stderr.strip() or 'bagのfailed状態を解除できませんでした')
            final_state = self._unit_state(profile)
        if final_state != 'inactive':
            raise RuntimeError('強制停止後のunit状態を正常化できませんでした: '+final_state)
        with self._lock:
            self._recording_runtime[profile].update({
                'recorder_state': 'CANCELLED', 'output': '', 'verified': False,
                'cancelled_before_output': True, 'stamp_unix_sec': time.time(),
                'error': '', 'started_at_unix_sec': None,
            })
        self.get_logger().warning(
            '{} bagを保存先不在のため強制停止しました（保存済みbagなし）: {}'
            .format(profile, output))
        self._set_component(profile, 'STOPPED', '')
        return True

    def _stop_bag_safely(self, profile):
        state = self._unit_state(profile)
        if self._cancel_missing_stopped_bag(profile, state):
            return
        if state in ('inactive', 'unknown'):
            if state == 'unknown':
                raise RuntimeError('{} bag unit状態を取得できません'.format(profile))
            with self._lock:
                runtime = dict(self._recording_runtime[profile])
                previous_state = self._states[profile]
            was_recording = previous_state in ('STARTING', 'RUNNING', 'STOPPING')
            if was_recording and (
                    not runtime.get('output')
                    or not self._verify_bag_completion(profile)):
                raise RuntimeError(
                    '{} bag unitはinactiveですが保存完了を検証できません'.format(profile))
            if runtime.get('output') and self._verify_bag_completion(profile):
                with self._lock:
                    runtime_error = self._recording_runtime[profile].get('error', '')
                if runtime_error:
                    raise RuntimeError('{} bag保存検証失敗: {}'.format(
                        profile, runtime_error))
                self._set_component(profile, 'STOPPED', '')
            elif previous_state in ('STOPPED', 'UNKNOWN'):
                self._set_component(profile, 'STOPPED', '')
            return
        if state == 'failed':
            # failed unitには生きたrecorder processがない。未検証bagがあれば
            # Core停止も中断し、保存済みと誤認したまま次へ進まない。
            with self._lock:
                has_output = bool(self._recording_runtime[profile].get('output'))
            if has_output and not self._verify_bag_completion(profile):
                raise RuntimeError(
                    '{} bag unitがfailedで、保存完了を確認できません。'
                    'Core停止を中断します'.format(profile))
            self._set_component(profile, 'ERROR', 'bag unitがfailed状態です')
            return
        with self._lock:
            self._operation_targets.add(profile)
        self._set_component(profile, 'STOPPING', '')
        if self._force_stop_missing_bag(profile):
            return
        try:
            self._request_recorder_stop(profile)
        except RuntimeError:
            if self._force_stop_missing_bag(profile):
                return
            raise
        stop_started = time.monotonic()
        warned = False
        completion_confirmed = False
        while True:
            unit_state = self._unit_state(profile)
            if self._cancel_missing_stopped_bag(profile, unit_state):
                return
            if unit_state == 'failed':
                raise RuntimeError(
                    '{} bag unitが異常終了しました。Core停止を中断します'.format(profile))
            if unit_state in ('active', 'activating', 'deactivating'):
                if self._force_stop_missing_bag(profile):
                    return
            completion_confirmed = self._verify_bag_completion(profile)
            if unit_state == 'inactive':
                if not completion_confirmed:
                    raise RuntimeError(
                        '{} bagのmetadata.yamlとros2 bag infoを確認できません。'
                        'Core停止は行いません'.format(profile))
                with self._lock:
                    runtime_error = self._recording_runtime[profile].get('error', '')
                if runtime_error:
                    raise RuntimeError('{} bag保存検証失敗: {}'.format(
                        profile, runtime_error))
                self._set_component(profile, 'STOPPED', '')
                return
            if completion_confirmed:
                with self._lock:
                    runtime_error = self._recording_runtime[profile].get('error', '')
                if runtime_error:
                    raise RuntimeError('{} bag保存検証失敗: {}'.format(
                        profile, runtime_error))
                if unit_state == 'deactivating':
                    # systemctl stopが先行している場合は、その安全待機処理の完了を待つ。
                    while True:
                        final_state = self._unit_state(profile)
                        if final_state == 'inactive':
                            break
                        if final_state in ('failed', 'unknown'):
                            raise RuntimeError(
                                '{} bag unitが{}状態で停止しました'.format(
                                    profile, final_state))
                        time.sleep(.5)
                else:
                    # recorder自身が正常終了した後にlaunch残プロセスを止める。
                    # ここへ到達する時点でmetadata.yamlとros2 bag infoは確認済み。
                    result = self._systemctl(
                        'stop', self.units[profile], timeout=30.0)
                    if result.returncode != 0:
                        raise RuntimeError(
                            result.stderr.strip() or
                            '{} bag unit停止に失敗しました'.format(profile))
                if self._unit_state(profile) != 'inactive':
                    raise RuntimeError(
                        '{} bagは保存確認済みですがunitが停止していません'.format(profile))
                self._set_component(profile, 'STOPPED', '')
                return
            elapsed = time.monotonic() - stop_started
            if elapsed >= self.bag_stop_warning and not warned:
                self._set_component(
                    profile, 'STOPPING',
                    'bag書き込み完了を待っています。安全のため強制終了しません')
                warned = True
            time.sleep(.5)

    def _stop_core_unit(self):
        try:
            result = self._systemctl(
                'stop', self.units['core'],
                timeout=self.shutdown_timeout + 5.0)
        except subprocess.TimeoutExpired:
            result = None
        if result is None or result.returncode != 0:
            # bagは検証済みで停止済み。ここではCore unitだけにTERM fallbackを行う。
            self._systemctl(
                'kill', '--signal=SIGTERM', '--kill-who=all',
                self.units['core'], timeout=5.0)
            raise RuntimeError('Core正常停止timeout後にSIGTERM fallbackを依頼しました')
        core_state = self._unit_state('core')
        if core_state == 'failed':
            raise RuntimeError('Robot Coreの停止処理が失敗しました。systemd journalを確認してください')
        if core_state != 'inactive':
            raise RuntimeError('停止要求後もRobot Core unitが動作中です')
        self._set_component('core', 'STOPPED', '')

    def _run_operation(self, operation):
        try:
            if operation == 'start_core':
                self._start_core()
            elif operation == 'stop_core':
                # bag保存で時間がかかっても先に走行を不許可にする。
                self._stop_vehicle()
                # recorderがcompletion.jsonまで書き終えたことを確認してからCoreを止める。
                for profile in ('debug', 'mission'):
                    self._stop_bag_safely(profile)
                self._stop_core_unit()
            elif operation.startswith('start_') and operation.endswith('_bag'):
                profile = operation[len('start_'):-len('_bag')]
                self._start_bag(profile)
            elif operation.startswith('stop_') and operation.endswith('_bag'):
                profile = operation[len('stop_'):-len('_bag')]
                self._stop_bag_safely(profile)
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(
                '{} failed: {}\n{}'.format(operation, exc, traceback.format_exc()))
            with self._lock:
                # Core停止中に実測で見つかったbag unitも対象へ追加されるため、
                # 失敗時点の対象一覧を読み直して状態を反映する。
                targets = set(self._operation_targets)
                for component in targets:
                    unit_state = self._safe_unit_state(component)
                    if component in ('core', 'vehicle'):
                        self._states[component] = self._state_from_unit(unit_state)
                    elif component in self.PROFILES and unit_state == 'inactive':
                        if self._verify_bag_completion(component):
                            self._states[component] = 'STOPPED'
                            self._errors[component] = ''
                            continue
                        self._states[component] = 'ERROR'
                    elif component in self.PROFILES and unit_state in (
                            'active', 'activating', 'deactivating'):
                        self._states[component] = (
                            'STOPPING' if operation.startswith('stop_') else
                            self._state_from_unit(unit_state))
                    else:
                        self._states[component] = 'ERROR'
                    self._errors[component] = str(exc)
        finally:
            with self._lock:
                self._operation = None
                self._operation_targets = set()

    def _safe_unit_state(self, component):
        try:
            return self._unit_state(component)
        except (OSError, subprocess.TimeoutExpired):
            return 'unknown'


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
