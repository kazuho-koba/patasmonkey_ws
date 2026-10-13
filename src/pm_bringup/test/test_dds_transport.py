"""センサを起動せずDDS既定・環境隔離・診断のopt-inを検査する。"""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from pm_bringup.dds_transport import transport_environment, launch_transport_actions


def test_xml_capacity_and_transport():
    path = Path(__file__).parents[1]/'config/dds_transport_2m.xml'
    root = ET.parse(path).getroot()
    ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    assert root.find('.//d:segment_size', ns).text == '2097152'
    assert [x.text for x in root.findall('.//d:type', ns)] == ['UDPv4', 'SHM']
    assert root.find('.//d:useBuiltinTransports', ns).text == 'false'


def test_environment_isolated_and_trace_off(tmp_path):
    xml = tmp_path/'transport.xml'
    xml.write_text('<profiles/>')
    original = {'FASTRTPS_DEFAULT_PROFILES_FILE': '/old.xml', 'PM_DDS_TRACE': '1',
                'PM_RCL_TRACE_PATH': '/tmp/trace.csv',
                'LD_PRELOAD': '/tmp/librcl_receive_trace.so:/tmp/other.so'}
    result = transport_environment(str(xml), original)
    assert result['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(xml)
    assert result['LD_PRELOAD'] == '/tmp/other.so'
    assert 'PM_DDS_TRACE' not in result and 'PM_RCL_TRACE_PATH' not in result
    assert original['PM_DDS_TRACE'] == '1'
    assert transport_environment(str(xml), original, True)['PM_DDS_TRACE'] == '1'


def test_missing_xml_fails_before_start(tmp_path):
    with pytest.raises(ValueError):
        transport_environment(str(tmp_path/'missing.xml'), {})


def test_launch_environment_and_explicit_override(tmp_path):
    context = LaunchContext()
    custom = tmp_path/'custom.xml'
    custom.write_text('<profiles/>')
    context.launch_configurations['dds_profile'] = str(custom)
    for action in launch_transport_actions():
        if isinstance(action, (DeclareLaunchArgument, OpaqueFunction)):
            action.execute(context)
    assert context.environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(custom)
    assert context.launch_configurations['dds_receive_diagnostics'] == 'false'


def test_core_native_diagnostics_default_off():
    from pm_bringup.stack_launch import create_launch_description
    context = LaunchContext()
    for action in create_launch_description(drive_default=False).entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for name in ('openvins_save_total_state', 'openvins_record_timing',
                 'openvins_console_log', 'openvins_parameter_snapshot',
                 'openvins_imu_input_diagnostics', 'wit_timing_diagnostics',
                 'oak_timing_diagnostics', 'mapper_callback_diagnostics',
                 'mapper_executor_diagnostics', 'vio_adapter_diagnostics'):
        assert context.launch_configurations[name] == 'false'
