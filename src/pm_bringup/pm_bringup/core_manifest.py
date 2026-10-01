"""pm_coreの実行条件をparameterとして保持する、処理負荷の小さいmetadata node。"""
import rclpy
from rclpy.node import Node


def main(args=None):
    """センサや指令を扱わず、mission recorderからのparameter取得にだけ応答する。"""
    rclpy.init(args=args)
    node = Node('pm_core_manifest')
    node.declare_parameter('manifest_json', '{}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
