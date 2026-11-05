"""3D twistから2D EKF用の周辺共分散を作る専用の入力境界。

元の3D topicは変更しない。Foxy robot_localizationのforceTwoDはvz/wx/wyの
分散だけを1e-6へ置換するため、これらとvx/vy/wzの相関を残すと共分散が
不定になる。2Dで使用する(vx,vy,wz)の周辺分布をそのまま取り出し、
使用しない次元との交差項を除く。XY速度相互の共分散は保持する。
これは姿勢による水平投影ではなく、2D観測の共分散を構成する処理である。
"""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

from pm_localization.vio_rigid_transform import _covariance


def planar_twist_covariance(covariance):
    """[vx,vy,vz,wx,wy,wz]から、平面内/平面外の相関を分離する。

    平面内の3×3主小行列は周辺共分散なので、元がPSDならPSDを保つ。
    平面外は独立に扱い、Foxy側で仮想的なゼロ観測へ置換されてもPSDを保つ。
    """
    source = _covariance(covariance)
    result = np.diag(np.diag(source)).copy()
    planar = [0, 1, 5]
    result[np.ix_(planar, planar)] = source[np.ix_(planar, planar)]
    return result


class PlanarTwistCovarianceNode(Node):
    """同じ値・frame・headerで、2D専用topicにのみ周辺共分散をpublish。"""
    def __init__(self):
        super().__init__('planar_twist_covariance_node')
        self.declare_parameter('input_topic', '/vio/odometry/twist_gated')
        self.declare_parameter('output_topic', '/vio/odometry/twist_planar')
        self.declare_parameter('message_type', 'odometry')
        kind = self.get_parameter('message_type').value
        if kind not in ('odometry', 'twist_stamped'):
            raise ValueError('message_typeはodometryまたはtwist_stamped')
        self.odometry = kind == 'odometry'
        message_class = Odometry if self.odometry else TwistWithCovarianceStamped
        self.publisher = self.create_publisher(message_class, self.get_parameter('output_topic').value, 50)
        self.create_subscription(message_class, self.get_parameter('input_topic').value, self.callback, 50)

    def callback(self, message):
        source = message.twist
        try:
            covariance = planar_twist_covariance(source.covariance)
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.get_logger().warn('2D共分散を棄却: '+str(exc))
            return
        output = copy.deepcopy(message)
        output.twist.covariance = covariance.reshape(-1).tolist()
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarTwistCovarianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
