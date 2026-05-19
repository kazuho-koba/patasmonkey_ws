import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateBridgeNode(Node):
    """
    入力：/wheel_radians (JointState, 左右のみ、単位はrad)
    出力：/joint_states(JointState, URDFのジョイント名に展開)
    """

    def __init__(self):
        super().__init__("joint_state_bridge")

        # パラメータ
        self.in_topicname = self.declare_parameter("in_topicname", "/wheel_radians").value
        self.out_topicname = self.declare_parameter("out_topicname", "/joint_states").value

        # 入力側のジョイント名（メッセージについている名称）
        self.left_jointname = self.declare_parameter(
            "left_jointname", "left_wheel_joint"
        ).value
        self.right_jointname = self.declare_parameter(
            "right_jointname", "right_wheel_joint"
        ).value

        # 出力側（URDFジョイント名のリスト）
        self.left_wheel_joints = self.declare_parameter(
            "left_wheel_joints",
            ["wheelaxis_leftfront", "wheelaxis_leftrear"],
        ).value
        self.right_wheel_joints = self.declare_parameter(
            "right_wheel_joints",
            ["wheelaxis_rightfront", "wheelaxis_rightrear"],
        ).value

        # URDFのうち、上記に該当しない固定ジョイント（ホイールアームなど、今は可視化しない部分）
        self.static_joints=self.declare_parameter("static_joints", [""]).value
        self.static_joints = [s for s in self.static_joints if isinstance(s, str) and len(s) > 0] # パラメータ未指定の場合は空に直す
        self.static_positions=self.declare_parameter("static_positions", [0.0]).value
        # バリデーション：static_joints と static_positions の長さ合わせ
        if len(self.static_positions) == 0 and len(self.static_joints) > 0:
            # positions未指定なら0埋め
            self.static_positions = [0.0] * len(self.static_joints)

        if len(self.static_joints) != len(self.static_positions):
            self.get_logger().warn(
                "static_joints and static_positions length mismatch. "
                f"static_joints={len(self.static_joints)} static_positions={len(self.static_positions)}. "
                "Will use min length."
            )

        # pub/subの定義
        self.pub = self.create_publisher(JointState, self.out_topicname, 10)
        self.sub = self.create_subscription(JointState, self.in_topicname, self.cb, 50)

        self.get_logger().info(
            f"JointStateBridge started. in={self.in_topicname} out={self.out_topicname}¥n"
            f"left_jointname={self.left_jointname} right_jointname={self.right_jointname}¥n"
            f"  left_wheel_joints={self.left_wheel_joints}\n"
            f"  right_wheel_joints={self.right_wheel_joints}\n"
            f"  static_joints={self.static_joints}\n"
            f"  static_positions={self.static_positions}"
        )

    def cb(self, msg: JointState):
        """コールバック関数"""
        # 入力msgから左右のposition（モータの通算回転角）を取り出す
        name_to_pos = {}
        if msg.name and msg.position and len(msg.name) == len(msg.position):
            for n, p in zip(msg.name, msg.position):
                name_to_pos[n] = p

        if (
            self.left_jointname not in name_to_pos
            or self.right_jointname not in name_to_pos
        ):
            # とりあえずメッセージ内に想定外のデータが入っていたらスキップ
            self.get_logger().debug(
                f"Missing names in input JointState. got={msg.name}"
            )
            return

        left_pos = float(name_to_pos[self.left_jointname])
        right_pos = float(name_to_pos[self.right_jointname])

        out = JointState()
        # stampは入力を引き継ぎ、空なら現在時刻を新たに取得し格納
        out.header = msg.header
        if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()

        # URDFジョイント名に展開
        out.name = []
        out.position = []

        for jn in self.left_wheel_joints:
            out.name.append(jn)
            out.position.append(left_pos)

        for jn in self.right_wheel_joints:
            out.name.append(jn)
            out.position.append(right_pos)

        # ホイール以外のジョイントを固定値で暫定的に補完
        n = min(len(self.static_joints), len(self.static_positions))
        for i in range(n):
            out.name.append(str(self.static_joints[i]))
            out.position.append(float(self.static_positions[i]))

        # velocity/effortは空で暫定（後で必要なら追加する）
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
