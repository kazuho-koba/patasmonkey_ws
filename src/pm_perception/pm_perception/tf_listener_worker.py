"""TF受信だけを別executorで処理するopt-in比較用worker。"""

import threading

from rclpy.executors import SingleThreadedExecutor, ShutdownException


class TransformListenerWorker:
    """共有tf2 Bufferへの書込だけを別threadへ移し、grid更新はmapperに残す。

    BufferCoreはlookupとset_transformを内部mutexで保護する。TFの時刻・内容・QoSを
    変えず、mapperと同じnodeを2つのexecutorへ登録することも避ける。Python GILの
    競合は残るため、CPU削減やfusion改善は実測するまでは保証しない。
    """

    def __init__(self, node, csv_path=None, flush_period_sec=1.0):
        if csv_path:
            from pm_perception.executor_diagnostics import MeasuredSingleThreadedExecutor
            self.executor = MeasuredSingleThreadedExecutor(node, csv_path, flush_period_sec)
        else:
            self.executor = SingleThreadedExecutor(context=node.context)
        self.node = node
        self.error = None
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self._spin, name="mapper_tf", daemon=False)
        self.thread.start()

    def _spin(self):
        """worker異常を保存し、mapper側が通常の処理経路で検出できるようにする。"""
        try:
            self.executor.spin()
        except ShutdownException:
            pass
        except Exception as error:
            self.error = error
            self.node.get_logger().error("TF listener worker failed: %s" % error)

    def close(self):
        """executorを起こして終了を待ち、終了済みthreadのCSVを確定する。"""
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            raise RuntimeError("TF listener worker did not stop within 5 seconds")
        self.executor.remove_node(self.node)
        if hasattr(self.executor, "close"):
            self.executor.close()
