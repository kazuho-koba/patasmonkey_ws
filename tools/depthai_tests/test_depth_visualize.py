import cv2
import numpy as np
import depthai as dai


def colorize_depth(depth_frame: np.ndarray, max_depth_mm: int = 5000) -> np.ndarray:
    # depth_frame は uint16 の mm単位を想定
    depth_clipped = np.clip(depth_frame, 0, max_depth_mm)

    # 0mm は無効値のことが多いので黒に残す
    depth_8bit = (255 * (1.0 - depth_clipped / max_depth_mm)).astype(np.uint8)
    depth_8bit[depth_frame == 0] = 0

    return cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)


with dai.Pipeline() as pipeline:
    # 左右のモノクロカメラを作成
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # StereoDepthノードを作成
    stereo = pipeline.create(dai.node.StereoDepth)

    # 左右カメラからStereoDepthへ入力
    left.requestOutput(size=(640, 400), fps=30).link(stereo.left)
    right.requestOutput(size=(640, 400), fps=30).link(stereo.right)

    # 基本設定
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(True)

    stereo.initialConfig.setConfidenceThreshold(
        50
    )  # 距離計測の確信度がこれ以下ならピクセル棄却、200はやや厳しめ
    # stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5) # 5*5のカーネルを使った中央値フィルタにより距離画像平滑化（setSubpixelと同時使用は不可）

    # depth出力キューを作成
    depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

    pipeline.start()

    while True:
        depth_msg = depth_queue.get()
        depth = depth_msg.getFrame()  # uint16, 単位は通常mm

        depth_vis = colorize_depth(depth, max_depth_mm=5000)

        center_y = depth.shape[0] // 2
        center_x = depth.shape[1] // 2
        center_depth = int(depth[center_y, center_x])

        cv2.circle(depth_vis, (center_x, center_y), 5, (255, 255, 255), -1)
        cv2.putText(
            depth_vis,
            f"center: {center_depth} mm",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        cv2.imshow("OAK-D S2 Depth", depth_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

cv2.destroyAllWindows()
