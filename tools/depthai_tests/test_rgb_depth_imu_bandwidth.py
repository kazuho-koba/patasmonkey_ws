import time
from collections import deque

import cv2
import depthai as dai
import numpy as np


# =========================
# パラメータ設定
# =========================

# RGB
RGB_WIDTH = 1280
RGB_HEIGHT = 800
RGB_FPS = 30

# Depth (Stereo)
DEPTH_WIDTH = 1280
DEPTH_HEIGHT = 800
DEPTH_FPS = 30
DEPTH_MAX_MM = 10000

# IMU
IMU_FREQ = 200  # Hz

# Stereo設定
USE_SUBPIXEL = True
USE_LR_CHECK = True
CONFIDENCE_THRESHOLD = 50

# =========================


def colorize_depth(depth_frame: np.ndarray, max_depth_mm: int):
    depth_clipped = np.clip(depth_frame, 0, max_depth_mm)
    depth_8bit = (255 * (1.0 - depth_clipped / max_depth_mm)).astype(np.uint8)
    depth_8bit[depth_frame == 0] = 0
    return cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)


def fps_from_times(times: deque):
    if len(times) < 2:
        return 0.0
    dt = times[-1] - times[0]
    return (len(times) - 1) / dt if dt > 0 else 0.0


with dai.Pipeline() as pipeline:
    # =========================
    # RGB
    # =========================
    rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    rgb_queue = rgb.requestOutput(
        size=(RGB_WIDTH, RGB_HEIGHT),
        type=dai.ImgFrame.Type.BGR888p,
        fps=RGB_FPS,
    ).createOutputQueue(maxSize=4, blocking=False)

    # =========================
    # Stereo Depth
    # =========================
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)

    left.requestOutput(size=(DEPTH_WIDTH, DEPTH_HEIGHT), fps=DEPTH_FPS).link(stereo.left)
    right.requestOutput(size=(DEPTH_WIDTH, DEPTH_HEIGHT), fps=DEPTH_FPS).link(stereo.right)

    stereo.setLeftRightCheck(USE_LR_CHECK)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(USE_SUBPIXEL)

    stereo.initialConfig.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)

    depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

    # =========================
    # IMU
    # =========================
    imu = pipeline.create(dai.node.IMU)

    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, IMU_FREQ)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, IMU_FREQ)

    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    imu_queue = imu.out.createOutputQueue(maxSize=50, blocking=False)

    # =========================

    pipeline.start()

    rgb_times = deque(maxlen=60)
    depth_times = deque(maxlen=60)
    imu_times = deque(maxlen=300)

    latest_rgb = None
    latest_depth_vis = None
    latest_center_depth = 0
    latest_acc = None
    latest_gyro = None

    last_print = time.time()

    print("Running test. Press 'q' to quit.")

    while True:
        now = time.time()

        # RGB
        msg = rgb_queue.tryGet()
        if msg is not None:
            latest_rgb = msg.getCvFrame()
            rgb_times.append(now)

        # Depth
        msg = depth_queue.tryGet()
        if msg is not None:
            depth = msg.getFrame()
            latest_depth_vis = colorize_depth(depth, DEPTH_MAX_MM)

            cy = depth.shape[0] // 2
            cx = depth.shape[1] // 2
            latest_center_depth = int(depth[cy, cx])

            cv2.circle(latest_depth_vis, (cx, cy), 5, (255, 255, 255), -1)
            depth_times.append(now)

        # IMU
        msg = imu_queue.tryGet()
        if msg is not None:
            for p in msg.packets:
                acc = p.acceleroMeter
                gyro = p.gyroscope

                latest_acc = (acc.x, acc.y, acc.z)
                latest_gyro = (gyro.x, gyro.y, gyro.z)

                imu_times.append(now)

        rgb_fps = fps_from_times(rgb_times)
        depth_fps = fps_from_times(depth_times)
        imu_rate = fps_from_times(imu_times)

        # 描画
        if latest_rgb is not None:
            img = latest_rgb.copy()
            cv2.putText(img, f"RGB FPS: {rgb_fps:.1f}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("RGB", img)

        if latest_depth_vis is not None:
            img = latest_depth_vis.copy()
            cv2.putText(img, f"Depth FPS: {depth_fps:.1f}", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow("Depth", img)

        # ログ
        if now - last_print > 1.0:
            last_print = now

            acc_str = "None"
            if latest_acc:
                norm = (latest_acc[0]**2 + latest_acc[1]**2 + latest_acc[2]**2) ** 0.5
                acc_str = f"{norm:.2f} m/s^2"

            print(
                f"RGB={rgb_fps:5.1f} fps | "
                f"Depth={depth_fps:5.1f} fps | "
                f"IMU={imu_rate:6.1f} Hz | "
                f"depth={latest_center_depth:4d} mm | "
                f"|acc|={acc_str}"
            )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()