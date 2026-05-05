import time
import depthai as dai


def ts_ms(timestamp, base_timestamp):
    return (timestamp - base_timestamp).total_seconds() * 1000.0


with dai.Pipeline() as pipeline:
    imu = pipeline.create(dai.node.IMU)

    # 公式例では加速度・ジャイロを同期取得する構成。
    # まずは安定確認のため100Hz程度から始める。
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)

    # 低遅延寄り。複数ストリーム併用時は値を大きくして負荷を下げる余地あり。
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    imu_queue = imu.out.createOutputQueue(maxSize=50, blocking=False)

    pipeline.start()

    base_ts = None
    count = 0
    start_wall = time.time()

    while True:
        imu_data = imu_queue.get()
        packets = imu_data.packets

        for packet in packets:
            accel = packet.acceleroMeter
            gyro = packet.gyroscope

            accel_ts = accel.getTimestampDevice()
            gyro_ts = gyro.getTimestampDevice()

            if base_ts is None:
                base_ts = min(accel_ts, gyro_ts)

            print(
                f"t_acc={ts_ms(accel_ts, base_ts):9.3f} ms | "
                f"acc[m/s^2] x={accel.x: .4f}, y={accel.y: .4f}, z={accel.z: .4f} | "
                f"t_gyro={ts_ms(gyro_ts, base_ts):9.3f} ms | "
                f"gyro[rad/s] x={gyro.x: .4f}, y={gyro.y: .4f}, z={gyro.z: .4f}"
            )

            count += 1

        if time.time() - start_wall > 10.0:
            print(f"\nReceived IMU packets: {count}")
            print("OK")
            break