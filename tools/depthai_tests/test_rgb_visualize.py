import cv2
import depthai as dai

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()

    video_queue = cam.requestOutput(
        size=(640, 480),
        type=dai.ImgFrame.Type.BGR888p,
        fps=30,
    ).createOutputQueue()

    pipeline.start()

    while True:
        frame = video_queue.get()
        img = frame.getCvFrame()

        cv2.imshow("OAK-D S2 RGB", img)

        # q キーで終了
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
