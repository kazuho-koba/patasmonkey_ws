import depthai as dai

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()

    video_queue = cam.requestOutput(
        size=(300, 300),
        type=dai.ImgFrame.Type.BGR888p,
        fps=15,
    ).createOutputQueue()

    pipeline.start()

    for i in range(30):
        frame = video_queue.get()
        img = frame.getCvFrame()
        print(f"frame {i}: shape={img.shape}, dtype={img.dtype}")

print("OK")