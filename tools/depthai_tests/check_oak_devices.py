import depthai as dai

devices = dai.Device.getAllAvailableDevices()

print(f"available devices: {len(devices)}")

for i, device in enumerate(devices):
    print(f"\nDevice {i}")
    print(f"raw: {device}")

    for attr in ["mxid", "name", "state", "protocol", "platform"]:
        if hasattr(device, attr):
            print(f"{attr}: {getattr(device, attr)}")

    for method in ["getMxId", "getMxid"]:
        if hasattr(device, method):
            print(f"{method}(): {getattr(device, method)()}")