from pixel_camera import PixelCameraClient

camera = PixelCameraClient(connection="usb")
try:
    camera.connect()
    camera.configure()
    for index in range(3):
        camera.start_recording(duration_s=10)
        result = camera.wait_for_transfer(timeout=120)
        print(index + 1, result.file_path, result.source_deleted)
finally:
    camera.disconnect()
