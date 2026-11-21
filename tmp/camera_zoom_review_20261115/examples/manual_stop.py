import time

from pixel_camera import PixelCameraClient

camera = PixelCameraClient(connection="usb")
try:
    camera.connect()
    camera.configure()
    camera.start_recording()  # Safety auto-stop is 30 seconds.
    time.sleep(10)
    accepted = camera.stop_recording()
    print("Stop accepted; file finalization and transfer follow:", accepted)
    print(camera.wait_for_transfer().file_path)
finally:
    camera.disconnect()
