from pixel_camera import PixelCameraClient

camera = PixelCameraClient(connection="usb")
try:
    camera.connect()
    camera.configure(width=1280, height=720, fps=240)
    camera.start_recording(duration_s=10)
    result = camera.wait_for_transfer(timeout=120)
    print(result.file_path)
    print(f"Encoded frames: {result.frame_count} ({result.frame_count_source})")
    print(f"PC hash verified; phone source deleted: {result.source_deleted}")
finally:
    camera.disconnect()
