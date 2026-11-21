from pathlib import Path
__path__.append(str(Path(__file__).resolve().parents[6] / 'Debug_camera/python/pixel_camera'))
from .client import PixelCameraClient
from .models import PixelCameraError, TransferResult
