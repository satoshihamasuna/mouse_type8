"""調整盤と9×9で共用する色目印のマスク。距離・姿勢の採否判定は呼出し側で行う。"""
from __future__ import annotations

import cv2
import numpy as np


def colour_mask(hsv, colour, detection=None, *, defaults=None):
    settings = dict(defaults or {})
    limits = (detection or {}).get(colour, {})
    allowed = {'minimum_saturation', 'minimum_value', 'minimum_area_px',
               'opening_kernel_size', 'opening_kernel_shape', 'hue_ranges'}
    if not isinstance(limits, dict) or set(limits)-allowed:
        raise ValueError('unknown colour detection setting')
    settings.update(limits)
    ranges = settings.get('hue_ranges', [[0, 10], [170, 179]] if colour == 'red' else [[25, 36]])
    if (not isinstance(ranges, (list, tuple)) or not 1 <= len(ranges) <= 4
            or any(not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                   or any(isinstance(value, bool) or not isinstance(value, (int, np.integer)) for value in bounds)
                   or not 0 <= bounds[0] <= bounds[1] <= 179 for bounds in ranges)):
        raise ValueError('hue_ranges must contain one to four inclusive integer ranges within zero to179')
    saturation, value = settings.get('minimum_saturation', 60), settings.get('minimum_value', 65)
    if not all(isinstance(v, (int, float)) and np.isfinite(v) and 0 <= v <= 255 for v in (saturation, value)):
        raise ValueError('colour thresholds must be finite values between zero and 255')
    area = settings.get('minimum_area_px', 3)
    kernel_size, shape = settings.get('opening_kernel_size', 0), settings.get('opening_kernel_shape', 'rect')
    if not isinstance(area, (int, float)) or not np.isfinite(area) or not 1 <= area <= 3000 or kernel_size not in (0, 3):
        raise ValueError('invalid colour component area or opening kernel size')
    if shape not in ('rect', 'ellipse'):
        raise ValueError('invalid opening kernel shape')
    hue, sat, val = cv2.split(hsv)
    mask = np.zeros(hue.shape, dtype=bool)
    for low, high in ranges:
        mask |= (hue >= low) & (hue <= high)
    mask = (mask & (sat >= saturation) & (val >= value)).astype(np.uint8)*255
    if kernel_size:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT if shape == 'rect' else cv2.MORPH_ELLIPSE,
                                          (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask, area
