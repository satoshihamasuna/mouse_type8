"""色目印以外の機体画像を剛体登録し、別撮り間の設置変化を独立に確認する。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from pixel_camera_framing import read_frame
from video_marker_pose import marker_registration


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def register(template, current, mask):
    warp = np.eye(2, 3, dtype=np.float32)
    score, warp = cv2.findTransformECC(template, current, warp, cv2.MOTION_EUCLIDEAN,
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 150, 1e-7), mask, 3)
    return float(score), warp, math.degrees(math.atan2(float(warp[1, 0]), float(warp[0, 0])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run = args.trial.resolve()
    session = run.parent
    args.output.mkdir(exist_ok=False)
    protocol = dict(rotation_limit_deg=1, centre_change_limit_mm=3, minimum_correlation=.85,
        minimum_mask_pixels=500, samples_per_window=9, static_window_s=[1.2, 2.4],
        prerun_window_relative_to_motion_s=[-.35, -.15], pixels_per_mm=2,
        roi_half_width_mm=35, roi_half_height_mm=40, mask_ellipse_half_axes_mm=[17, 28],
        exclude_saturation_above=60, exclude_value_above=235,
        synthetic_angles_deg=[0, .5, 1.5], synthetic_maximum_abs_angle_error_deg=.15,
        method='Euclidean ECC only; no scale fit, no endpoint used; body pixels excluding saturated colours and glare',
        scope='Supplemental relative setup comparison; not absolute body yaw validation')
    (args.output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n', encoding='utf-8')
    config = load(session/'config.used.json')
    reference = load(config['reference'])
    static = load(session/'static_preflight.json')
    trial = load(run/'trial.json')
    report = load(run/'full/report.json')
    log = pd.read_csv(run/'motion.csv')
    motion_start = report['log_time_zero_at_video_s']+(trial['log_quality']['motion_start_time_ms']-float(log.time_ms.iloc[0]))%2**32/1000
    sparse = load(session/'static_preflight_check.json')
    centre = np.median([item['centre_board_xy_mm'] for item in sparse['frames']], axis=0)
    scale = protocol['pixels_per_mm']
    width, height = 140, 160
    to_patch = np.array([[scale, 0, width/2-scale*centre[0]],
                         [0, scale, height/2-scale*centre[1]], [0, 0, 1]], dtype=float)

    def median_patch(video, stamps):
        images, checks = [], []
        for stamp in stamps:
            frame, actual = read_frame(video, float(stamp))
            correction = marker_registration(frame, reference)
            assert correction.get('available'), correction
            matrix = to_patch @ np.asarray(correction['homography_image_to_board_mm'])
            images.append(cv2.warpPerspective(frame, matrix, (width, height)))
            checks.append(dict(requested_s=float(stamp), actual_s=actual, grid_quality=correction['grid_quality']))
        return np.median(images, axis=0).astype(np.uint8), checks

    before, checks1 = median_patch(static['video_file'], np.linspace(1.2, 2.4, 9))
    current, checks2 = median_patch(trial['video_file'], np.linspace(motion_start-.35, motion_start-.15, 9))
    hsv = cv2.cvtColor(before, cv2.COLOR_BGR2HSV)
    exclude = ((hsv[:, :, 1] > 60) | (hsv[:, :, 2] > 235)).astype('uint8')
    exclude = cv2.dilate(exclude, np.ones((3, 3), 'uint8'))
    mask = np.zeros((height, width), np.uint8)
    cv2.ellipse(mask, (width//2, height//2), (34, 56), 0, 0, 360, 255, -1)
    mask[exclude.astype(bool)] = 0
    assert np.count_nonzero(mask) >= protocol['minimum_mask_pixels']
    convert = lambda image: cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)/255, (3, 3), 0)
    template, input_image = convert(before), convert(current)
    assert template[mask > 0].std() > .03
    synthetic = []
    for angle in protocol['synthetic_angles_deg']:
        known = cv2.getRotationMatrix2D((width/2, height/2), angle, 1)
        known[:, 2] += [1.0, -.5]
        moved = cv2.warpAffine(template, known, (width, height))
        score, warp, measured = register(template, moved, mask)
        error = abs(abs(measured)-abs(angle))
        synthetic.append(dict(requested_angle_deg=angle, measured_angle_deg=measured, error_deg=error, correlation=score))
        assert error <= .15 and score >= .85, synthetic[-1]
    score, warp, angle = register(template, input_image, mask)
    midpoint = np.array([width/2, height/2, 1.0])
    delta = (warp @ midpoint-midpoint[:2])/scale
    accepted = score >= .85 and abs(angle) <= 1 and np.linalg.norm(delta) <= 3
    aligned = cv2.warpAffine(current, warp, (width, height), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    for name, image in [('static_body.png', before), ('prerun_body.png', current), ('body_mask.png', mask),
                        ('aligned_overlay.png', cv2.addWeighted(before, .5, aligned, .5, 0))]:
        cv2.imwrite(str(args.output/name), image)
    paths = [run/'trial.json', run/'full/report.json', session/'static_preflight.json',
        session/'static_preflight_check.json', Path(static['video_file']), Path(trial['video_file']),
        Path(config['reference']), Path(__file__)]
    result = dict(state='PASSED' if accepted else 'FAILED', correlation=score,
        relative_rotation_deg=angle, centre_translation_mm=delta.tolist(),
        mask_pixels=int(np.count_nonzero(mask)), synthetic_validation=synthetic,
        static_frames=checks1, prerun_frames=checks2, protocol=protocol, warp=warp.tolist(),
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    (args.output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('state', 'correlation', 'relative_rotation_deg', 'centre_translation_mm', 'mask_pixels', 'synthetic_validation')}), flush=True)


if __name__ == '__main__':
    main()
