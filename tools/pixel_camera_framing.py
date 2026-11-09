"""Pixelの露出補正とArUco読み取り範囲をPCで調整する。機体は操作しない。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from fractions import Fraction
import json
import math
import os
from pathlib import Path
import queue
import re
import subprocess
import sys
import threading

PROJECT = Path(__file__).resolve().parents[1]
LOGS = PROJECT / "tools/logs/pixel_camera"
PREFERENCES = LOGS / "framing_settings.json"
MARKER_REFERENCE = LOGS / "marker_reference.json"
DEFAULT_ROIS = {"TL": [450, 115, 515, 185], "TR": [914, 143, 980, 210],
                "BL": [443, 554, 509, 621], "BR": [867, 574, 935, 641]}


def ev_to_steps(ev, step, limits):
    value, increment = float(ev), float(Fraction(str(step)))
    if not math.isfinite(value) or not math.isfinite(increment) or increment <= 0:
        raise ValueError("EVは有限の数値、刻み幅は正数で指定してください")
    result = round(value / increment)
    if not limits[0] <= result <= limits[1]:
        raise ValueError("端末の露出補正範囲を超えています")
    return result


def clamp_roi(box, width, height):
    x0, x1 = sorted((max(0, min(width, round(box[0]))), max(0, min(width, round(box[2])))))
    y0, y1 = sorted((max(0, min(height, round(box[1]))), max(0, min(height, round(box[3])))))
    if x1 - x0 < 12 or y1 - y0 < 12:
        raise ValueError("読み取り範囲は縦横12ピクセル以上にしてください")
    return [x0, y0, x1, y1]


def backend_result(output):
    for line in reversed(output.splitlines()):
        if line.startswith("FRAMING_RESULT="):
            value = json.loads(line.split("=", 1)[1])
            if not value.get("ok"):
                raise RuntimeError(value.get("error", "カメラ処理に失敗しました"))
            return value
    raise RuntimeError(output[-1500:] or "カメラから結果が返りませんでした")


def camera_backend(action, steps, torch=False):
    from pixel_camera_capture import CAMERA_PORT, camera_workspace, record_trial
    workspace = camera_workspace()
    sys.path.insert(0, str(workspace / "python"))
    from pixel_camera import PixelCameraClient
    client = PixelCameraClient(connection="usb", port=CAMERA_PORT)
    client.connect()
    try:
        if action == "status":
            status = client.get_status()
            cameras = client.get_capabilities()["cameras"]
            camera = next(item for item in cameras if item.get("target_fixed_240_supported") and item.get("back_facing"))
            limits = [int(item) for item in re.findall(r"-?\d+", camera["ev_steps_range"])]
            return dict(ok=True, device=status["device"], limits=limits, step=camera["ev_step"])
        configured = client.configure(exposure_compensation_steps=steps, torch=torch)
        if configured.get("exposure_compensation_steps") != steps:
            raise RuntimeError("アプリの更新が必要です。露出補正の設定応答が一致しません")
        if configured.get("torch") is not torch:
            raise RuntimeError("撮影用ライト対応アプリへの更新が必要です")
        if action == "configure":
            return dict(ok=True, configured=configured)
        manifest, capture = record_trial(client, PROJECT, seconds=3, label=f"framing_ev_steps_{steps}_no_motion")
        if capture["state"] != "PASSED":
            raise RuntimeError(capture.get("error", "確認撮影の検証に失敗しました") + " / " + str(manifest))
        metadata = json.loads(Path(capture["metadata_file"]).read_text(encoding="utf-8"))
        return dict(ok=True, configured=configured, capture_json=str(manifest),
                    video_file=capture["video_file"], metadata_file=capture["metadata_file"],
                    ev_steps_requested=metadata.get("ev_steps_requested"),
                    ev_steps_result_values=metadata.get("ev_steps_result_values"), step=metadata.get("ev_step"),
                    torch_requested=metadata.get("torch_requested"),
                    flash_mode_result_values=metadata.get("flash_mode_result_values"))
    finally:
        client.disconnect()


def read_frame(video, seconds=2.5):
    import cv2
    cap = cv2.VideoCapture(str(video))
    try:
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
        cap.set(cv2.CAP_PROP_POS_MSEC, seconds * 1000)
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("確認画像を動画から取得できませんでした")
        if frame.shape[:2] != (720, 1280):
            raise RuntimeError("動画の向きまたはサイズが1280×720と異なります")
        return frame, cap.get(cv2.CAP_PROP_POS_MSEC) / 1000
    finally:
        cap.release()


def inspect_rois(frame, rois, refine_corners=False):
    import cv2
    import numpy as np
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    result = {}
    for name, box in rois.items():
        x0, y0, x1, y1 = clamp_roi(box, frame.shape[1], frame.shape[0])
        crop = frame[y0:y1, x0:x1]
        corners, ids, _ = detector.detectMarkers(crop)
        method = "native"
        if ids is None:
            padded = cv2.copyMakeBorder(crop, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=(255, 255, 255))
            corners, ids, _ = detector.detectMarkers(cv2.resize(padded, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
            method = "padded_3x" if ids is not None else "unread"
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        if ids is None:
            contrast = cv2.createCLAHE(clipLimit=2, tileGridSize=(4, 4)).apply(gray)
            contrast = cv2.resize(contrast, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
            contrast = cv2.copyMakeBorder(contrast, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
            attempts = []
            for constant in (3, 7):
                params = cv2.aruco.DetectorParameters()
                params.adaptiveThreshWinSizeMax = 51
                params.adaptiveThreshWinSizeStep = 4
                params.adaptiveThreshConstant = constant
                attempts.append(cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params).detectMarkers(contrast))
            if all(attempt[1] is not None for attempt in attempts) and sorted(attempts[0][1].ravel().tolist()) == sorted(attempts[1][1].ravel().tolist()):
                corners, ids, _ = attempts[-1]
                method = "clahe_2x_two_thresholds_agree"
        coordinates = []
        if ids is not None:
            for quad in corners:
                points = quad.reshape(4, 2).astype(float)
                if method == "padded_3x":
                    points = (points + .5) / 3 - .5 - 20
                elif method == "clahe_2x_two_thresholds_agree":
                    points = (points - 20 + .5) / 2 - .5
                points += [x0, y0]
                coordinates.append(points.tolist())
        result[name] = dict(roi=[x0, y0, x1, y1], ids=[] if ids is None else ids.flatten().tolist(),
                            corners_px=coordinates,
                            method=method, mean_gray=float(gray.mean()),
                            white_clipped_fraction=float(np.mean(gray >= 250)))
    if refine_corners:
        from pixel_camera_reference import refine_marker_vertices
        result = refine_marker_vertices(frame, result)
    return result


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def gui(initial_video=None):
    import tkinter as tk
    from tkinter import ttk, messagebox
    import cv2
    from PIL import Image, ImageTk
    from pixel_camera_capture import camera_workspace, camera_python

    class FramingWindow:
        def __init__(self, root):
            self.root, self.frame, self.photo, self.source = root, None, None, None
            self.result, self.drag, self.busy = {}, None, False
            self.step, self.limits, self.configured_steps = "1/6", [-24, 24], None
            self.rois = {name: list(box) for name, box in DEFAULT_ROIS.items()}
            saved = {}
            if PREFERENCES.is_file():
                saved = json.loads(PREFERENCES.read_text(encoding="utf-8"))
                self.rois.update({name: clamp_roi(box, 1280, 720) for name, box in saved.get("rois", {}).items() if name in self.rois})
            self.queue = queue.Queue()
            self.marker_reference_path = Path(saved.get("marker_reference_file", MARKER_REFERENCE))
            self.marker_reference = json.loads(self.marker_reference_path.read_text(encoding="utf-8")) if self.marker_reference_path.is_file() else None
            root.title("Pixel 撮影調整 — 明るさ・ArUco")
            root.geometry("1120x850")
            controls = ttk.Frame(root, padding=10)
            controls.pack(fill="x")
            ttk.Label(controls, text="露出補正 EV").pack(side="left")
            self.ev = tk.StringVar(value="0")
            self.spin = ttk.Spinbox(controls, from_=-4, to=4, increment=1/6, width=9, textvariable=self.ev, format="%.2f")
            self.spin.pack(side="left", padx=8)
            self.torch = tk.BooleanVar(value=False)
            ttk.Checkbutton(controls, text="撮影用ライト", variable=self.torch).pack(side="left", padx=5)
            self.buttons = []
            for text, action in [("明るさを適用", "configure"), ("確認撮影 3秒", "record"), ("接続を確認", "status")]:
                button = ttk.Button(controls, text=text, command=lambda a=action: self.start(a))
                button.pack(side="left", padx=5)
                self.buttons.append(button)
            self.status = tk.StringVar(value="接続状態を確認しています…")
            ttk.Label(root, textvariable=self.status, padding=(10, 2), wraplength=1070).pack(fill="x")
            ttk.Label(root, text="＋で明るく／−で暗く。適用後「確認撮影」で画像を更新します。録画は720p・240 fps、機体は動かしません。", padding=10).pack(fill="x")
            roi_controls = ttk.Frame(root, padding=(10, 3))
            roi_controls.pack(fill="x")
            ttk.Label(roi_controls, text="読み取り範囲").pack(side="left")
            self.selected = tk.StringVar(value="TL")
            ttk.Combobox(roi_controls, textvariable=self.selected, values=list(DEFAULT_ROIS), state="readonly", width=6).pack(side="left", padx=8)
            ttk.Label(roi_controls, text="左上TL／右上TR／左下BL／右下BRを選び、画像をドラッグ").pack(side="left")
            ttk.Button(roi_controls, text="範囲を保存", command=self.save).pack(side="right")
            self.marker_correction = tk.BooleanVar(value=saved.get("marker_correction", True))
            ttk.Checkbutton(roi_controls, text="読めたマーカーで補正", variable=self.marker_correction,
                            command=self.update_image).pack(side="right", padx=8)
            self.edit_roi = tk.BooleanVar(value=False)
            ttk.Checkbutton(roi_controls, text="範囲を編集", variable=self.edit_roi).pack(side="right", padx=5)
            self.canvas = tk.Canvas(root, width=1024, height=576, background="#222", highlightthickness=0)
            self.canvas.pack(padx=10, pady=5)
            self.canvas.bind("<ButtonPress-1>", self.begin_drag)
            self.canvas.bind("<B1-Motion>", self.move_drag)
            self.canvas.bind("<ButtonRelease-1>", self.end_drag)
            self.details = tk.StringVar(value="確認撮影の画像を待っています")
            ttk.Label(root, textvariable=self.details, padding=10, wraplength=1080).pack(fill="x")
            if initial_video:
                self.load_video(initial_video)
            self.root.after(100, self.poll)
            self.start("status")

        def start(self, action):
            if self.busy:
                return
            try:
                steps = ev_to_steps(self.ev.get(), self.step, self.limits) if action != "status" else 0
            except (ValueError, ZeroDivisionError) as error:
                messagebox.showerror("設定を確認してください", str(error))
                return
            self.busy = True
            selected_torch = self.torch.get()
            for button in self.buttons:
                button.state(["disabled"])
            self.status.set("確認撮影・転送・検証中…" if action == "record" else "カメラへ接続しています…")
            def worker():
                try:
                    command = [str(camera_python(camera_workspace())), str(Path(__file__).resolve()), "--backend", action, "--steps", str(steps)]
                    if selected_torch:
                        command.append("--torch")
                    completed = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", timeout=190,
                                               env=dict(os.environ, PYTHONUTF8="1"),
                                               creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
                    self.queue.put((action, backend_result(completed.stdout), None))
                except Exception as error:
                    self.queue.put((action, None, str(error)))
            threading.Thread(target=worker, daemon=True).start()

        def poll(self):
            try:
                action, value, error = self.queue.get_nowait()
                self.busy = False
                for button in self.buttons:
                    button.state(["!disabled"])
                if error:
                    self.status.set("失敗：" + error)
                else:
                    if action == "status":
                        self.step, self.limits = value["step"], value["limits"]
                        self.spin.configure(from_=self.limits[0]*float(Fraction(self.step)), to=self.limits[1]*float(Fraction(self.step)), increment=float(Fraction(self.step)))
                        steps = value["device"].get("exposure_compensation_steps")
                        self.torch.set(bool(value["device"].get("torch", False)))
                        if steps is None:
                            raise RuntimeError("露出補正対応アプリへの更新が必要です")
                    else:
                        steps = value["configured"]["exposure_compensation_steps"]
                    self.configured_steps = steps
                    self.ev.set(f"{steps*float(Fraction(self.step)):.2f}")
                    self.status.set(f"接続済み：次の撮影は {self.ev.get()} EV（{steps}段階／1段階 {self.step} EV）")
                    if action == "record":
                        self.load_video(value["video_file"], value)
                        actual = value.get("ev_steps_result_values")
                        self.status.set(f"確認撮影完了：要求 {steps}段階 / 実測応答 {actual}。画像は撮影終盤の1フレームです。")
            except queue.Empty:
                pass
            except Exception as error:
                self.status.set("失敗：" + str(error))
            self.root.after(100, self.poll)

        def load_video(self, video, source=None):
            self.frame, pts = read_frame(video)
            self.source = dict(source or {}, video_file=str(video), frame_pts_seconds=pts, orientation_auto=False)
            self.update_image()
            if source:
                self.save_review()

        def update_image(self):
            if self.frame is None:
                return
            from pixel_camera_reference import detect_board_reference
            refined = bool(self.marker_reference and self.marker_reference.get("corner_refinement") == "subpix_original_win3")
            self.result = inspect_rois(self.frame, self.rois, refine_corners=refined)
            self.reference = detect_board_reference(self.frame, self.result, self.marker_reference) if self.marker_correction.get() else dict(available=False, reason="disabled")
            rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            self.photo = ImageTk.PhotoImage(Image.fromarray(rgb).resize((1024, 576)))
            self.canvas.delete("all")
            self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
            for name, item in self.result.items():
                coords = [value*.8 for value in item["roi"]]
                color = "#50ff80" if item["ids"] else "#ffcc44"
                self.canvas.create_rectangle(*coords, outline=color, width=2)
                for quad in item["corners_px"]:
                    points = [coordinate*.8 for point in quad for coordinate in point]
                    self.canvas.create_polygon(*points, outline=color, fill="", width=2)
                label = f"{name}: " + ("ID " + ",".join(map(str, item["ids"])) if item["ids"] else "未読")
                self.canvas.create_text(coords[0], coords[1]-3, anchor="sw", text=label, fill=color, font=("Yu Gothic UI", 12, "bold"))
            if self.reference.get("available"):
                for point in self.reference["points_px"].values():
                    x, y = [value*.8 for value in point]
                    self.canvas.create_line(x-7, y, x+7, y, fill="#55ffff", width=2)
                    self.canvas.create_line(x, y-7, x, y+7, fill="#55ffff", width=2)
            count = sum(bool(item["ids"]) for item in self.result.values())
            if self.reference.get("available"):
                reference_status = f"射影補正OK：ID {self.reference['used_marker_ids']}・{self.reference['inlier_count']}頂点、頂点対応RMS {self.reference['reprojection_rms_px']:.2f} px"
                grid_quality = self.reference.get("grid_quality")
                if grid_quality:
                    reference_status += f" / この画像の未使用格子点RMS {grid_quality['position_rmse_mm']:.2f} mm"
                evaluation = self.reference.get("accuracy_evaluation") or {}
                case = ','.join(map(str, sorted(self.reference['used_marker_ids'])))
                accuracy = evaluation.get("cases", {}).get(case)
                if accuracy and not grid_quality:
                    reference_status += f" / 盤面位置RMS {accuracy['position_rmse_mm']:.2f} mm（保存動画検査）"
            else:
                reference_status = "補正未確定：" + self.reference.get("reason", "unknown")
            self.details.set(f"ArUco {count}/4 / {reference_status}。水色は補正後の盤面基準点です。" +
                             "\n" + "  |  ".join(f"{name}: {item['ids'] or '未読'}・白とび {item['white_clipped_fraction']:.1%}" for name, item in self.result.items()))

        def begin_drag(self, event):
            if self.frame is not None and not self.busy and self.edit_roi.get():
                self.drag = (event.x/.8, event.y/.8)

        def move_drag(self, event):
            if self.drag:
                self.canvas.delete("drag")
                self.canvas.create_rectangle(self.drag[0]*.8, self.drag[1]*.8, event.x, event.y, outline="cyan", width=2, tags="drag")

        def end_drag(self, event):
            if self.drag:
                try:
                    self.rois[self.selected.get()] = clamp_roi([*self.drag, event.x/.8, event.y/.8], 1280, 720)
                    self.update_image()
                except ValueError:
                    self.canvas.delete("drag")
                finally:
                    self.drag = None

        def save_review(self):
            if not self.source or not self.source.get("capture_json"):
                return
            folder = LOGS / "framing" / Path(self.source["capture_json"]).name.removesuffix(".capture.json")
            folder.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(folder / "frame.png"), self.frame)
            profile_snapshot = folder / "marker_reference.json"
            if self.marker_reference is not None:
                write_json(profile_snapshot, self.marker_reference)
            write_json(folder / "inspection.json", dict(source=self.source, dictionary="DICT_4X4_50", corners=self.result,
                                                        board_reference=self.reference,
                                                        marker_reference_file=str(profile_snapshot) if self.marker_reference is not None else None,
                                                        source_marker_reference_file=str(self.marker_reference_path),
                                                        inspected_at=datetime.now(timezone.utc).isoformat()))
            if self.reference.get("available"):
                from pixel_camera_reference import rectify_board
                cv2.imwrite(str(folder / "rectified_board.png"), rectify_board(self.frame, self.reference))

        def save(self):
            try:
                value = dict(saved_at=datetime.now(timezone.utc).isoformat(), image_size=[1280, 720],
                             orientation_auto=False, dictionary="DICT_4X4_50", rois=self.rois,
                             configured_ev_steps=self.configured_steps, ev_step=self.step)
                value["marker_correction"] = self.marker_correction.get()
                value["marker_reference_file"] = str(self.marker_reference_path)
                history = LOGS / "framing" / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_settings.json")
                write_json(history, value)
                write_json(PREFERENCES, value)
                self.save_review()
                self.status.set("読み取り範囲を保存しました：" + str(PREFERENCES))
            except Exception as error:
                messagebox.showerror("保存に失敗しました", str(error))

    root = tk.Tk()
    FramingWindow(root)
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("status", "configure", "record"))
    parser.add_argument("--steps", type=int, default=0)
    parser.add_argument("--torch", action="store_true", help="撮影中だけPixelの補助ライトを点灯")
    parser.add_argument("--video", type=Path, help="保存動画を最初に表示（再撮影なし）")
    args = parser.parse_args()
    if args.backend:
        try:
            value = camera_backend(args.backend, args.steps, args.torch)
        except Exception as error:
            value = dict(ok=False, error=type(error).__name__ + ": " + str(error))
        print("FRAMING_RESULT=" + json.dumps(value, ensure_ascii=False), flush=True)
        return 0 if value["ok"] else 1
    gui(args.video)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
