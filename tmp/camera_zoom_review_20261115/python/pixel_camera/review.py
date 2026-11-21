"""Frame-by-frame inspection and contact sheets from saved MP4 files."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
import sys
import uuid

from PySide6.QtCore import QBuffer, QIODevice, QObject, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QImage, QKeySequence, QPainter, QPixmap, QShortcut
from PySide6.QtWidgets import (
    QApplication, QFileDialog, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPushButton, QSlider, QSpinBox, QVBoxLayout, QWidget,
)

from .review_media import FrameReader, scan_video
from .runtime import PROJECT_ROOT
from .transfer import atomic_bytes, atomic_json


def frame_image(frame):
    return QImage(frame.rgb, frame.width, frame.height, frame.stride, QImage.Format.Format_RGB888).copy()


def image_bytes(image):
    buffer = QBuffer()
    if not buffer.open(QIODevice.OpenModeFlag.WriteOnly) or not image.save(buffer, "PNG"):
        raise OSError("PNGの作成に失敗しました")
    return bytes(buffer.data())


def export_frames(index, start, *, count=24, parent=None, reference_note=""):
    if not 0 <= start < index.count or not 1 <= count <= 24:
        raise ValueError("書き出すフレームの範囲が不正です")
    index.ensure_unchanged()
    positions = list(range(start, min(index.count, start + count)))
    folder = (Path(parent) if parent else PROJECT_ROOT / "recordings/reviews") / (
        datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8])
    folder.mkdir(parents=True, exist_ok=False)
    evidence = index.evidence()
    evidence.update(reference_note=reference_note, reference_independently_verified=False, frames=[])
    columns, thumb_w, thumb_h, gap, top = 6, 320, 180, 12, 92
    rows = (len(positions) + columns - 1) // columns
    sheet = QImage(columns * (thumb_w + gap) + gap, top + rows * 222 + 44,
                   QImage.Format.Format_RGB32)
    sheet.fill(QColor("#f4f7fa"))
    painter = QPainter(sheet)
    try:
        painter.setPen(QColor("#132330"))
        painter.setFont(QFont("Yu Gothic UI", 14))
        painter.drawText(12, 26, "Pixel Camera — 連続フレーム検査")
        painter.setFont(QFont("Yu Gothic UI", 9))
        painter.drawText(12, 49, index.path.name)
        painter.drawText(12, 70, "PTSは動画の時刻です。物理的な240コマ/秒の撮影は、独立した時間基準との比較が必要です。")
        with FrameReader(index) as reader:
            for ordinal, position in enumerate(positions):
                frame = reader.get(position)
                image = frame_image(frame)
                if count == 1:
                    atomic_bytes(folder / "frame.png", image_bytes(image))
                x, y = gap + (ordinal % columns) * (thumb_w + gap), top + (ordinal // columns) * 222
                painter.drawImage(x, y, image.scaled(thumb_w, thumb_h, Qt.AspectRatioMode.KeepAspectRatio,
                                                    Qt.TransformationMode.SmoothTransformation))
                at, interval = index.relative_seconds(position), index.interval_ms(position)
                stamp = index.points[position]
                painter.drawText(x, y + 197, f"#{position:05d}   PTS {at * 1000:.3f} ms")
                painter.drawText(x, y + 213, "Δ 前コマ: " + (f"{interval:.3f} ms" if interval is not None else "—"))
                evidence["frames"].append(dict(index=position, source_pts=stamp.pts,
                    time_base=str(stamp.time_base), relative_seconds=at, interval_ms=interval))
        painter.drawText(12, sheet.height() - 16, "検査開始 #" + str(start) + " / source SHA-256: " + index.sha256[:24] + "…")
    finally:
        painter.end()
    index.ensure_unchanged()
    atomic_bytes(folder / "contact_sheet.png", image_bytes(sheet))
    atomic_json(folder / "review.json", evidence)
    return folder


class ReviewSignals(QObject):
    result = Signal(int, str, object)
    error = Signal(int, str)


class ReviewWindow(QMainWindow):
    def __init__(self, path=None):
        super().__init__()
        self.setWindowTitle("Pixel Camera — コマ送り検査")
        self.resize(1120, 860)
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.signals = ReviewSignals()
        self.signals.result.connect(self.completed)
        self.signals.error.connect(self.failed)
        self.generation = 0
        self.closed = False
        self.index = None
        self.reader = None  # Accessed only on the single executor worker.
        self.position = 0
        self.current_image = None
        self.last_export = None

        panel = QWidget()
        self.setCentralWidget(panel)
        layout = QVBoxLayout(panel)
        self.open_button = QPushButton("MP4を開く")
        self.open_button.clicked.connect(self.choose_file)
        self.file_label = QLabel("録画・転送が完了した動画を開いてください")
        self.file_label.setWordWrap(True)
        row = QHBoxLayout()
        row.addWidget(self.open_button)
        row.addWidget(self.file_label, 1)
        layout.addLayout(row)
        self.info = QLabel("")
        layout.addWidget(self.info)
        self.picture = QLabel("保存済み動画を1コマずつ表示します")
        self.picture.setMinimumSize(640, 360)
        self.picture.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.picture.setStyleSheet("background:#17212c;color:white")
        layout.addWidget(self.picture, 1)
        self.timing = QLabel("—")
        layout.addWidget(self.timing)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.valueChanged.connect(self.schedule_seek)
        layout.addWidget(self.slider)
        self.seek_timer = QTimer(self)
        self.seek_timer.setSingleShot(True)
        self.seek_timer.timeout.connect(lambda: self.select_frame(self.slider.value()))
        actions = QHBoxLayout()
        self.previous = QPushButton("← 前のコマ")
        self.next = QPushButton("次のコマ →")
        self.previous.clicked.connect(lambda: self.select_frame(self.position - 1))
        self.next.clicked.connect(lambda: self.select_frame(self.position + 1))
        self.frame_number = QSpinBox()
        self.frame_number.valueChanged.connect(self.select_frame)
        actions.addWidget(self.previous)
        actions.addWidget(self.next)
        actions.addWidget(QLabel("フレーム番号（0から）"))
        actions.addWidget(self.frame_number)
        layout.addLayout(actions)
        self.reference = QLineEdit()
        self.reference.setPlaceholderText("時間基準の説明（任意）例: 別計測で周波数を確認したLED。説明の入力だけで検証済みにはしません")
        layout.addWidget(self.reference)
        export_row = QHBoxLayout()
        self.export_one = QPushButton("この1コマをPNG保存")
        self.export_sheet = QPushButton("ここから24コマの一覧を保存")
        self.export_folder = QPushButton("保存フォルダを開く")
        self.export_folder.setEnabled(False)
        self.export_one.clicked.connect(lambda: self.export(1))
        self.export_sheet.clicked.connect(lambda: self.export(24))
        self.export_folder.clicked.connect(self.open_export)
        for button in (self.export_one, self.export_sheet, self.export_folder):
            export_row.addWidget(button)
        layout.addLayout(export_row)
        note = QLabel("左右キーでコマ送りできます。表示する時刻は復号フレームのPTSです。実際の撮影タイミングは外部の時間基準と比較してください。")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.status = QLabel("待機中")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.shortcuts = [QShortcut(QKeySequence(key), self) for key in (Qt.Key.Key_Left, Qt.Key.Key_Right)]
        self.shortcuts[0].activated.connect(lambda: self.select_frame(self.position - 1) if self.previous.isEnabled() else None)
        self.shortcuts[1].activated.connect(lambda: self.select_frame(self.position + 1) if self.next.isEnabled() else None)
        self.set_busy(True)
        if path:
            self.load(path)

    def set_busy(self, busy):
        ready = self.index is not None and not busy
        self.previous.setEnabled(ready and self.position > 0)
        self.next.setEnabled(ready and self.position < self.index.count - 1)
        for control in (self.slider, self.frame_number, self.export_one, self.export_sheet):
            control.setEnabled(ready)

    def submit(self, operation, function):
        generation = self.generation
        future = self.executor.submit(function)
        def done(value):
            if self.closed:
                return
            try:
                result = value.result()
            except Exception as error:
                self.signals.error.emit(generation, str(error))
            else:
                self.signals.result.emit(generation, operation, result)
        future.add_done_callback(done)

    def choose_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "確定済みMP4を選択", str(PROJECT_ROOT / "recordings/remote"), "MP4 (*.mp4)")
        if path:
            self.load(path)

    def load(self, path):
        self.generation += 1
        self.seek_timer.stop()
        self.index = None
        self.set_busy(True)
        self.file_label.setText(str(path))
        self.status.setText("全フレームの復号とSHA-256を確認中…")
        def scan():
            if self.reader:
                self.reader.close()
                self.reader = None
            index = scan_video(path)
            self.reader = FrameReader(index)
            return index, self.reader.get(0)
        self.submit("load", scan)

    def schedule_seek(self, value):
        self.seek_timer.start(80)

    def select_frame(self, position):
        if self.index is None or not self.slider.isEnabled() or not 0 <= position < self.index.count:
            return
        self.seek_timer.stop()
        self.set_busy(True)
        self.status.setText(f"フレーム #{position} を復号中…")
        self.submit("frame", lambda: self.reader.get(position))

    def completed(self, generation, operation, value):
        if generation != self.generation:
            return
        if operation == "load":
            self.index, frame = value
            self.slider.setRange(0, self.index.count - 1)
            self.frame_number.setRange(0, self.index.count - 1)
            evidence = self.index.evidence()
            fps = evidence["fps_from_decoded_pts"]
            self.info.setText(f"{self.index.width}×{self.index.height} / 復号 {self.index.count} コマ / "
                              + (f"PTS実測 {fps:.3f} fps" if fps is not None else "PTS実測fps不明"))
            self.show_frame(frame)
        elif operation == "frame":
            self.show_frame(value)
        elif operation == "export":
            self.last_export = value
            self.export_folder.setEnabled(True)
            self.status.setText("保存しました: " + str(value))
            self.set_busy(False)

    def show_frame(self, frame):
        self.position = frame.index
        self.current_image = frame_image(frame)
        for control in (self.slider, self.frame_number):
            control.blockSignals(True)
            control.setValue(self.position)
            control.blockSignals(False)
        interval = self.index.interval_ms(self.position)
        self.timing.setText(f"#{self.position} / {self.index.count - 1}   PTS {self.index.relative_seconds(self.position):.6f} 秒"
                            + (f"   前コマとの差 {interval:.3f} ms" if interval is not None else ""))
        self.render_image()
        self.status.setText("左右キーまたはボタンで1コマずつ確認できます")
        self.set_busy(False)

    def render_image(self):
        if self.current_image is not None:
            self.picture.setPixmap(QPixmap.fromImage(self.current_image).scaled(
                self.picture.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "current_image"):
            self.render_image()

    def export(self, count):
        if self.index is None or not self.export_sheet.isEnabled():
            return
        index, position, note = self.index, self.position, self.reference.text()
        self.set_busy(True)
        self.status.setText("フレーム画像と時刻の一覧を保存中…")
        self.submit("export", lambda: export_frames(index, position, count=count, reference_note=note))

    def open_export(self):
        if self.last_export:
            from PySide6.QtCore import QUrl
            from PySide6.QtGui import QDesktopServices
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.last_export)))

    def failed(self, generation, message):
        if generation == self.generation:
            self.set_busy(False)
            self.status.setText("エラー: " + message)

    def closeEvent(self, event):
        self.closed = True
        self.seek_timer.stop()
        def cleanup():
            if self.reader:
                self.reader.close()
        self.executor.submit(cleanup)
        self.executor.shutdown(wait=False, cancel_futures=False)
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", nargs="?", type=Path)
    parser.add_argument("--export-sheet", action="store_true", help="Export a contact sheet and exit")
    parser.add_argument("--start", type=int, default=0, help="First decoded frame index, starting at 0")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    if args.export_sheet:
        if args.video is None:
            parser.error("--export-sheet requires a video")
        print(export_frames(scan_video(args.video), args.start, parent=args.output), flush=True)
        return
    window = ReviewWindow(args.video)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
