from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
import os
import subprocess
import sys
import threading

from PySide6.QtCore import QObject, QTimer, QUrl, Signal
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QPlainTextEdit, QProgressBar, QPushButton,
    QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from .client import PixelCameraClient
from .runtime import DEFAULT_RUNTIME, config
from .stability import run_stability


class Signals(QObject):
    result = Signal(str, object)
    failure = Signal(str, object)
    progress = Signal(str)


class CameraWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pixel Camera — USB遠隔録画")
        self.resize(960, 820)
        self.executor = ThreadPoolExecutor(max_workers=3)
        self.signals = Signals()
        self.signals.result.connect(self.finished)
        self.signals.failure.connect(self.failed)
        self.client = None
        self.polling = False
        self.server_ready = False
        self.action_busy = False
        self.closed = False
        self.batch_running = False
        self.cancel_batch = threading.Event()
        self.last_report = None
        value = config(DEFAULT_RUNTIME, 8765)
        self.history_values = []
        panel = QWidget()
        self.setCentralWidget(panel)
        layout = QVBoxLayout(panel)

        connection = QGroupBox("1. 接続")
        form = QFormLayout(connection)
        self.method = QComboBox()
        self.method.addItem("USB (ADB)")
        self.directory = QLineEdit(value["save_dir"])
        browse = QPushButton("保存先を選ぶ")
        browse.clicked.connect(self.browse)
        directory_row = QHBoxLayout()
        directory_row.addWidget(self.directory)
        directory_row.addWidget(browse)
        self.code = QLineEdit(value["token"])
        self.code.setReadOnly(True)
        copy = QPushButton("接続コードをコピー")
        copy.clicked.connect(lambda: QApplication.clipboard().setText(self.code.text()))
        code_row = QHBoxLayout()
        code_row.addWidget(self.code)
        code_row.addWidget(copy)
        self.connect_button = QPushButton("接続 / 再接続")
        self.connect_button.clicked.connect(self.connect_camera)
        self.disconnect_button = QPushButton("操作クライアントを切断")
        self.disconnect_button.clicked.connect(self.disconnect_camera)
        self.shutdown_button = QPushButton("PCサーバーを停止")
        self.shutdown_button.clicked.connect(self.shutdown_server)
        controls = QHBoxLayout()
        for button in (self.connect_button, self.disconnect_button, self.shutdown_button):
            controls.addWidget(button)
        self.connection_label = QLabel("スマホに接続コードを入力し「PC接続を開始」を押してください。")
        form.addRow("接続方法", self.method)
        form.addRow("PC保存先", directory_row)
        form.addRow("スマホ用接続コード", code_row)
        form.addRow(controls)
        form.addRow(self.connection_label)
        layout.addWidget(connection)

        recording = QGroupBox("2. 録画")
        capture = QFormLayout(recording)
        self.mode = QComboBox()
        self.mode.addItem("1280 × 720 / 240 fps / H.264")
        self.mode.setEnabled(False)
        self.duration = QSpinBox()
        self.duration.setRange(1, 30)
        self.duration.setValue(10)
        self.duration.setSuffix(" 秒")
        self.start_button = QPushButton("録画開始")
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_recording)
        self.stop_button = QPushButton("録画停止")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_recording)
        actions = QHBoxLayout()
        actions.addWidget(self.start_button)
        actions.addWidget(self.stop_button)
        self.status_label = QLabel("未接続")
        self.elapsed_label = QLabel("0.00 秒")
        self.space_label = QLabel("—")
        capture.addRow("撮影モード", self.mode)
        capture.addRow("自動停止", self.duration)
        capture.addRow(actions)
        capture.addRow("状態", self.status_label)
        capture.addRow("撮影経過", self.elapsed_label)
        capture.addRow("端末空き容量", self.space_label)
        capture.addRow(QLabel("露出・フォーカス・ホワイトバランスは自動です。"))
        self.repetitions = QSpinBox()
        self.repetitions.setRange(1, 1000)
        self.repetitions.setValue(20)
        self.repetitions.setSuffix(" 回")
        self.batch_button = QPushButton("連続試験を開始")
        self.batch_button.setEnabled(False)
        self.batch_button.clicked.connect(self.start_batch)
        self.cancel_button = QPushButton("現在の保存後に中止")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self.cancel_batch.set)
        self.report_button = QPushButton("試験レポートを開く")
        self.report_button.setEnabled(False)
        self.report_button.clicked.connect(self.open_report)
        batch_actions = QHBoxLayout()
        for button in (self.batch_button, self.cancel_button, self.report_button):
            batch_actions.addWidget(button)
        capture.addRow("連続試験の回数", self.repetitions)
        capture.addRow(batch_actions)
        layout.addWidget(recording)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.speed = QLabel("転送待ち — 照合成功後だけ端末の元動画を削除します")
        layout.addWidget(self.progress)
        layout.addWidget(self.speed)
        self.history_table = QTableWidget(0, 5)
        self.history_table.setHorizontalHeaderLabels(["PC受信時刻", "フレーム数", "サイズ (MiB)", "端末動画削除", "ファイル"])
        self.history_table.horizontalHeader().setStretchLastSection(True)
        self.history_table.cellDoubleClicked.connect(self.open_recording)
        layout.addWidget(self.history_table)
        self.review_button = QPushButton("選択した動画をコマ送りで検査")
        self.review_button.setEnabled(False)
        self.review_button.clicked.connect(self.open_review)
        layout.addWidget(self.review_button)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)
        self.log.setMaximumHeight(120)
        layout.addWidget(self.log)
        self.signals.progress.connect(self.log.appendPlainText)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(1000)

    def submit(self, name, function):
        future = self.executor.submit(function)
        def done(value):
            if self.closed:
                return
            try:
                result = value.result()
            except Exception as error:
                self.signals.failure.emit(name, error)
            else:
                self.signals.result.emit(name, result)
        future.add_done_callback(done)

    def browse(self):
        path = QFileDialog.getExistingDirectory(self, "動画の保存先", self.directory.text())
        if path:
            self.directory.setText(path)

    def connect_camera(self):
        if self.action_busy or self.batch_running:
            return
        self.action_busy = True
        self.connect_button.setEnabled(False)
        destination = Path(self.directory.text()).resolve()
        if self.client is None or self.client.save_dir != destination:
            self.client = PixelCameraClient(save_dir=destination)
        self.connection_label.setText("PCサーバーを起動し、USBを設定しています…")
        self.submit("connect", lambda: self.client.connect(wait_for_device=3))

    def disconnect_camera(self):
        if self.batch_running:
            return
        self.server_ready = False
        self.batch_button.setEnabled(False)
        if self.client:
            self.client.disconnect()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.connection_label.setText("操作クライアントを切断しました。録画と転送はPCサーバーが継続します。")

    def shutdown_server(self):
        if self.batch_running:
            return
        if self.client:
            self.submit("shutdown", self.client.shutdown_server)

    def start_recording(self):
        if self.action_busy or self.batch_running or not self.client:
            return
        self.action_busy = True
        self.start_button.setEnabled(False)
        seconds = self.duration.value()
        def start():
            self.client.configure()
            return self.client.start_recording(duration_s=seconds)
        self.submit("start", start)

    def stop_recording(self):
        if self.client and not self.batch_running:
            self.stop_button.setEnabled(False)
            self.submit("stop", self.client.stop_recording)

    def start_batch(self):
        if self.action_busy or self.batch_running or not self.client:
            return
        self.batch_running = True
        self.action_busy = True
        self.cancel_batch.clear()
        self.batch_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.start_button.setEnabled(False)
        for button in (self.connect_button, self.disconnect_button, self.shutdown_button):
            button.setEnabled(False)
        count, seconds, client = self.repetitions.value(), self.duration.value(), self.client
        self.submit("stability", lambda: run_stability(client, repetitions=count, duration_s=seconds,
                    cancel=self.cancel_batch, on_progress=self.signals.progress.emit))

    def open_report(self):
        if self.last_report:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.last_report)))

    def poll(self):
        if not self.server_ready or self.polling or not self.client:
            return
        self.polling = True
        client = self.client
        self.submit("poll", lambda: (client.get_status(), client.history()))

    def finished(self, name, value):
        if name == "poll":
            self.polling = False
            status, history = value
            self.display_status(status)
            self.display_history(history)
            return
        if name == "stability":
            self.batch_running = False
            self.cancel_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.shutdown_button.setEnabled(True)
            self.last_report, report = value
            self.report_button.setEnabled(True)
            self.log.appendPlainText(f"連続試験: {report['state']} / 合格 {report['passed_runs']} 回")
            self.log.appendPlainText("レポート: " + str(self.last_report))
        self.action_busy = False
        self.connect_button.setEnabled(not self.batch_running)
        if name == "connect":
            self.server_ready = True
            self.connection_label.setText("スマホに接続しました")
            self.submit("capabilities", self.client.get_capabilities)
        elif name == "start":
            self.log.appendPlainText("録画開始を受け付けました: " + value)
        elif name == "stop":
            self.log.appendPlainText("停止要求を受け付けました。MP4確定と転送を待っています。")
        elif name == "shutdown":
            self.server_ready = False
            self.connection_label.setText("PCサーバーを停止しました")
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)
        elif name == "capabilities":
            self.log.appendPlainText("端末: " + str(value.get("device", {}).get("model")))

    def failed(self, name, error):
        if name == "stability":
            self.batch_running = False
            self.cancel_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.shutdown_button.setEnabled(True)
        if name == "poll":
            self.polling = False
        else:
            self.action_busy = False
            self.connect_button.setEnabled(True)
        self.log.appendPlainText(f"{datetime.now():%H:%M:%S} {name}: {error}")
        if name == "connect":
            self.server_ready = getattr(error, "code", "") == "PHONE_NOT_CONNECTED"
            self.connection_label.setText(str(error))
        if name == "poll":
            self.connection_label.setText("PCサーバーとの通信を確認してください")
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)

    def display_status(self, status):
        connected = status.get("connected", False)
        value = status.get("device", {})
        phase = value.get("phase", "IDLE")
        own = value.get("owner", "") in ("", self.client.client_id)
        labels = {"IDLE": "待機中", "PREPARING": "録画準備中", "RECORDING": "録画中",
                  "STOPPING": "停止要求済み", "FINALIZING": "MP4確定中", "TRANSFERRING": "転送中", "ERROR": "録画エラー"}
        self.connection_label.setText("スマホ接続済み" if connected else "スマホ未接続 — アプリのPC接続とUSBを確認してください")
        self.status_label.setText(labels.get(phase, phase) + ("（別クライアントが操作中）" if not own else "") + "  " + value.get("message", ""))
        self.status_label.setWordWrap(True)
        self.elapsed_label.setText(f"{value.get('elapsed_seconds', 0):.2f} 秒")
        self.space_label.setText(f"{value.get('available_bytes', 0) / 1024**3:.2f} GiB")
        self.start_button.setEnabled(connected and phase in {"IDLE", "ERROR"} and not self.action_busy and not self.batch_running)
        self.batch_button.setEnabled(connected and phase == "IDLE" and not self.action_busy and not self.batch_running)
        self.repetitions.setEnabled(not self.batch_running)
        self.stop_button.setEnabled(connected and own and not self.batch_running and phase in {"PREPARING", "RECORDING", "STOPPING"})
        self.duration.setEnabled(not self.batch_running and phase in {"IDLE", "ERROR"})
        self.progress.setValue(value.get("transfer_percent", 0))
        transfers = status.get("transfers", {})
        if phase in {"PREPARING", "RECORDING", "STOPPING", "FINALIZING"}:
            self.progress.setValue(0)
            self.speed.setText("録画の確定後に自動転送します")
        elif transfers:
            current = list(transfers.values())[-1]
            total = current.get("total", 0)
            if total:
                self.progress.setValue(min(100, int(current.get("received", 0) * 100 / total)))
            state_text = {"RECEIVING": "転送中", "VERIFIED": "照合完了", "FAILED_RETAIN_SOURCE": "転送失敗・元動画保持"}.get(current.get("state"), "転送待ち")
            self.speed.setText(f"{state_text} / {current.get('bytes_per_second', 0) / 1024**2:.1f} MiB/s")

    def display_history(self, history):
        if history == self.history_values:
            return
        previous = {item["recording_id"]: item for item in self.history_values}
        self.history_values = history
        self.review_button.setEnabled(bool(history))
        self.history_table.setRowCount(len(history))
        for row, item in enumerate(history):
            values = [datetime.fromtimestamp(item["received_at"]).strftime("%H:%M:%S"),
                      str(item.get("frame_count", "—")), f"{item['size'] / 1024**2:.1f}",
                      "削除済み" if item.get("source_deleted") else "確認待ち", Path(item["file_path"]).name]
            for column, text in enumerate(values):
                self.history_table.setItem(row, column, QTableWidgetItem(text))
            if item["recording_id"] not in previous:
                self.log.appendPlainText("PC保存・SHA-256照合完了: " + item["file_path"])
                QApplication.beep()

    def open_review(self):
        row = self.history_table.currentRow()
        if not 0 <= row < len(self.history_values):
            self.log.appendPlainText("履歴で検査する動画を選んでください")
            return
        path = self.history_values[row]["file_path"]
        try:
            subprocess.Popen([sys.executable, "-m", "pixel_camera.review", path],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        except OSError as error:
            self.log.appendPlainText("コマ送り検査を起動できません: " + str(error))

    def open_recording(self, row, column):
        if 0 <= row < len(self.history_values):
            QDesktopServices.openUrl(QUrl.fromLocalFile(self.history_values[row]["file_path"]))

    def closeEvent(self, event):
        self.cancel_batch.set()
        self.closed = True
        self.timer.stop()
        self.executor.shutdown(wait=False, cancel_futures=True)
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = CameraWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
