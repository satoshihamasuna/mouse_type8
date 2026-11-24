"""既存のターン調整GUIをBLE・録画選択・本体ログ画像付きで起動する。"""
from pathlib import Path
import os
import subprocess
import sys


def configure_tuner(gui):
    gui.title('TYPE8 ターン調整 — BLE・ログ・録画')
    gui.port.set('BLE')
    gui.connect_button.configure(command=gui.camera_panel.toggle_connection)
    gui.geometry(f'{min(1250, gui.winfo_screenwidth()-40)}x{min(1000, gui.winfo_screenheight()-100)}+20+20')
    gui.motion.set('turn')
    gui.turn_type.set('long_r180')
    gui.turn_preset_speed.set(500)
    for variable in gui.values.values():
        variable.set('')
    gui._update_motion_fields()
    gui.camera_panel.enable_workflow()
    gui.camera_panel.status.set('機体を選び「① BLE接続」または上部の「Connect」から開始してください')
    for parent in gui.winfo_children():
        _configure_actions(parent, gui.camera_panel)


def _configure_actions(parent, panel):
    from tkinter import ttk
    for widget in parent.winfo_children():
        if isinstance(widget, ttk.Button):
            label = widget.cget('text')
            if label == 'Apply':
                widget.configure(command=panel.apply_settings)
            elif label in ('Apply and execute', 'Get log CSV', 'ログ初期化'):
                widget.pack_forget()
        _configure_actions(widget, panel)


def main():
    from pixel_camera_environment import initialize_camera_environment
    initialize_camera_environment()
    # 既存の録画用環境があれば再利用する。未導入でも録画OFFで起動できる。
    from pixel_camera_capture import camera_workspace, camera_python
    try:
        python = camera_python(camera_workspace())
    except RuntimeError:
        python = Path(sys.executable)
    if python.resolve() != Path(sys.executable).resolve():
        return subprocess.call([str(python), str(Path(__file__).resolve()), *sys.argv[1:]],
            env=dict(os.environ, PYTHONUTF8='1'))
    from ble.mouse_gui_ble import install_ble_port
    install_ble_port(8766)
    from myshell_debug import DebugGui
    gui = DebugGui()
    configure_tuner(gui)
    gui.mainloop()
    return 0


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    for name in ('ble_python', f'ble_python{sys.version_info.major}{sys.version_info.minor}'):
        sys.path.insert(0, str(root / '.tools' / name))
    raise SystemExit(main())
