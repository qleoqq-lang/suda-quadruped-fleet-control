import logging
import os
from pathlib import Path
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "windows")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from dual_dog.ui.gui import MainWindow, OPENING_CEREMONY_STYLE_SHEET


def main() -> None:
    app = QApplication.instance() or QApplication([])
    app.setStyle("Fusion")
    app.setStyleSheet(OPENING_CEREMONY_STYLE_SHEET)
    window = MainWindow(ROOT / "config" / "robots.json")
    window.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen)
    window.show()
    outputs = (
        ("suda_opening_theme_preview.png", (1280, 920)),
        ("suda_opening_theme_minimum.png", (1100, 850)),
    )
    for filename, size in outputs:
        window.resize(*size)
        for _ in range(4):
            app.processEvents()
        output = ROOT / "_local" / "diagnostics" / filename
        output.parent.mkdir(parents=True, exist_ok=True)
        if not window.grab().save(str(output)):
            raise RuntimeError(f"failed to save {output}")
        print(output)
    window.refresh_timer.stop()
    window.music_player.stop()
    logging.getLogger().removeHandler(window.log_handler)
    window.deleteLater()
    app.processEvents()


if __name__ == "__main__":
    main()