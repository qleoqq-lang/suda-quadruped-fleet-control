"""Network survey and channel maintenance inside the fleet console."""

import json
import logging
import time

from PySide6.QtCore import QThread, QTimer, Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QGridLayout, QGroupBox,
    QHeaderView, QLabel, QLineEdit, QPlainTextEdit, QPushButton,
    QScrollArea, QSizePolicy, QTableWidget, QTableWidgetItem, QVBoxLayout,
    QWidget,
)

from dual_dog.devices.network_guard import NetworkLease
from dual_dog.devices.wifi_optimizer import (
    OUTPUT, Router, change_and_verify, fresh_scan, rank_channels,
    worth_switching, write_json,
)


class NetworkJob(QThread):
    result_ready = Signal(object)
    failed = Signal(str)
    message = Signal(str)

    def __init__(self, password, ips, candidate=None, allow_offline=False, parent=None):
        super().__init__(parent)
        self.password = password
        self.ips = ips
        self.candidate = candidate
        self.allow_offline = allow_offline

    def run(self):
        owner = self

        class JobLog(logging.Handler):
            def emit(self, record):
                owner.message.emit(record.getMessage())

        logger = logging.getLogger("dual_dog.devices.wifi_optimizer")
        handler = JobLog()
        logger.addHandler(handler)
        try:
            router = Router(self.password)
            snapshot = router.wifi()
            radio = snapshot["info"][1]
            allowed = {int(item["c"]) for item in radio["available_channels"] if "40" in map(str, item["b"])}
            if self.candidate is not None:
                if self.candidate not in allowed:
                    raise RuntimeError("路由器当前不支持该信道的40MHz频宽")
                change_and_verify(
                    router,
                    snapshot,
                    self.candidate,
                    self.ips,
                    require_all_reachable=not self.allow_offline,
                )
                radio = router.wifi()["info"][1]
                self.result_ready.emit({"changed": True, "channel": radio["channel"], "bandwidth": radio["bandwidth"]})
                return
            self.message.emit("正在主动扫描现场Wi-Fi…")
            networks = fresh_scan()
            if not any(row["ssid"] == radio["ssid"] for row in networks):
                raise RuntimeError("扫描未发现目标路由器，无法提供可靠推荐")
            report = {
                "time": time.time(), "current_channel": int(radio["channelInfo"]["channel"]),
                "bandwidth": radio["bandwidth"], "ssid": radio["ssid"],
                "ranking": rank_channels(networks, radio["ssid"], allowed), "networks": networks,
            }
            write_json(OUTPUT / "latest_scan.json", report)
            self.result_ready.emit(report)
        except Exception as error:
            self.failed.emit(str(error).split(";stok=")[0])
        finally:
            self.password = ""
            logger.removeHandler(handler)


class NetworkPanel(QWidget):
    busy_changed = Signal(bool)

    def __init__(self, configs, maintenance_allowed, parent=None):
        super().__init__(parent)
        self.configs = configs
        self.maintenance_allowed = maintenance_allowed
        self.worker = None
        self.lease = None
        self.pending_candidate = None
        self.candidate = None
        self.repetitions = 0
        self.last_change = 0.0
        self.manual_select_requested = False
        self.last_report = None
        self.busy = False
        self._last_wait_reason = None
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        content = QWidget()
        content.setMinimumWidth(980)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        scroll.setWidget(content)
        outer.addWidget(scroll)

        settings = QGroupBox("现场 Wi-Fi 检测与自动选择")
        controls = QGridLayout(settings)
        controls.setHorizontalSpacing(12)
        controls.setVerticalSpacing(8)
        controls.setColumnStretch(1, 1)
        controls.setColumnStretch(3, 1)
        notice = QLabel("先断开设备并停止控制任务，再检测或优化。切换期间会短暂断网；检测工具不发送机器人动作。")
        notice.setWordWrap(True)
        controls.addWidget(notice, 0, 0, 1, 4)
        controls.addWidget(QLabel("小米路由器 192.168.31.1"), 1, 0)
        self.password = QLineEdit()
        self.password.setEchoMode(QLineEdit.Password)
        self.password.setPlaceholderText("路由器管理密码（仅本次使用）")
        self.password.setMinimumWidth(280)
        controls.addWidget(self.password, 1, 1, 1, 2)
        self.load_button = QPushButton("读取上次结果")
        self.load_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        controls.addWidget(self.load_button, 1, 3)

        self.select_button = QPushButton("自动选择空闲信道")
        self.select_button.setProperty("role", "primary")
        self.select_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        controls.addWidget(self.select_button, 2, 0, 1, 2)
        self.scan_button = QPushButton("仅检测")
        self.scan_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        controls.addWidget(self.scan_button, 2, 2)
        self.channel_choice = QComboBox()
        for channel in (36, 44, 149, 157):
            self.channel_choice.addItem(str(channel), channel)
        controls.addWidget(QLabel("固定 5GHz 信道"), 3, 0)
        controls.addWidget(self.channel_choice, 3, 1)
        self.pin_button = QPushButton("固定到 40 MHz")
        self.pin_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        controls.addWidget(self.pin_button, 3, 2, 1, 2)
        self.auto = QCheckBox("持续自动选择（每60秒检测，连续3次推荐一致才切换）")
        controls.addWidget(self.auto, 4, 0, 1, 4)
        self.status = QLabel("未开始检测")
        self.status.setWordWrap(True)
        controls.addWidget(self.status, 5, 0, 1, 4)
        layout.addWidget(settings)

        self.ranking = QTableWidget(0, 4)
        self.ranking.setHorizontalHeaderLabels(["候选信道", "频宽", "干扰估算（越低越好）", "同频范围AP数量"])
        self.ranking.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.ranking.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.ranking.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.ranking.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.ranking.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.ranking.verticalHeader().setVisible(False)
        self.ranking.setMinimumHeight(145)
        self.ranking.setMaximumHeight(180)
        layout.addWidget(self.ranking)
        self.networks = QTableWidget(0, 5)
        self.networks.setHorizontalHeaderLabels(["Wi-Fi 名称", "BSSID", "频段", "信道", "接收信号"])
        self.networks.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        for column in range(1, 5):
            self.networks.horizontalHeader().setSectionResizeMode(column, QHeaderView.ResizeToContents)
        self.networks.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.networks.verticalHeader().setVisible(False)
        self.networks.setMinimumHeight(230)
        self.networks.setTextElideMode(Qt.ElideRight)
        layout.addWidget(self.networks, 1)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setMinimumHeight(100)
        self.log.setMaximumHeight(150)
        layout.addWidget(self.log)
        details = QLabel("仅评估路由器支持的非DFS信道，固定40MHz；改善不足30%不切换，切换间隔至少10分钟。信号评分不等于实际空口占用率。")
        details.setWordWrap(True)
        layout.addWidget(details)
        self.timer = QTimer(self)
        self.timer.setInterval(60000)
        self.timer.timeout.connect(self.auto_tick)
        self.select_button.clicked.connect(self.auto_select)
        self.pin_button.clicked.connect(self.pin_channel)
        self.scan_button.clicked.connect(self.scan)
        self.load_button.clicked.connect(self.load_saved)
        self.auto.toggled.connect(self.toggle_auto)
        try:
            self.last_change = float(json.loads((OUTPUT / "cooldown.json").read_text(encoding="utf-8"))["time"])
        except (OSError, ValueError, KeyError):
            pass
        self.load_saved()

    def append(self, message):
        self.log.appendPlainText(time.strftime("%H:%M:%S ") + message)

    def load_saved(self):
        try:
            report = json.loads((OUTPUT / "latest_scan.json").read_text(encoding="utf-8"))
            self.display_report(report, saved=True)
        except (OSError, ValueError, KeyError, TypeError):
            self.status.setText("暂无历史检测结果")

    def display_report(self, report, saved=False):
        self.last_report = report
        ranking = report["ranking"]
        self.ranking.setRowCount(len(ranking))
        for row, entry in enumerate(ranking):
            values = [entry["channel"], "40 MHz", entry["score"], entry["neighbors"]]
            for column, value in enumerate(values):
                self.ranking.setItem(row, column, QTableWidgetItem(str(value)))
        networks = sorted(report["networks"], key=lambda item: item["signal_percent"], reverse=True)
        self.networks.setRowCount(len(networks))
        for row, entry in enumerate(networks):
            values = [entry["ssid"] or "隐藏SSID", entry["bssid"], "5GHz" if entry["channel"] > 14 else "2.4GHz", entry["channel"], f"{entry['signal_percent']}%"]
            for column, value in enumerate(values):
                self.networks.setItem(row, column, QTableWidgetItem(str(value)))
        stamp = time.strftime("%H:%M:%S", time.localtime(report["time"]))
        preferred = ranking[0]["channel"] if ranking else "无"
        source = "历史结果" if saved else "本次检测"
        self.status.setText(f"{source} {stamp} · 当前信道 {report['current_channel']} · 最低干扰候选 {preferred} · 检测到 {len(networks)} 个AP")

    def allowed(self):
        reason = self.maintenance_allowed()
        if reason:
            self.candidate, self.repetitions = None, 0
            self.status.setText("等待网络维护：" + reason)
            if self._last_wait_reason != reason:
                self.append(reason)
                self._last_wait_reason = reason
            return False
        self._last_wait_reason = None
        return True

    def launch(self, candidate=None, allow_offline=False):
        if self.busy or not self.allowed():
            return False
        if not self.password.text():
            self.status.setText("请输入路由器管理密码")
            return False
        self.lease = NetworkLease("Local\\BPX_Wifi_Optimizer")
        if not self.lease.acquire():
            self.status.setText("独立信道工具正在运行，请先停止它")
            self.lease = None
            return False
        self.busy = True
        self.select_button.setEnabled(False)
        self.pin_button.setEnabled(False)
        self.channel_choice.setEnabled(False)
        self.scan_button.setEnabled(False)
        self.password.setEnabled(False)
        self.busy_changed.emit(True)
        self.worker = NetworkJob(
            self.password.text(),
            [config.ip for config in self.configs[:10]],
            candidate,
            allow_offline,
            self,
        )
        self.worker.message.connect(self.append)
        self.worker.result_ready.connect(self.result)
        self.worker.failed.connect(self.failed)
        self.worker.finished.connect(self.finished)
        if candidate is not None:
            self.last_change = time.time()
            write_json(OUTPUT / "cooldown.json", {"time": self.last_change})
            self.status.setText(f"正在切换到信道 {candidate} 并验证网络恢复…")
        else:
            self.status.setText("正在检测，请稍候…")
        self.worker.start()
        return True

    def scan(self):
        self.launch()

    def auto_select(self):
        if self.busy or not self.allowed():
            return False
        if not self.password.text():
            self.status.setText("请输入路由器管理密码")
            return False
        self.pending_candidate = None
        self.manual_select_requested = True
        if not self.launch():
            self.manual_select_requested = False
            return False
        self.status.setText("正在扫描并自动选择空闲信道…")
        return True

    def pin_channel(self):
        if self.busy or not self.allowed():
            return False
        if not self.password.text():
            self.status.setText("请输入路由器管理密码")
            return False
        channel = int(self.channel_choice.currentData())
        self.pending_candidate = None
        self.manual_select_requested = False
        if not self.launch(channel, allow_offline=True):
            return False
        self.status.setText(f"正在固定到信道 {channel} / 40 MHz 并验证网络恢复…")
        return True

    def apply_manual_selection(self, report):
        ranking = report["ranking"]
        if not ranking:
            self.status.setText("未找到可用信道")
            self.manual_select_requested = False
            return
        best = ranking[0]
        current = int(report["current_channel"])
        if not worth_switching(current, ranking):
            if best["channel"] == current:
                self.status.setText(f"当前信道 {current} 已是最优空闲信道，无需切换")
            else:
                self.status.setText(f"当前信道无需切换；候选信道 {best['channel']} 的改善不足")
            self.append(f"自动选择完成：当前 {current}，最佳候选 {best['channel']}，不切换")
            self.manual_select_requested = False
            return
        remaining = int(max(0, 600 - (time.time() - self.last_change)))
        if remaining:
            self.status.setText(f"发现更优信道 {best['channel']}，但切换冷却还剩 {remaining} 秒")
            self.append(f"自动选择暂停：切换冷却剩余 {remaining} 秒")
            self.manual_select_requested = False
            return
        quality = "空闲信道" if best["neighbors"] == 0 else f"最低干扰信道（重叠AP {best['neighbors']} 个）"
        self.pending_candidate = best["channel"]
        self.status.setText(f"发现{quality} {best['channel']}，准备从 {current} 切换并验证…")
        self.append(f"自动选择：从信道 {current} 切换到 {quality} {best['channel']}")

    def toggle_auto(self, enabled):
        self.pending_candidate = None
        self.candidate, self.repetitions = None, 0
        if enabled:
            if not self.password.text():
                self.auto.setChecked(False)
                self.status.setText("启用持续自动选择前请输入路由器管理密码")
                return
            self.timer.start()
            self.auto_tick()
        else:
            self.timer.stop()
            self.append("持续自动选择已停止；已开始的网络恢复验证会继续完成")

    def auto_tick(self):
        if self.auto.isChecked() and not self.busy:
            self.launch()

    def result(self, report):
        if report.get("changed"):
            self.status.setText(f"网络验证完成 · 当前信道 {report['channel']} · {report['bandwidth']}MHz")
            self.candidate, self.repetitions = None, 0
            self.manual_select_requested = False
            return
        self.display_report(report)
        if self.manual_select_requested:
            self.apply_manual_selection(report)
            return
        if not self.auto.isChecked():
            return
        ranking = report["ranking"]
        best = ranking[0]["channel"] if ranking else None
        self.repetitions = self.repetitions + 1 if best == self.candidate else 1
        self.candidate = best
        if not worth_switching(report["current_channel"], ranking):
            self.append("没有明显更优信道，保持当前配置")
        elif self.repetitions < 3 or time.time() - self.last_change < 600:
            self.append(f"一致推荐 {self.repetitions}/3；等待连续结果及冷却结束")
        else:
            self.pending_candidate = best

    def failed(self, message):
        self.pending_candidate = None
        self.candidate, self.repetitions = None, 0
        self.manual_select_requested = False
        self.status.setText("检测/优化失败：" + message)
        self.append(message)

    def finished(self):
        self.worker.deleteLater()
        self.worker = None
        if self.lease:
            self.lease.close()
            self.lease = None
        self.busy = False
        self.select_button.setEnabled(True)
        self.pin_button.setEnabled(True)
        self.channel_choice.setEnabled(True)
        self.scan_button.setEnabled(True)
        self.password.setEnabled(True)
        self.busy_changed.emit(False)
        candidate = self.pending_candidate
        manual_select = self.manual_select_requested
        self.pending_candidate = None
        self.manual_select_requested = False
        if candidate is not None and (self.auto.isChecked() or manual_select):
            QTimer.singleShot(0, lambda channel=candidate: self.launch(channel))
