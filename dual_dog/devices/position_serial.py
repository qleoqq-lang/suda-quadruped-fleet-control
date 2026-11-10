"""Non-blocking, receive-only Qt serial transport; no robot SDK dependency."""
from __future__ import annotations

from PySide6.QtCore import QIODevice, QObject, Signal
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo

from dual_dog.devices.positioning import TagLineParser


class PositionSerial(QObject):
    samples = Signal(object)
    status = Signal(str)
    connection_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.port = QSerialPort(self)
        self.port.setReadBufferSize(65536)
        self.port.readyRead.connect(self._read)
        self.port.errorOccurred.connect(self._error)
        self.parser = TagLineParser()
        self.received_bytes = 0
        self._opening = False

    @staticmethod
    def available_ports() -> list[tuple[str, str, bool]]:
        return [(p.portName(), p.description(), p.hasVendorIdentifier()) for p in QSerialPortInfo.availablePorts()]

    def start(self, name: str = "") -> bool:
        self.stop()
        if not name:
            usb = [p[0] for p in self.available_ports() if p[2]]
            if len(usb) != 1:
                self.status.emit("未找到唯一 USB 串口，请刷新并手动选择。")
                return False
            name = usb[0]
        self.parser = TagLineParser()
        self.received_bytes = 0
        self.port.setPortName(name)
        self.port.setBaudRate(115200)
        self.port.setDataBits(QSerialPort.Data8)
        self.port.setParity(QSerialPort.NoParity)
        self.port.setStopBits(QSerialPort.OneStop)
        self.port.setFlowControl(QSerialPort.NoFlowControl)
        self._opening = True
        opened = self.port.open(QIODevice.ReadOnly)
        self._opening = False
        if not opened:
            self.status.emit(f"无法打开 {name}：{self.port.errorString()}（请检查占用/接线）")
            return False
        self.port.setDataTerminalReady(False)
        self.port.setRequestToSend(False)
        self.status.emit(f"已连接 {name} · 115200 / 8N1 · 只接收，等待 TAG1～TAG10")
        self.connection_changed.emit(True)
        return True

    def stop(self) -> None:
        if self.port.isOpen():
            self.port.close()
        self.connection_changed.emit(False)

    def _read(self) -> None:
        data = bytes(self.port.readAll())
        self.received_bytes += len(data)
        samples = self.parser.feed(data)
        if samples:
            self.samples.emit(samples)

    def _error(self, error) -> None:
        if self._opening or error == QSerialPort.NoError:
            return
        if error in (QSerialPort.ResourceError, QSerialPort.DeviceNotFoundError,
                     QSerialPort.PermissionError, QSerialPort.ReadError):
            message = self.port.errorString()
            self.stop()
            self.status.emit(f"串口已断开：{message}；重新插入后刷新端口并连接。")
