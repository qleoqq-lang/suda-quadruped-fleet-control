"""Visual theme for the Soochow University freshman opening ceremony console."""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (
    QColor,
    QFont,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPen,
)
from PySide6.QtWidgets import QWidget


SUDA_RED = "#7A0000"
SUDA_RED_BRIGHT = "#A80A10"
SUDA_GOLD = "#D8B56A"
SUDA_GOLD_LIGHT = "#F0D79B"


def _polygon_path(points: list[tuple[float, float]]) -> QPainterPath:
    path = QPainterPath(QPointF(*points[0]))
    for point in points[1:]:
        path.lineTo(*point)
    path.closeSubpath()
    return path


def _fill_polygon(
    painter: QPainter,
    points: list[tuple[float, float]],
    color: QColor | str,
) -> None:
    painter.fillPath(_polygon_path(points), QColor(color))


def _draw_clock_tower(painter: QPainter, x: float, y: float) -> None:
    """Draw an abstract clock tower, not a reproduction of the official emblem."""
    painter.save()
    painter.translate(x, y)
    gold = QColor(SUDA_GOLD_LIGHT)

    painter.setPen(QPen(gold, 1.35))
    painter.setBrush(QColor(255, 248, 232, 22))
    _fill_polygon(painter, [(8, 33), (34, 10), (60, 33)], QColor(255, 248, 232, 22))
    _fill_polygon(painter, [(14, 26), (34, 9), (54, 26)], QColor(216, 181, 106, 34))

    painter.drawRoundedRect(QRectF(17, 35, 34, 60), 2.0, 2.0)
    painter.drawRect(QRectF(22, 29, 24, 7))
    painter.drawEllipse(QRectF(23, 42, 22, 22))
    painter.drawLine(QPointF(34, 44), QPointF(34, 53))
    painter.drawLine(QPointF(34, 53), QPointF(40, 57))
    painter.drawLine(QPointF(28, 72), QPointF(40, 72))
    painter.drawLine(QPointF(24, 80), QPointF(44, 80))
    painter.drawLine(QPointF(20, 88), QPointF(48, 88))
    painter.drawLine(QPointF(17, 95), QPointF(51, 95))

    painter.setFont(QFont("Georgia", 8, QFont.DemiBold))
    painter.setPen(QColor(SUDA_GOLD_LIGHT))
    painter.drawText(QRectF(18, 96, 32, 14), Qt.AlignCenter, "1900")
    painter.restore()


def _draw_ginkgo_leaf(
    painter: QPainter,
    center_x: float,
    center_y: float,
    radius: float,
    rotation: float,
    color: QColor,
) -> None:
    painter.save()
    painter.translate(center_x, center_y)
    painter.rotate(rotation)
    path = QPainterPath(QPointF(0, radius))
    path.cubicTo(
        -radius * 0.62,
        radius * 0.24,
        -radius * 0.88,
        -radius * 0.55,
        -radius * 0.18,
        -radius * 0.72,
    )
    path.cubicTo(
        -radius * 0.05,
        -radius * 1.03,
        radius * 0.05,
        -radius * 1.03,
        radius * 0.18,
        -radius * 0.72,
    )
    path.cubicTo(
        radius * 0.88,
        -radius * 0.55,
        radius * 0.62,
        radius * 0.24,
        0,
        radius,
    )
    path.closeSubpath()
    painter.fillPath(path, color)
    painter.restore()


def _draw_open_book(painter: QPainter, x: float, y: float) -> None:
    """Draw open pages and gingko leaves as a quiet welcome motif."""
    painter.save()
    painter.translate(x, y)

    for index, rotation in enumerate(range(-48, 49, 16)):
        alpha = 32 + (index % 3) * 9
        _draw_ginkgo_leaf(
            painter,
            38 + index * 0.6,
            14,
            8.5,
            rotation,
            QColor(216, 181, 106, alpha),
        )

    painter.setPen(QPen(QColor(SUDA_GOLD_LIGHT), 1.35))
    painter.setBrush(QColor(255, 248, 232, 32))
    left_page = QPainterPath(QPointF(4, 43))
    left_page.cubicTo(17, 38, 29, 39, 40, 47)
    left_page.lineTo(40, 78)
    left_page.cubicTo(28, 70, 16, 70, 4, 75)
    left_page.closeSubpath()
    right_page = QPainterPath(QPointF(76, 43))
    right_page.cubicTo(63, 38, 51, 39, 40, 47)
    right_page.lineTo(40, 78)
    right_page.cubicTo(52, 70, 64, 70, 76, 75)
    right_page.closeSubpath()
    painter.drawPath(left_page)
    painter.drawPath(right_page)
    painter.drawLine(QPointF(40, 47), QPointF(40, 78))

    painter.setPen(QPen(QColor(240, 215, 155, 105), 0.8))
    for offset in (0, 5, 10):
        painter.drawLine(QPointF(11, 52 + offset), QPointF(34, 55 + offset))
        painter.drawLine(QPointF(46, 55 + offset), QPointF(69, 52 + offset))

    painter.setFont(QFont("Georgia", 7, QFont.DemiBold))
    painter.setPen(QColor(SUDA_GOLD_LIGHT))
    painter.drawText(QRectF(11, 82, 58, 14), Qt.AlignCenter, "SUDA")
    painter.restore()


class SudaOpeningHeader(QWidget):
    """Header painted in the red, ivory and gold palette of the ceremony."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(196)

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        bounds = QRectF(self.rect()).adjusted(1, 1, -1, -1)

        gradient = QLinearGradient(0, 0, self.width(), self.height())
        gradient.setColorAt(0.0, QColor("#6C0000"))
        gradient.setColorAt(0.34, QColor(SUDA_RED))
        gradient.setColorAt(0.70, QColor(SUDA_RED_BRIGHT))
        gradient.setColorAt(1.0, QColor("#710008"))
        painter.setPen(QPen(QColor(SUDA_GOLD), 1.2))
        painter.setBrush(gradient)
        painter.drawRoundedRect(bounds, 18, 18)

        inner = bounds.adjusted(4, 4, -4, -4)
        painter.setPen(QPen(QColor(240, 215, 155, 115), 0.8))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(inner, 15, 15)

        painter.save()
        clip = QPainterPath()
        clip.addRoundedRect(bounds, 18, 18)
        painter.setClipPath(clip)
        painter.setPen(QPen(QColor(240, 215, 155, 18), 1))
        spacing = max(36, self.width() // 24)
        for offset in range(-self.height(), self.width() + self.height(), spacing):
            painter.drawLine(
                QPointF(offset, self.height()),
                QPointF(offset + self.height(), 0),
            )
        painter.setBrush(QColor(240, 215, 155, 20))
        painter.setPen(Qt.NoPen)
        for x in range(18, self.width(), 58):
            for y in (18, self.height() - 19):
                painter.drawEllipse(QPointF(x, y), 1.2, 1.2)
        painter.restore()

        painter.save()
        painter.translate(20, 6)
        painter.scale(0.78, 0.78)
        _draw_clock_tower(painter, 0, 0)
        painter.restore()

        painter.save()
        painter.translate(self.width() - 98, 7)
        painter.scale(0.80, 0.80)
        _draw_open_book(painter, 0, 0)
        painter.restore()

        painter.setPen(QPen(QColor(SUDA_GOLD), 0.9))
        y = self.height() - 7
        painter.drawLine(QPointF(22, y), QPointF(self.width() - 22, y))
        center_x = self.width() / 2
        _fill_polygon(
            painter,
            [
                (center_x - 4, y),
                (center_x, y - 4),
                (center_x + 4, y),
                (center_x, y + 4),
            ],
            QColor(SUDA_GOLD),
        )