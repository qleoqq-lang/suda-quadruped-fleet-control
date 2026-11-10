"""Suqian Chu-Han inspired vector decoration for the football console."""

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QLinearGradient, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget


def silhouette(painter, points, color):
    path = QPainterPath(QPointF(*points[0]))
    for point in points[1:]:
        path.lineTo(*point)
    path.closeSubpath()
    painter.fillPath(path, QColor(color))


def draw_hero(painter, x, y, lady=False):
    """Stylized theatrical figures, rather than historical portrait likenesses."""
    painter.save()
    painter.translate(x, y)
    gold = "#d4b579"
    painter.setPen(QPen(QColor("#8f855d"), 0.8))
    painter.setBrush(QColor("#234f48"))
    painter.drawEllipse(QRectF(18, 1, 65, 65))
    painter.setPen(Qt.NoPen)
    painter.setBrush(QColor(gold))
    painter.drawEllipse(QRectF(44, 14, 12, 14))
    if lady:
        painter.drawEllipse(QRectF(46, 7, 9, 10))
        silhouette(painter, [(43,15),(38,11),(34,15),(43,18),(56,17),(66,11),(59,10)], gold)
        silhouette(painter, [(43,28),(55,28),(60,39),(57,46),(71,73),(29,73),(42,45),(38,38)], gold)
        silhouette(painter, [(42,29),(34,34),(18,24),(14,28),(31,47),(43,40)], gold)
        silhouette(painter, [(54,29),(64,36),(79,25),(84,27),(71,46),(57,41)], gold)
        painter.setPen(QPen(QColor(gold), 2))
        painter.setBrush(Qt.NoBrush)
        ribbon = QPainterPath(QPointF(15,26))
        ribbon.cubicTo(-2, 8, 23, 2, 28, 12)
        ribbon.cubicTo(32, 20, 5, 42, 12, 57)
        painter.drawPath(ribbon)
        ribbon = QPainterPath(QPointF(81,27))
        ribbon.cubicTo(101, 9, 79, 4, 75, 15)
        ribbon.cubicTo(72, 30, 103, 42, 88, 64)
        painter.drawPath(ribbon)
        painter.setPen(QPen(QColor("#234f48"), 1))
        painter.drawLine(48,46,40,69)
        painter.drawLine(52,46,60,69)
    else:
        silhouette(painter, [(40,16),(43,7),(57,7),(61,17)], gold)
        painter.setBrush(Qt.NoBrush)
        plume = QPainterPath(QPointF(49,9))
        plume.cubicTo(24,-9,15,11,25,22)
        painter.setPen(QPen(QColor(gold), 2))
        painter.drawPath(plume)
        silhouette(painter, [(40,29),(29,34),(33,49),(39,43),(36,60),(29,74),(45,74),(49,59),(54,74),(70,74),(62,57),(62,36),(56,29)], gold)
        silhouette(painter, [(37,30),(18,38),(30,65),(40,56)], "#a99362")
        silhouette(painter, [(58,30),(68,38),(81,31),(84,36),(68,46),(59,40)], gold)
        painter.setPen(QPen(QColor(gold), 2))
        painter.drawLine(83,12,83,76)
        silhouette(painter, [(83,3),(78,17),(83,14),(88,17)], gold)
        painter.setPen(QPen(QColor("#234f48"), 1))
        for row in (37,43,49):
            painter.drawLine(41,row,57,row)
        painter.drawLine(49,32,49,53)
    painter.restore()


class FootballHeader(QWidget):
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        bounds = QRectF(self.rect()).adjusted(1,1,-1,-1)
        gradient = QLinearGradient(0,0,self.width(),self.height())
        gradient.setColorAt(0,QColor("#102f30"))
        gradient.setColorAt(0.5,QColor("#20584b"))
        gradient.setColorAt(1,QColor("#102f30"))
        painter.setPen(QPen(QColor("#ad945d"),1))
        painter.setBrush(gradient)
        painter.drawRoundedRect(bounds,16,16)
        # Side medallions stay outside the centered title and above controls.
        draw_hero(painter, 30, 8)
        draw_hero(painter, self.width()-130, 8, lady=True)
        painter.setPen(QColor("#e3cf9d"))
        painter.setFont(QFont("Microsoft YaHei UI", 9))
        painter.drawText(QRectF(24,84,115,18),Qt.AlignCenter,"项羽 · 霸王雄风")
        painter.drawText(QRectF(self.width()-139,84,115,18),Qt.AlignCenter,"虞姬 · 楚韵芳华")
        # Abstract gate roofs and water lines evoke the city without claiming
        # to reproduce a specific monument or an official event emblem.
        painter.setPen(QPen(QColor("#547568"),0.8))
        for x in (155,self.width()-211):
            painter.drawLine(x,74,x+56,74)
            painter.drawLine(x+9,68,x+47,68)
            painter.drawLine(x+13,64,x+43,64)
            painter.drawLine(x+13,64,x+6,69)
            painter.drawLine(x+43,64,x+50,69)
            for col in (17,28,39):
                painter.drawLine(x+col,69,x+col,74)
            for row in (81,86,91):
                painter.drawLine(x-4,row,x+60,row)
        # Restrained Chu-style angular border below the cultural motifs.
        painter.setPen(QPen(QColor("#8f855d"),0.8))
        for start in (26,self.width()-134):
            for i in range(6):
                x=start+i*18
                painter.drawPolyline([QPointF(x,108),QPointF(x,104),QPointF(x+11,104),QPointF(x+11,108)])
