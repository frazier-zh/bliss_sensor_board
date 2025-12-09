"""
    Project: BLISS
    Application: BLISS Sensor Board User Interface
    File: src/plot.py
    Description: Plot class based on PyQtGraph.
    Author: Fang Zihang (Dr.)
    Email: zh.fang@nus.edu.sg
    Affiliation: National University of Singapore
"""
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPainterPath
import pyqtgraph as pg
import numpy

class Marker(pg.TargetItem):
    def __init__(self, **kwargs):
        path = QPainterPath()
        # Symbol:
        #  +---+
        #  |   |
        #   \ /
        #    +  (0, 0)
        path.moveTo(0, 0)
        path.lineTo(-0.4, -0.4)
        path.lineTo(-0.4, -1)
        path.lineTo(0.4, -1)
        path.lineTo(0.4, -0.4)
        path.closeSubpath()
        super().__init__(symbol=path, **kwargs)
        self.recycle = False

    def mouseClickEvent(self, ev):
        if not ev.button() == Qt.MouseButton.RightButton:
            return
        ev.accept()
        self.recycle = True

    def hoverEvent(self, ev):
        if (not ev.isExit()) and ev.acceptDrags(Qt.MouseButton.LeftButton):
            self.setMouseHover(True)
        else:
            self.setMouseHover(False)

class PlotViewBox(pg.ViewBox):
    def wheelEvent(self, ev, axis=None):
        if axis in (0, 1):
            mask = [False, False]
            mask[axis] = self.state['mouseEnabled'][axis]
        else:
            mask = self.state['mouseEnabled'][:]

        if not any(mask):
            # if mouse zoom/pan is not enabled, ignore the event
            ev.ignore()
            return

        s = 1.02 ** (ev.delta() * self.state['wheelScaleFactor']) # actual scaling factor
        s = [(None if m is False else s) for m in mask]
        
        self._resetTarget()
        self.scaleBy(s)
        ev.accept()
        self.sigRangeChangedManually.emit(mask)

class Plot(pg.PlotWidget):
    marker_changed = pyqtSignal(Marker)

    def __init__(self,
                color="r",
                color2="w",
                linewidth=1,
                ):
        self.vb = PlotViewBox()
        super().__init__(viewBox=self.vb)
        self.vb.setMouseEnabled(x=False, y=True)
        self.vb.setMenuEnabled(False)
        self.vb.setDefaultPadding(0.0)
        self.vb.disableAutoRange()
        self.hideButtons()

        self.data = self.plot([], [], pen=pg.mkPen(color, width=linewidth))

        self.arrow = pg.ArrowItem(angle=180, pen=pg.mkPen(color2, width=1), brush=pg.mkBrush(color))
        self.addItem(self.arrow, ignoreBounds=True)

        self.label = pg.LabelItem("", color=color)
        self.label.setFlag(self.arrow.GraphicsItemFlag.ItemIgnoresTransformations)
        self.label.setParentItem(self.plotItem)
        self.label.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-50, 5))

        self.line = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen(color2, width=1))
        self.addItem(self.line, ignoreBounds=True)
        self.line_label = pg.InfLineLabel(self.line, "", position=0.7)

        self.markers = []

    def update(self):
        # handle all the remaining plot item update
        
        # clean up dead markers
        recycle_markers = [marker for marker in self.markers if marker.recycle]
        if not len(recycle_markers) == 0:
            for marker in recycle_markers:
                self.markers.remove(marker)
                self.removeItem(marker)
                self.marker_changed.emit(marker)

    def set_data(self, x, y):
        self.data.setData(x, y)

    def set_last(self, y, label: str):
        x_right = self.vb.viewRange()[0][1]
        self.arrow.setPos(x_right, y)
        self.label.setText(label)

    def set_line(self, x, label: str):
        self.line.setPos(x)
        self.line_label.setText(label)

    def set_region(self, xmin, xmax):
        self.setXRange(xmin, xmax, padding=0)

    def set_axis_style(self, style):
        for axis_name in ["top", "bottom", "right", "left"]:
            if not axis_name in style:
                self.hideAxis(axis_name)
                continue
            self.showAxis(axis_name)
            axis_style = style[axis_name]
            axis = self.getAxis(axis_name)
            if "style" in axis_style:
                axis.setStyle(**axis_style["style"])
            if "width" in axis_style:
                axis.setWidth(axis_style["width"])
            if "height" in axis_style:
                axis.setHeight(axis_style["height"])

    def mouseDoubleClickEvent(self, ev):
        ev.accept()
        pos = self.vb.mapSceneToView(ev.pos())
        new_marker = Marker(pos=pos, size=16, brush="#1ad5ff", movable=False)
        self.addItem(new_marker, ignoreBounds=True)
        self.markers.append(new_marker)
        self.marker_changed.emit(new_marker)

class MasterViewBox(pg.ViewBox):
    scale_changed = pyqtSignal(float)
    mouse_clicked = pyqtSignal(float)

    def mouseClickEvent(self, ev):
        if ev.button() in [Qt.MouseButton.LeftButton, Qt.MouseButton.RightButton]:
            ev.accept()
            mouse_pos = self.mapSceneToView(ev.scenePos())
            self.mouse_clicked.emit(mouse_pos.x())

    def mouseDragEvent(self, ev, axis=None):
        return
    
    def wheelEvent(self, ev, axis=None):
        s = 1.02 ** (ev.delta() * self.state['wheelScaleFactor']) # actual scaling factor
        ev.accept()
        self.scale_changed.emit(s)

class MasterPlot(pg.PlotWidget):
    # Outbound signal, only active when LinearRegionItem is manually adjusted
    region_changed = pyqtSignal(float, float)

    def __init__(self,
                n_channels,
                max_size,
                min_size=5,
                linewidth=1,
                colors=None,
                color2="w",
                ):
        self.vb = MasterViewBox()
        super().__init__(viewBox=self.vb)
        self.vb.setMouseEnabled(x=True, y=False)
        self.vb.setMenuEnabled(False)
        self.vb.enableAutoRange(axis=pg.ViewBox.YAxis)
        self.vb.disableAutoRange(axis=pg.ViewBox.XAxis)
        self.hideButtons()

        self.n_channels = n_channels
        self.min_size = min_size
        self.max_size = max_size
        if colors is None:
            self.colors = ["r"] * self.n_channels
        else:
            self.colors = colors
        self.xmin = 0
        self.xmax = min_size
        self.region_size = min_size
        self.user_input = False

        self.data = [self.plot([], [], pen=pg.mkPen(self.colors[i], width=linewidth)) for i in range(self.n_channels)]
        self.region = pg.LinearRegionItem(
            values=(self.xmin, self.xmax),
            pen=pg.mkPen(color2, width=4),
            swapMode="push",
        )
        self.addItem(self.region, ignoreBounds=True)
        
        self.markers = {}

        self.vb.scale_changed.connect(self.on_viewbox_scale_changed)
        self.vb.mouse_clicked.connect(self.on_viewbox_mouse_clicked)
        self.region.sigRegionChanged.connect(self.on_region_changed)
        self.region.sigRegionChangeFinished.connect(self.on_region_change_finished)
        
    def set_data(self, x, y):
        self.xmax = x[-1] + 1
        self.setXRange(self.xmin, self.xmax, padding=0)
        for i in range(self.n_channels):
            self.data[i].setData(x, y[:, i])

    def try_set_region(self, xmin, xmax):
        if not self.user_input:
            self.set_region(xmin, xmax)

    def set_marker(self, marker):
        if marker.recycle:
            # remove previous markers
            if marker in self.markers:
                master_marker = self.markers[marker]
                self.removeItem(master_marker)
            self.markers.pop(marker)
        else:
            # create new markers
            pos = marker.pos()
            pos[1] = 0
            new_marker = Marker(pos=pos, size=10, brush="#1ad5ff", movable=False)
            self.addItem(new_marker, ignoreBounds=True)
            self.markers[marker] = new_marker

    def update(self):
        pass

    def set_region(self, xmin, xmax):
        self.region.blockSignals(True)
        self.region.setRegion((xmin, xmax))
        self.region.blockSignals(False)

    def get_region(self):
        return self.region.getRegion()

    def set_axis_style(self, style):
        for axis_name in ["top", "bottom", "right", "left"]:
            if not axis_name in style:
                self.hideAxis(axis_name)
                continue
            self.showAxis(axis_name)
            axis_style = style[axis_name]
            axis = self.getAxis(axis_name)
            if "style" in axis_style:
                axis.setStyle(**axis_style["style"])
            if "width" in axis_style:
                axis.setWidth(axis_style["width"])
            if "height" in axis_style:
                axis.setHeight(axis_style["height"])
    
    def on_viewbox_scale_changed(self, scale):
        rgn_min, rgn_max = self.region.getRegion()
        current_size = rgn_max - rgn_min
        new_size = numpy.clip(current_size * scale, a_min=self.min_size, a_max=self.max_size)
        if new_size == current_size:
            return
        self.region_size = new_size
        self.set_region_center(size=new_size)

    def on_viewbox_mouse_clicked(self, mx):
        new_center = numpy.clip(mx, a_min=self.xmin, a_max=self.xmax)
        self.set_region_center(center=new_center)

    def set_region_center(self, center=None, size=None):
        rgn_min, rgn_max = self.region.getRegion()
        if center is None:
            center = (rgn_min + rgn_max) / 2
        if size is None:
            size = rgn_max - rgn_min
        new_min = center - size / 2
        new_max = center + size / 2
        self.set_region(new_min, new_max)
        self.region_changed.emit(new_min, new_max)

    def on_region_changed(self):
        self.user_input = True

    def on_region_change_finished(self):
        self.user_input = False
        
        # Region changes via direct manipulation of LinearRegionItem
        #   is subject for rejection
        # Maintain region scale while making sure
        #   center is within [self.xmin, self.xmax]
        rgn_min, rgn_max = self.region.getRegion()
        center = (rgn_min + rgn_max) / 2
        size = rgn_max - rgn_min
        size = numpy.clip(size, a_min=self.min_size, a_max=self.max_size)
        if center > self.xmax:
            rgn_min = min(rgn_min, self.xmax)
            rgn_max = rgn_min + size
        elif center < self.xmin:
            rgn_max = max(rgn_max, self.xmin)
            rgn_min = rgn_max - size
        else:
            rgn_min = center - size / 2
            rgn_max = center + size / 2
        self.region_size = size
        self.set_region(rgn_min, rgn_max)
        self.region_changed.emit(rgn_min, rgn_max)
    