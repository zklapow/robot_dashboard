from robot_monitor import RobotMonitor

from QtCore import pyqtSignal
from QtGui import QPushButton

class MonitorDashWidget(QPushButton):
    def __init__(self, context):
        super(MonitorDashWidget, self).__init__()
        self.setObjectName("Robot Monitor")

        self._monitor = None
        self._context = context

        self.clicked.connect(self._show_monitor)

    def _show_monitor(self):
        if not self._monitor:
            self._monitor = RobotMonitor('diagnostics_agg')
            self._monitor.destroyed.connect(self._monitor_close)

        self._context.add_widget(self._monitor)

    def _monitor_close(self):
        self._monitor.close()
        self._monitor = None
