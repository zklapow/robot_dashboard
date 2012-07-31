from robot_monitor import RobotMonitor
from QtGui import QPushButton

class RobotMonitorWidget(QPushButton):
    def __init__(self):
        super(RobotMonitorWidget, self).__init__()

        self.monitor = None

        self.clicked.connect(self.show_monitor)

    def show_monitor(self):
        if not self.monitor:
            self.monitor = RobotMonitor('diagnostics_agg')

        self.monitor.show()
