import rospy
from diagnostic_msgs.msg import DiagnosticArray

import QtGui
from QtGui import QWidget, QVBoxLayout, QTreeWidget, QTextCursor, QTreeWidgetItem

from QtCore import pyqtSignal

class RobotMonitor(QWidget):
    sig_err = pyqtSignal(str)
    sig_warn = pyqtSignal(str)
    sig_clear = pyqtSignal()

    def __init__(self, topic):
        super(RobotMonitor, self).__init__()
        layout = QVBoxLayout()

        self.err = QTreeWidget()
        self.err.setHeaderLabel("Errors")
        self.warn = QTreeWidget()
        self.warn.setHeaderLabel("Warnings")

        self.sig_clear.connect(self.clear)
        self.sig_err.connect(self.disp_err)
        self.sig_warn.connect(self.disp_warn)

        self.comp = QTreeWidget()
        self.comp.setHeaderLabel("All")

        layout.addWidget(self.err)
        layout.addWidget(self.warn)
        layout.addWidget(self.comp)

        self.setLayout(layout)

        self.sub = rospy.Subscriber(topic, DiagnosticArray, self.cb)

    def cb(self, msg):
        self.sig_clear.emit()
        for status in msg.status:
            if status.level == status.WARN:
                txt = "%s : %s"%(status.name, "Warning")
                self.sig_warn.emit(txt)
            elif status.level == status.ERROR:
                txt = "%s : %s"%(status.name, "Error")
                self.sig_err.emit(txt)

    def clear(self):
        self.err.clear()
        self.warn.clear()

    def disp_err(self, msg):
        i = QTreeWidgetItem()
        i.setText(0, msg)
        self.err.addTopLevelItem(i)
        
    def disp_warn(self,msg):
        i = QTreeWidgetItem()
        i.setText(0, msg)
        self.warn.addTopLevelItem(i)
