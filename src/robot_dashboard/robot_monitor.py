import rospy
from diagnostic_msgs.msg import DiagnosticArray

import QtGui
from QtGui import QWidget, QVBoxLayout, QTreeWidget, QTextCursor, QTreeWidgetItem

from QtCore import pyqtSignal

def get_nice_name(status_name):
    return status_name.split('/')[-1]

def remove_parent_name(status_name):
    return ('/'.join(status_name.split('/')[2:])).strip()

def get_parent_name(status_name):
    return ('/'.join(status_name.split('/')[:-1])).strip()

class StatusItem(QTreeWidgetItem):
    def __init__(self, status):
        super(StatusItem, self).__init__()
    
        self.items = []
        self.name = status.name
        self.level = status.level
        
        self.setText(0, '/' + get_nice_name(self.name))

    def get_children(self, msg):
        ret = []

        for k in msg.status:
            if k.name.startswith(self.name):
                if not k.name == self.name:
                    ret.append(k)

        return ret

    def update(self, level, msg):
        self.level = level
        
        children = self.get_children(msg)

        names = [s.name for s in self.items]
        new_items = []
        remove = []
        for i in children:
            name = i.name
            if name in names:
                w = self.items[names.index(name)]
                w.update(i.level, msg)
            elif len(self.strip_child(name).split('/')) <= 2:
                sti = StatusItem(i)
                sti.update(i.level, msg)
                self.items.append(sti)
                new_items.append(sti)
        self.addChildren(new_items)

    def strip_child(self, child):
        return child.replace(self.name, '')

class RobotMonitor(QWidget):
    sig_err = pyqtSignal(str)
    sig_warn = pyqtSignal(str)
    sig_clear = pyqtSignal()

    def __init__(self, topic):
        super(RobotMonitor, self).__init__()
        self.top_items = []
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
        self.update_tree(msg)
        self.update_we(msg)

    def update_tree(self, msg):
        #Update the tree from the bottom

        names = [get_nice_name(k.name) for k in self.top_items]
        add = []
        for i in self._top_level(msg):
            name = get_nice_name(i.name)
            if name in names:
                self.top_items[names.index(name)].update(i.level, msg)
            else:
                nw = StatusItem(i)
                nw.update(i.level, msg)
                self.top_items.append(nw)
                add.append(nw)
        
        self.comp.addTopLevelItems(add)
        
    def _top_level(self, msg):
        ret = []
        for i in msg.status:
            if len(i.name.split('/')) == 2:
                ret.append(i)
        
        return ret

    def update_we(self, msg):
        for status in msg.status:
            if status.level == status.WARN:
                txt = "%s : %s"%(status.name, status.message)
                self.sig_warn.emit(txt)
            elif status.level == status.ERROR:
                txt = "%s : %s"%(status.name, status.message)
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
