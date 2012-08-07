"""
.. module:: widgets
    :synopsis: Widgets for the robot_dashboard.

.. moduleauthor:: Ze'ev Klapow <zklapow@willowgarage.com>

This module provides a set of standard widgets for using with the dashboard.

To use them you must provide instances of the to your dashboard in its :func:`get_widgets` method. For example::
    
    from robot_dashboard.dashboard import Dashboard
    from robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget

    class MyDashboard(Dashboard):
        def get_widgets(self):
            self.monitor = MonitorDashWidget(self.context)
            self.console = ConsoleDashWidget(self.console)

            return({'Diagnostics': [self.monitor, self.console]})

Would create a simple dashboard with the ability to open a robot_monitor and a ROS console.

Widget Types
============

"""

import roslib;roslib.load_manifest('robot_dashboard')
from robot_monitor import RobotMonitor

from .util import set_state

from rqt_console.console_widget import ConsoleWidget
from rqt_console.console_subscriber import ConsoleSubscriber
from rqt_console.message_data_model import MessageDataModel
from rqt_console.message_proxy_model import MessageProxyModel

from QtCore import pyqtSignal, QMutex, QTimer, QSize
from QtGui import QPushButton, QMenu, QIcon, QWidget, QVBoxLayout, QColor

import os.path
import rospkg

rp = rospkg.RosPack()

image_path = os.path.join(rp.get_path('robot_dashboard'), 'images')

class MenuDashWidget(QPushButton):
    """A widget which displays a pop-up menu when clicked

    :param context: The plugin context to create the widget in.
    :type context: qt_gui.plugin_context.PluginContext
    :param name: The name to give this widget.
    :type name: str
    :param args: A set of actions this menu should perform.
    :type args: QtGui.QAction
    :param icon: The icon to display in this widgets button.
    :type icon: str
    """
    def __init__(self, context, name, *args, **kwargs):
        super(MenuDashWidget, self).__init__()
        self.name = name
        self.setObjectName(self.name)

        self._menu = QMenu()

        for arg in args:
            self._menu.addAction(arg)

        icon = kwargs.get('icon', None)

        if icon:
            self._icon = QIcon(os.path.join(image_path, icon))
            self.setIcon(self._icon)

        self.setMenu(self._menu)

        # Remove the stupid menu indicator
        self.setStyleSheet('QPushButton::menu-indicator {image: url(none.jpg);}')

    def add_action(self, name, callback):
        """Add an action to the menu, and return the newly created action.

        :param name: The name of the action.
        :type name: str
        :param callback: Function to be called when this item is pressed.
        :type callback: callable
        """
        return self._menu.addAction(name, callback)

class ButtonDashWidget(QPushButton):
    """A simple customizable push button widget.

    :param context: The plugin context to create the widget in.
    :type context: qt_gui.plugin_context.PluginContext
    :param name: The name to give this widget.
    :type name: str
    :param cb: A function to be called when this button is clicked.
    :type cb: callable
    :param icon: The icon to display in this widgets button.
    :type icon: str
    """
    sig_up = pyqtSignal(int)
    def __init__(self, context, name, cb = None, icon = None, states = None):
        QPushButton.__init__(self)
        self.name = name
        self.setObjectName(self.name)

        if states is not None:
            self.states = states

        #TODO: Encapsulate this functionality
        self._update_state = set_state(self)
        self.update_state = lambda state: self.sig_up.emit(int(state))
        self.sig_up.connect(self._update_state)

        if icon:
            self._icon = QIcon(os.path.join(image_path, icon))
            self.setIcon(self._icon)
        if cb:
            self.clicked.connect(cb)


class MonitorDashWidget(QPushButton):
    """A widget which brings up the robot_monitor.

    :param context: The plugin context to create the monitor in.
    :type context: qt_gui.plugin_context.PluginContext
    """
    def __init__(self, context):
        super(MonitorDashWidget, self).__init__()
        self.setObjectName("MonitorWidget")

        self._monitor = RobotMonitor('diagnostics_agg')
        self._monitor.destroyed.connect(self._monitor_close)
        self._monitor.sig_err.connect(self.err)
        self._monitor.sig_warn.connect(self.warn)

        # Only display a state for 10 sec
        self._timer = QTimer()
        self._timer.timeout.connect(self.ok)
        self._timer.setInterval(10000)
        self._timer.start()

        self.icon = QIcon(os.path.join(image_path, 'wrench.svg'))
        self.setIcon(self.icon)

        self.context = context

        self.clicked.connect(self._show_monitor)

        self.state = 0
        #self.setStyleSheet('QPushButton {background-color: green; border: none;}')

    def _show_monitor(self):
        print(self._monitor)
        #self.context.add_widget(self._monitor)
        self._monitor.show()

    def err(self, msg):
        self.state = 2
        #self.setStyleSheet('QPushButton {background-color:Red; border: none;}')

        # Restart the timer when a new state arrives
        self._timer.start()

    def warn(self, msg):
        if self.state <= 1:
            self.state = 1
            #self.setStyleSheet('QPushButton {background-color:Yellow; border: none;}')

            # Restaert the timer when a new state arrives
            self._timer.start()

    def ok(self):
        self.state = 0
        #self.setStyleSheet('QPushButton {background-color:Green; border: none;}')

    def _monitor_close(self):
        print("Monitor closed")

class ConsoleDashWidget(QPushButton):
    """A widget which brings up the ROS console.

    :param context: The plugin context to create the monitor in.
    :type context: qt_gui.plugin_context.PluginContext
    """
    def __init__(self, context):
        super(ConsoleDashWidget, self).__init__()
        self.setObjectName('Console')

        self._icon = QIcon(os.path.join(image_path, 'chat.svg'))
        self.setIcon(self._icon)

        self._datamodel = MessageDataModel()
        self._proxymodel = MessageProxyModel()
        self._proxymodel.setSourceModel(self._datamodel)

        self._subscriber = ConsoleSubscriber(self._message_cb)

        self._console = None
        self.context = context
        self.clicked.connect(self._show_console)

        self._mutex = QMutex()
        self._timer = QTimer()
        self._timer.timeout.connect(self._insert_messages)
        self._timer.start(100)

    def _show_console(self):
        if not self._console:
            self._console = ConsoleWidget(self._proxymodel)
            self._console.destroyed.connect(self._console_destroyed)

        self.context.add_widget(self._console)
 
    def _insert_messages(self):
        self._mutex.lock()
        msgs = self._datamodel._insert_message_queue
        self._datamodel._insert_message_queue = []
        self._mutex.unlock()
        self._datamodel.insert_rows(msgs)

        # The console may not yet be initialized or may have been closed
        # So fail silently
        try:
            self._console.update_status()
        except:
            pass

    def _message_cb(self, msg): 
        if not self._datamodel._paused:
            self._mutex.lock()
            self._datamodel._insert_message_queue.append(msg)
            self._mutex.unlock()

    def _console_destroyed(self):
        self._console = None
