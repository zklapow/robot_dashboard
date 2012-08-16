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
import rospy
from robot_monitor import RobotMonitor

from .util import make_stately, make_icon

from rqt_console.console_widget import ConsoleWidget
from rqt_console.console_subscriber import ConsoleSubscriber
from rqt_console.message_data_model import MessageDataModel
from rqt_console.message_proxy_model import MessageProxyModel
from diagnostic_msgs.msg import DiagnosticArray

from QtCore import pyqtSignal, QMutex, QTimer, QSize
from QtGui import QPushButton, QMenu, QIcon, QWidget, QVBoxLayout, QColor, QProgressBar, QToolButton

from PIL import Image
from PIL.ImageQt import ImageQt

import os.path
import rospkg

rp = rospkg.RosPack()

image_path = os.path.join(rp.get_path('robot_dashboard'), 'images')

class IconToolButton(QToolButton):
    """This is the base class for all widgets. It provides state and icon switching support as well as convinience functions for creating icons.
    """
    state_changed = pyqtSignal(int)
    def __init__(self, name):
        super(IconToolButton, self).__init__()
        self.state_changed.connect(self._update_state)
        self.pressed.connect(self._pressed)
        self.released.connect(self._released)

        self.setStyleSheet('QToolButton {border: none;}')

        # List of QIcons to use for each state
        self._icons = []
        self._clicked_icons = []

        self.state = 0

    def _update_state(self, state):
        self.setIcon(self._icons[self.state])

    def update_state(self, state):
        self.state = state
        self.state_changed.emit(self.state)

    def  _pressed(self):
        self.setIcon(self._clicked_icons[self.state])

    def _released(self):
        self.setIcon(self._icons[self.state])

    def load_image(self, path):
        if os.path.exists(path):
            return Image.open(path)
        elif os.path.exists(os.path.join(image_path, path)):
            return Image.open(os.path.join(image_path, path)) 
        else:
            raise(Exception("Could not load %s"% path))

    def overlay(self, image, name):
        over = self.load_image(name)
        return Image.composite(over, image, over) 

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
    sig_state = pyqtSignal(int)
    def __init__(self, context, name, *args, **kwargs):
        super(MenuDashWidget, self).__init__()
        self.name = name
        self.setObjectName(self.name)

        make_stately(self, signal = self.sig_state)

        self._menu = QMenu()

        for arg in args:
            self._menu.addAction(arg)

        icon = kwargs.get('icon', None)

        # Check for icons existence
        if icon:
            if os.path.exists(icon):
                self._icon = QIcon(icon)
            elif os.path.exists(os.path.join(image_path, icon)):
                self._icon = QIcon(os.path.join(image_path, icon))
            else:
                raise(Exception("Could not create icon! %s does not exist."%icon))

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
    sig_state = pyqtSignal(int)
    def __init__(self, context, name, cb = None, icon = None, states = None):
        QPushButton.__init__(self)
        self.name = name
        self.setObjectName(self.name)

        self.clicked.connect(self.on_click)

        if states is not None:
            self.states = states

        make_stately(self, signal=self.sig_state)

        if icon:
            self._icon = QIcon(os.path.join(image_path, icon))
            self.setIcon(self._icon)
        if cb:
            self.clicked.connect(cb)

    def on_click(self):
        """Called when the button is clicked."""
        pass


class MonitorDashWidget(IconToolButton):
    """A widget which brings up the robot_monitor.

    :param context: The plugin context to create the monitor in.
    :type context: qt_gui.plugin_context.PluginContext
    """
    err= pyqtSignal()
    warn = pyqtSignal()
    def __init__(self, context):
        super(MonitorDashWidget, self).__init__('Monitor Widget')
        self.setObjectName("MonitorWidget")

        self._monitor = None
        self._monitor_sub = rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self._monitor_cb)
        self._last_msg_time = rospy.Time.now()
        self.err.connect(self._error)
        self.warn.connect(self._warning)

        # Only display a state for 10 sec
        self._timer = QTimer()
        self._timer.timeout.connect(self.ok)
        self._timer.setInterval(10000)
        self._timer.start()

        self._last_update = rospy.Time.now()

        # Unclicked icons
        self._icon = self.load_image('diagnostics.png')
        self._warn_icon = self.overlay(self._icon, 'warn-overlay.png')
        self._err_icon = self.overlay(self._icon, 'err-overlay.png')
        self._stale_icon = self.overlay(self._icon, 'stale-overlay.png')
        self._icons = [make_icon(self._icon), make_icon(self._warn_icon), make_icon(self._err_icon), make_icon(self._stale_icon)]

        # Clicked icons
        self._icon_click = self.load_image('diagnostics-click.png')
        self._warn_click = self.overlay(self._icon_click, 'warn-overlay.png')
        self._err_click = self.overlay(self._icon_click, 'err-overlay.png')
        self._stale_click = self.overlay(self._icon_click, 'stale-overlay.png')
        self._clicked_icons = [make_icon(self._icon_click), make_icon(self._warn_click), make_icon(self._err_click), make_icon(self._stale_click)]

        self.setIcon(make_icon(self._icon))

        self.context = context

        self.clicked.connect(self._show_monitor)

        self.state = 0
        self.update_state(self.state)

    def _show_monitor(self):
        if self._monitor is None:
            self._monitor = RobotMonitor('diagnostics_agg')
            self._monitor.destroyed.connect(self._monitor_close)
        self.context.add_widget(self._monitor)

    def _monitor_cb(self, msg):
        self._last_msg_time = rospy.Time.now()
        warn = 0
        err = 0
        for status in msg.status:
            if status.level == status.WARN:
                warn = warn + 1
            elif status.level == status.ERROR:
                err = err + 1

        if err > 0:
            self.err.emit()
        elif warn > 0:
            self.warn.emit()

    def _error(self):
        self.update_state(2)

        # Restart the timer when a new state arrives
        self._timer.start()

    def _warning(self):
        if self.state <= 1:
            self.update_state(1)

            # Restaert the timer when a new state arrives
            self._timer.start()

    def ok(self):
        # Each time the timer runs we are either ok or stale
        diff = rospy.Time.now() - self._last_msg_time
        if diff.to_sec() > 5:
            self.update_state(3)
        else:
            self.update_state(0)

    def _monitor_close(self):
        self._monitor.close()
        self._monitor = None

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

class BatteryDashWidget(QProgressBar):
    perc_sig = pyqtSignal(float)
    def __init__(self, context, name='Battery'):
        super(BatteryDashWidget, self).__init__()
        self.setObjectName(name)

        self.perc_sig.connect(self._update_perc)
        self.perc_sig.emit(0.0)

        self.setRange(0, 100)
        self.setValue(50)

        self.setMaximumSize(75, 25)

    def update_perc(self, val):
        self.perc_sig.emit(val)

    def _update_perc(self, val):
        self._perc = val
        self.setValue(val)

    def update_time(self, val):
        self.time_remaining = val
        self.setStatusTip("%s remaining"%val)

    def update_plug(self, state):
        self.plugged_in = state
