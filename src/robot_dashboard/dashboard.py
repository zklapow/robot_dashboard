import roslib;roslib.load_manifest('robot_dashboard')
import rospy

import qt_gui.qt_binding_helper

from QtCore import QSize
from QtGui import QWidget, QHBoxLayout, QGroupBox, QToolBar
from qt_gui.plugin import Plugin

class Dashboard(Plugin):
    """Base class from which dashboards should inherit."""
    def __init__(self, context):
        super(Dashboard, self).__init__(context)
        self.context = context
        self.setup(context)

        self._main_widget = QToolBar()
        self._main_widget.setIconSize(QSize(80, 80))
        widgets = self.get_widgets()

        layout = QHBoxLayout()

        for k, v in widgets.iteritems():
            for i in v:
                try:
                    #i.setFixedSize(100, 100)
                    self._main_widget.addWidget(i)
                except:
                    raise(Exception("All widgets must be a subclass of QWidget!"))

            self._main_widget.addSeparator()

        # Display the dashboard
        context.add_widget(self._main_widget)

    def setup(self, context):
        """Called during ``__init__`` Subclasses should do initialization here.
        
        .. note::
            If this method is overriden it is important to call ``self.setObjectName()`` so that object names do not conflict.

        :param context: The plugin context
        :type context: qt_gui.plugin.Plugin
        """
        self.setObjectName('Dashboard')

    def get_widgets(self):
        """
        Most of the dashboard customization should be done here. If this function is not overriden the dashboard will display nothing.

        :returns: Dictionary of lists containing dashboard widgets.
        """
        return {}
