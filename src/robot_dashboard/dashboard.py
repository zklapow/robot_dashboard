import roslib;roslib.load_manifest('robot_dashboard')
import rospy

import qt_gui.qt_binding_helper

from QtCore import QSize
from QtGui import QWidget, QHBoxLayout, QGroupBox, QToolBar
from qt_gui.plugin import Plugin

class Dashboard(Plugin):
    """Base class from which dashboards should inherit."""
    def __init__(self, context, name = None):
        super(Dashboard, self).__init__(context)
        self.context = context

        if not name:
            self.name = 'Dashboard'
        else:
            self.name = name

        self.setup(context)

        self._main_widget = QToolBar()
        self._main_widget.setIconSize(QSize(80, 80))
        self._main_widget.setObjectName(self.name)
        self._main_widget.on_close.connect(self.on_close)
        widgets = self.get_widgets()

        layout = QHBoxLayout()

        for v in widgets:
            for i in v:
                try:
                    #i.setFixedSize(100, 100)
                    self._main_widget.addWidget(i)
                except:
                    raise(Exception("All widgets must be a subclass of QWidget!"))

            self._main_widget.addSeparator()

        # Display the dashboard
        context.add_toolbar(self._main_widget)

    def setup(self, context):
        """Called during ``__init__`` Subclasses should do initialization here.
        
        .. note::
            If this method is overriden it is important to call ``self.setObjectName()`` so that object names do not conflict.

        :param context: The plugin context
        :type context: qt_gui.plugin.Plugin
        """
        pass

    def on_close(self):
        """Called when the toolbar is closed by Qt. Cleanup shoudl be done here.
        """
        pass

    def get_widgets(self):
        """
        Most of the dashboard customization should be done here. If this function is not overriden the dashboard will display nothing.

        :returns: List of lists containing dashboard widgets.
        """
        return []
