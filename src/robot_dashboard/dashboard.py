import roslib;roslib.load_manifest('robot_dashboard')
import rospy

from qt_gui.plugin import Plugin
from QtGui import QWidget, QHBoxLayout, QGroupBox

class Dashboard(Plugin):
    def __init__(self, context):
        super(Dashboard, self).__init__(context)
        self.context = context
        self.setup(context)

        self._main_widget = QWidget()
        widgets = self.get_widgets()

        layout = QHBoxLayout()

        for k, v in widgets.iteritems():
            box = QGroupBox(str(k))
            box.setFixedSize(100, 100)
            blayout = QHBoxLayout()

            for i in v:
                try:
                    blayout.addWidget(i)
                except:
                    raise("All widgets must be a subclass of QWidget!")

            box.setLayout(blayout)
            layout.addWidget(box)

        # Display the dashboard
        self._main_widget.setLayout(layout)
        context.add_widget(self._main_widget)

    def setup(self, context):
        """Called during __init__. Subclasses should do initialization here."""
        self.setObjectName('Dashboard')

    def get_widgets(self):
        """Return a dictionary of lists containing the widgets to be displayed by this dashboard"""
        return {}
