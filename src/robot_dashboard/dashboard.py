import roslib;roslib.load_manifest('robot_dashboard')
import rospy

from qt_gui.plugin import Plugin

class Dashboard(Plugin):
    def __init__(self, context):
        super(Dashboard, self).__init__(context)

        self.setObjectName('Dashboard')
