import roslib;roslib.load_manifest('robot_dashboard')
import rospy

from QtGui import QMessageBox, QIcon, QPixmap
from PIL.ImageQt import ImageQt

def make_stately(self, signal = None):
    if not hasattr(self, 'sig_state') and not signal:
        raise(TypeError("make_stately requires the object to have a sig_state or for a signal to be provided"))
    elif not signal:
        signal = self.sig_state

    def ret(state):
        """This is the override point for customizing states.
        By default it simply sets the widgets stylesheet to one selected from ``self.states``. However you can do whatever you like here.
        For example you could change the widgets icon::
            def update_state(self, state):
                super(MyButton, self).update_state(state)
                if state == 1:
                    self._icon = QIcon(os.path.join(image_path, 'warn.svg'))
                    self.setIcon(self._icon)

        .. note:: You should not call this method directly but instead call :func:update_state

        .. autoclass:: robot_dashboard.mixins.StateMixin
            :private-members:

        :param state: The state to be set.
        :type state: int
        """
        if not hasattr(self, 'states'):
            self.states = ["#%s {background-color: green}"%self.objectName(), 
               "#%s {background-color: yellow}"%self.objectName(),
               "#%s {background-color: red}"%self.objectName()]
        self.setStyleSheet(self.states[state])

    if not hasattr(self, '_update_state'):
        self._update_state = ret
    self.update_state = lambda state: signal.emit(int(state))
    signal.connect(self._update_state)

def dashinfo(msg, obj, title = 'Info'):
    """Logs a message with ``rospy.loginfo`` and displays a ``QMessageBox`` to the user

    :param msg: Message to display.
    :type msg: str
    :param obj: Parent object for the ``QMessageBox``
    :type obj: QObject
    :param title: An optional title for the `QMessageBox``
    :type title: str
    """
    rospy.loginfo(msg)

    box = QMessageBox()
    box.setText(msg)
    box.setWindowTitle(title)
    box.show()

    obj._message_box = box

def dashwarn(msg, obj, title = 'Warning'):
    """Logs a message with ``rospy.logwarn`` and displays a ``QMessageBox`` to the user

    :param msg: Message to display.
    :type msg: str
    :param obj: Parent object for the ``QMessageBox``
    :type obj: QObject
    :param title: An optional title for the `QMessageBox``
    :type title: str
    """
    rospy.logwarn(msg)

    box = QMessageBox()
    box.setText(msg)
    box.setWindowTitle(title)
    box.show()

    obj._message_box = box

def dasherr(msg, obj, title = 'Error'):
    """Logs a message with ``rospy.logerr`` and displays a ``QMessageBox`` to the user

    :param msg: Message to display.
    :type msg: str
    :param obj: Parent object for the ``QMessageBox``
    :type obj: QObject
    :param title: An optional title for the `QMessageBox``
    :type title: str
    """
    rospy.logerr(msg)

    box = QMessageBox()
    box.setText(msg)
    box.setWindowTitle(title)
    box.show()

    obj._message_box = box

def make_icon(image, mode = 0):
    qim = ImageQt(image)
    icon = QIcon()
    icon.addPixmap(QPixmap.fromImage(qim), mode)
    return icon

