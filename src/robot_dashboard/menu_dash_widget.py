from QtGui import QPushButton, QMenu

class MenuDashWidget(QPushButton):
    def __init__(self, context, name, *args):
        super(MenuDashWidget, self).__init__()
        self.setObjectName(name)

        self._menu = QMenu()

        for arg in args:
            self._menu.addAction(arg)

        self.setMenu(self._menu)

    def add_action(self, name, callback):
        """Add an action to the menu, and return the newly created action"""
        return self._menu.addAction(name, callback)


