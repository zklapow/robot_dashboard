def set_state(self):
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
    return ret
