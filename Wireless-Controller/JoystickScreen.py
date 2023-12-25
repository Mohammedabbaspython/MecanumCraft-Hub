from kivy.clock import Clock
from kivy.properties import ObjectProperty

from BaseScreen import BaseScreen


class JoystickScreen(BaseScreen):
    right_joystick = ObjectProperty()
    left_joystick = ObjectProperty()
    left_pad = (0, 0)
    right_pad = (0, 0)

    def __init__(self, **kwargs):
        super(JoystickScreen, self).__init__(**kwargs)
        Clock.schedule_interval(self.send_pad, 0.1)

    def send_pad(self, dt):
        left_pad = (self._app.round_axis_value(self.left_joystick.joystick.pad[0]), self._app.round_axis_value(
            self.left_joystick.joystick.pad[1]))
        right_pad = (self._app.round_axis_value(self.right_joystick.joystick.pad[0]), self._app.round_axis_value(
            self.right_joystick.joystick.pad[1]))
        if left_pad != self.left_pad:
            self._app.send(f"lj:{left_pad[0]},{left_pad[1]}")
            self.left_pad = left_pad
        if right_pad != self.right_pad:
            self._app.send(f"rj:{right_pad[0]},{right_pad[1]}")
            self.right_pad = right_pad
