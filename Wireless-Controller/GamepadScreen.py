from kivy.clock import Clock
from kivy.core.window import Window

from BaseScreen import BaseScreen
from GamepadLabels import *


class GamepadScreen(BaseScreen):
    axes_labels = {}
    buttons_labels = {}
    hats_labels = {}

    def __init__(self, **kwargs):
        super(GamepadScreen, self).__init__(**kwargs)
        self.gamepad = Window.bind(on_joy_axis=self.on_joy_axis, on_joy_hat=self.on_joy_hat,
                                   on_joy_button_down=self.on_joy_button_down, on_joy_button_up=self.on_joy_button_up)

        Clock.schedule_once(self.add_widgets, 0)

    def add_widgets(self, dt):
        self.axes_labels = {i: AxisLabel() for i in range(self._app.axes_count)}
        self.buttons_labels = {i: ButtonLabel() for i in range(self._app.buttons_count)}
        self.hats_labels = {i: HatLabel() for i in range(self._app.hats_count)}

        for i, axis in self.axes_labels.items():
            axis.name = str(i)
            self.add_to_container(axis)
        for i, button in self.buttons_labels.items():
            button.name = str(i)
            self.add_to_container(button)
        for i, hat in self.hats_labels.items():
            hat.name = str(i)
            self.add_to_container(hat)

    def add_to_container(self, widget):
        container = self.ids.container
        if self._app.axes_count + self._app.buttons_count + self._app.hats_count > container.cols * container.rows:
            container.rows += 1
        container.add_widget(widget)

    def on_joy_hat(self, win, gamepad_id, hat_id, value):
        try:
            self.hats_labels[hat_id].update_hat_value(value)
        except KeyError:
            new_hat = HatLabel()
            new_hat.name = str(hat_id)
            new_hat.update_hat_value(value)

            self._app.hats_count += 1
            self.hats_labels[hat_id] = new_hat
            self.add_to_container(new_hat)

        self._app.send(f"hat:{hat_id},{value[0]},{value[1]}")

    def on_joy_button_down(self, win, gamepad_id, button_id):
        value = 1
        try:
            self.buttons_labels[button_id].update_button_value(value)
        except KeyError:
            new_button = ButtonLabel()
            new_button.name = str(button_id)
            new_button.update_button_value(value)

            self._app.buttons_count += 1
            self.buttons_labels[button_id] = new_button
            self.add_to_container(new_button)

        self._app.send(f"button:{button_id},{value}")

    def on_joy_button_up(self, win, gamepad_id, button_id):
        value = 0
        try:
            self.buttons_labels[button_id].update_button_value(value)
        except KeyError:
            new_button = ButtonLabel()
            new_button.name = str(button_id)
            new_button.update_button_value(value)

            self._app.buttons_count += 1
            self.buttons_labels[button_id] = new_button
            self.add_to_container(new_button)

        self._app.send(f"button:{button_id},{value}")

    def on_joy_axis(self, win, gamepad_id, axis_id, value):
        value = self._app.round_axis_value(value / 32767)
        try:
            self.axes_labels[axis_id].update_axis_value(value)
        except KeyError:
            new_axis = AxisLabel()
            new_axis.name = str(axis_id)
            new_axis.update_axis_value(value)

            self._app.axes_count += 1
            self.axes_labels[axis_id] = new_axis
            self.add_to_container(new_axis)

        self._app.send(f"axis:{axis_id},{value}")
