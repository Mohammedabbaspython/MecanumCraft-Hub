from kivy.properties import StringProperty
from kivymd.uix.boxlayout import MDBoxLayout


class AxisLabel(MDBoxLayout):
    name = StringProperty("")

    def update_axis_value(self, value: float):
        self.ids.axis_value.text = str(value)


class ButtonLabel(MDBoxLayout):
    name = StringProperty("")

    def update_button_value(self, value: int):
        self.ids.button_value.text = str(value)


class HatLabel(MDBoxLayout):
    name = StringProperty("")

    def update_hat_value(self, value: int):
        self.ids.hat_value.text = str(value)
