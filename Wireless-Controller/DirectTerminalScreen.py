from kivy.clock import Clock
from kivymd.uix.label import MDLabel

from BaseScreen import BaseScreen


class DirectTerminalScreen(BaseScreen):
    def __init__(self, **kwargs):
        super(DirectTerminalScreen, self).__init__(**kwargs)
        Clock.schedule_interval(self.update, 0.1)

    def update(self, dt):
        if len(self._app.received_data) > 0:
            self.update_container(str(self._app.received_data[0]), False)
            self._app.received_data.pop(0)

    def update_container(self, text, sent=True):
        if sent:
            self.ids.container.add_widget(MDLabel(text=text, halign="right"))
        else:
            self.ids.container.add_widget(MDLabel(text=text, theme_text_color="Custom", text_color=(0, 0, 1, 1)))

    def clear_terminal(self):
        self.ids.container.clear_widgets()
