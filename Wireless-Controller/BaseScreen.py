from kivy.properties import ObjectProperty
from kivy.uix.screenmanager import NoTransition
from kivymd.uix.screen import MDScreen


class BaseScreen(MDScreen):
    _app = ObjectProperty()

    def switch_screen(self, screen=""):
        self.manager.transition = NoTransition()
        if self.name in ("DirectTerminalScreen", "OptionsScreen"):
            screen = self.manager.previous_screen
        elif screen in ("GamepadScreen", "JoystickScreen"):
            self.manager.previous_screen = screen
        else:
            self.manager.previous_screen = self.name
        self.manager.current = screen
