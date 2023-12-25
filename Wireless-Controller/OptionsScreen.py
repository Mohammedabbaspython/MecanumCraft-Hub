from csv import DictReader, DictWriter

from kivymd.uix.menu import MDDropdownMenu

from BaseScreen import BaseScreen


class OptionsScreen(BaseScreen):
    rounding_menu = MDDropdownMenu

    def open_rounding_menu(self, caller):
        rounding_menu_items = [
            {"viewclass": "OneLineListItem",
             "text": i,
             "on_release": lambda x=i: self.rounding_menu_callback(x, caller)
             }
            for i in ["no rounding", *(str(i) for i in range(4))]
        ]

        self.rounding_menu = MDDropdownMenu(items=rounding_menu_items, width_mult=5)
        self.rounding_menu.caller = caller
        self.rounding_menu.open()

    def rounding_menu_callback(self, value, caller):
        self.update_option("joysticks rounding", value)
        caller.text = value
        self.rounding_menu.dismiss()

    @staticmethod
    def get_option(option):
        with open("options.csv", "r") as file:
            options = [i for i in DictReader(file)]

        for i in options:
            if i['option'] == option:
                return i['value']

    def update_option(self, option, value):
        with open("options.csv", "r") as file:
            options = [i for i in DictReader(file)]

        for i in options:
            if i['option'] == option:
                i['value'] = value
                self._app.options[option] = value

        with open("options.csv", "w") as file:
            writer = DictWriter(file, fieldnames=["option", "value"], delimiter=",")
            writer.writeheader()
            for i in options:
                writer.writerow(i)
