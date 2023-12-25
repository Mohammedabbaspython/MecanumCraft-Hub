from csv import DictReader
from threading import Thread

from kivy.clock import Clock
from kivy.network.urlrequest import UrlRequest
from kivy.utils import platform
from kivymd.app import MDApp
from kivymd.toast import toast
from kivymd.uix.menu import MDDropdownMenu

if platform == "android":
    from bluetooth.BluetoothA import Bluetooth, get_bluetooth_ports
else:
    from bluetooth.Bluetooth import Bluetooth, get_bluetooth_ports


class WirelessController(MDApp):
    axes_count = 6
    buttons_count = 17
    hats_count = 1

    options = {}

    port = ""
    bluetooth_device = Bluetooth()
    received_data = []
    send_data_queue = []

    menu = None

    def __init__(self, **kwargs):
        super(WirelessController, self).__init__(**kwargs)
        Clock.schedule_interval(self.receive, 0.3)
        Clock.schedule_interval(self.send_data, 0)

    def build(self):
        self.set_options()
        self.title = "Wireless Controller"
        self.theme_cls.material_style = "M3"
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Indigo"

    def set_options(self):
        with open("options.csv", "r") as file:
            options = [i for i in DictReader(file)]

        for i in options:
            self.options[i['option']] = i['value']

    def quick_connection(self, connection):
        self.options["connection method"] = connection

    def round_axis_value(self, value: float) -> float:
        rounding = self.options.get('joysticks rounding')
        if rounding == "no rounding":
            return value
        else:
            return round(value, int(rounding))

    def ports_menu(self, caller_widget):
        menu_items = [
                         {
                             "text": self.options.get("ip address"),
                             "viewclass": "OneLineListItem",
                             "on_release": lambda x: self.quick_connection("wifi")

                         },
                         {
                             "text": "Disconnect",
                             "viewclass": "OneLineListItem",
                             "on_release": self.close_device,
                         }] + [
                         {
                             "text": i,
                             "viewclass": "OneLineListItem",
                             "on_release": lambda x=i: self.menu_callback(x),
                         } for i in get_bluetooth_ports()
                     ]

        self.menu = MDDropdownMenu(caller=caller_widget, items=menu_items, width_mult=4, )

        self.menu.open()

    def menu_callback(self, port):
        self.quick_connection("bluetooth")
        self.port = port
        self.menu.dismiss()
        connect_thread = Thread(target=self.bluetooth_device.connect, args=(self.port, Clock.schedule_once, (
            self.show_is_connected,)))
        connect_thread.start()

    def show_is_connected(self, dt):
        if self.bluetooth_device.connected:
            toast("connected")

    def send(self, cmd):
        if self.options.get('connection method') == "wifi" or self.bluetooth_device.connected:
            self.send_data_queue.append(cmd)

    def send_data(self, dt):
        if len(self.send_data_queue):
            if self.options.get('connection method') == "wifi":
                UrlRequest(f"http://{self.options.get('ip address')}/post",
                           req_body=self.send_data_queue[0],
                           on_failure=lambda *args: toast("connection error"),
                           method="POST"
                           )
            elif self.options.get('connection method') == "bluetooth" and self.bluetooth_device.connected:
                send_thread = Thread(target=self.bluetooth_device.send, args=(self.send_data_queue[0],))
                send_thread.start()
            self.send_data_queue.pop(0)

    def receive(self, dt):
        if self.options.get('connection method') == "wifi":
            UrlRequest(f"http://{self.options.get('ip address')}/",
                       on_success=lambda x, data: self.on_receive(data)
                       )
        elif self.options.get('connection method') == "bluetooth":
            if self.bluetooth_device.connected:
                receive_thread = Thread(target=self.bluetooth_device.receive, args=(self.on_receive,))
                receive_thread.start()

    def on_receive(self, data):
        if data:
            self.received_data.append(data)

    def close_device(self):
        self.menu.dismiss()
        if self.options.get('connection method') == "bluetooth":
            self.bluetooth_device.close()
            if not self.bluetooth_device.connected:
                toast("disconnected")


if __name__ == "__main__":
    app = WirelessController()
    app.run()
