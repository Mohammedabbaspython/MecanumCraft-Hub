from bluetooth.AndroidBluetooth import AndroidBluetooth

from traceback import format_exc


# returns a list of strings containing the names of the bluetooth com ports
def get_bluetooth_ports() -> list:
    return AndroidBluetooth().get_paired_devices()


class Bluetooth:
    device = AndroidBluetooth()
    connected = False

    def connect(self, port: str, on_connect=lambda *args: None, args=()):
        try:
            self.connected = self.device.get_android_bluetooth_socket(port)
            on_connect(*args)
        except:
            print(format_exc())

    def send(self, data: str, new_line: bool = True):
        data = str(data)
        if new_line:
            data += "\n"
        if self.connected:
            self.device.BluetoothSend(bytes(data, 'utf-8'))

    # it will return an empty string if not connected
    # on_receive is function that take a function with one parameter
    def receive(self, on_receive=lambda data: None) -> str:
        data = ''
        if self.connected:
            data = self.device.BluetoothReceive()
            on_receive(data)
        return data

    def close(self):
        if self.connected:
            self.device.close()
            self.connected = False

    def __del__(self):
        self.close()


if __name__ == "__main__":

    def callback(arg1, arg2):
        print(arg1, arg2)


    # get all available serial ports
    print(get_bluetooth_ports())

    port_name = input("enter port: ")

    # initialize an object with on_connect callback
    bl = Bluetooth()
    bl.connect(port_name, on_connect=callback, args=(1, 2))

    print(bl.connected)

    # initialize an object without on_connect callback
    # bl = Bluetooth()
    # bl.connect(port_name)

    # send data
    bl.send("Hello World!\n")

    # receive data
    received_data = bl.receive()
    while not received_data:
        received_data = bl.receive()
    print(received_data)

    print("finish")
