from jnius import autoclass


class AndroidBluetooth:
    ReceiveData = None
    SendData = None
    socket = None

    def __init__(self):
        self.BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')
        self.BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice')
        self.BluetoothSocket = autoclass('android.bluetooth.BluetoothSocket')
        self.UUID = autoclass('java.util.UUID')
        self.BufferReader = autoclass('java.io.BufferedReader')
        self.InputStream = autoclass('java.io.InputStreamReader')
        self.ConnectionEstablished = False

    def get_android_bluetooth_socket(self, device_name):
        paired_devices = self.BluetoothAdapter.getDefaultAdapter().getBondedDevices().toArray()
        for device in paired_devices:
            if device.getName() == device_name:
                self.socket = device.createRfcommSocketToServiceRecord(
                    self.UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"))
                self.ReceiveData = self.BufferReader(self.InputStream(self.socket.getInputStream()))

                self.SendData = self.socket.getOutputStream()
                self.socket.connect()
                self.ConnectionEstablished = True
        return self.ConnectionEstablished

    def BluetoothSend(self, Message):
        if self.ConnectionEstablished:
            self.SendData.write(Message)
        else:
            pass

    def BluetoothReceive(self):
        data_stream = ''
        if self.ConnectionEstablished:
            data_stream = str(self.ReceiveData.readLine())
        return data_stream

    def close(self):
        if self.ConnectionEstablished:
            self.socket.close()

    def get_paired_devices(self):
        devices = []
        paired_devices = self.BluetoothAdapter.getDefaultAdapter().getBondedDevices().toArray()
        for i in paired_devices:
            devices.append(i.getName())
        return devices


