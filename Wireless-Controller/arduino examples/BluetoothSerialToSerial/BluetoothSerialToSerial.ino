
// bluetooth setup
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (Serial.available()) {
    SerialBT.println(Serial.readStringUntil('\n'));
  }
  if (SerialBT.available()) {
    String raw = SerialBT.readStringUntil('\n');

    parse(raw);    
  }

}


void parse(String raw) {
  String type = raw.substring(0, raw.indexOf(":"));
  String data = raw.substring(raw.indexOf(":") + 1);

  if (type == "lj") {
    String x = data.substring(0, data.indexOf(",")), y = data.substring(data.indexOf(",") + 1);

    // your function
    Serial.print("left-joystick");
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);

  } else if (type == "rj") {
    String x = data.substring(0, data.indexOf(",")), y = data.substring(data.indexOf(",") + 1);

    // your function
    Serial.print("right-joystick");
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);

  } else if (type == "axis") {
    String id = data.substring(0, data.indexOf(",")), value = data.substring(data.indexOf(",") + 1);

    // your function
    Serial.print("axis:");
    Serial.print(id);
    Serial.print(",");
    Serial.println(value);

  } else if (type == "button") {
    String id = data.substring(0, data.indexOf(",")), value = data.substring(data.indexOf(",") + 1);

    // your function
    Serial.print("button:");
    Serial.print(id);
    Serial.print(",");
    Serial.println(value);
  } else if (type == "hat") {
    String id = data.substring(0, data.indexOf(",")), values = data.substring(data.indexOf(",") + 1);
    String x = values.substring(0, data.indexOf(",")), y = values.substring(data.indexOf(",") + 1);

    // your function
    Serial.print("hat:");
    Serial.print(id);
    Serial.print(",");
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
  } else {
    Serial.println(raw);
  }
}
