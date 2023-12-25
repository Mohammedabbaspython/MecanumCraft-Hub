#include <WiFi.h>
#include <WebServer.h>

#define SSD "MOHAMMED-LEGION"
#define PASSWORD "12345678"

WebServer server(80);

void setup() {
  Serial.begin(115200);
  connect();
}


void loop() {
  server.handleClient();
}


void connect() {
  WiFi.mode(WIFI_STA);

  WiFi.begin(SSD, PASSWORD);
  Serial.print("connecting to ");
  Serial.print(SSD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n connected");

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/post", handlePost);
  server.on("/", handleGet);
  server.begin();
}


void handlePost() {
  String raw = server.arg("plain");

  server.send(200, "text/plain", "received");
  // handel data
  parse(raw);
}


void handleGet() {
  if (Serial.available()) {
    server.send(200, "text/plain", Serial.readString());
  } else {
    server.send(404, "text/plain", "no data");
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
