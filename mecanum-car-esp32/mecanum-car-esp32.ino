#include <BluetoothSerial.h>
#include <math.h>
#include <L298N.h>

#define TIMEOUT_MS 20000 //for in case the car disconnect

unsigned long last_cmd_time;

BluetoothSerial bluetoothSerial; 


// Pin Definitions for Motor Driver 1 (L298N1)
#define M1_ENA 23
#define M1_IN1 22
#define M1_IN2 19
#define M1_IN3 18
#define M1_IN4 5
#define M1_ENB 17

// Pin Definitions for Motor Driver 2 (L298N2)
#define M2_ENA 14
#define M2_IN1 27
#define M2_IN2 26
#define M2_IN3 25
#define M2_IN4 33
#define M2_ENB 32

// Motors Instances
L298N motor1(M1_ENA, M1_IN1, M1_IN2);
L298N motor2(M1_ENB, M1_IN3, M1_IN4);
L298N motor3(M2_ENA, M2_IN1, M2_IN2);
L298N motor4(M2_ENB, M2_IN3, M2_IN4);


// axes
double x = 0;
double y = 0;
double turn = 0;


void setup() {

  bluetoothSerial.begin("ESP32test");

  last_cmd_time = millis();
}


void loop() {

  if (bluetoothSerial.available()) {
    last_cmd_time = millis();
    String raw = bluetoothSerial.readStringUntil('\n');

    if (raw.equals("s") || raw.equals("stop")) {
      stop();
    } else {
      parse(raw);
      set_motors();
    }
  } else if (millis() - last_cmd_time > TIMEOUT_MS) {
    stop();
  }
}


void parse(String raw) {
  String type = raw.substring(0, raw.indexOf(":"));
  String data = raw.substring(raw.indexOf(":") + 1);

  if (type == "lj") {
    x = data.substring(0, data.indexOf(",")).toDouble();
    y = data.substring(data.indexOf(",") + 1).toDouble();

    

  } else if (type == "rj") {
    turn = data.substring(0, data.indexOf(",")).toDouble();
  } else if (type == "axis") {
    int id = data.substring(0, data.indexOf(",")).toInt();
    double value = data.substring(data.indexOf(",") + 1).toDouble();

    switch (id) {
      case 0:
        x = value;
        break;
      case 1:
        y = value;
        // inverting y sign due to joystick configration
        y *= -1;
        break;
      case 2:
        turn = value;
      default:
        break;
    }
  }
}


void set_motors() {
  double theta = atan2(y, x);
  double power = hypot(x, y);

  double p_sin = sin(theta - PI / 4);
  double p_cos = cos(theta - PI / 4);

  double normalizer = max(max(abs(p_sin), abs(p_cos)), 0.1);

  double power1 = power * p_cos / normalizer + turn;
  uint8_t speed1 =  calculate_speed(power1);
  
  double power2 = power * p_sin / normalizer - turn;
  uint8_t speed2 =  calculate_speed(power2);
  
  double power3 = power * p_cos / normalizer - turn;
  uint8_t speed3 =  calculate_speed(power3);

  double power4 = power * p_sin / normalizer + turn;
  uint8_t speed4 =  calculate_speed(power4);

  motor1.setSpeed(speed1);
  motor1.run(get_direction(power1));

  motor2.setSpeed(speed2);
  motor2.run(get_direction(power2));

  motor3.setSpeed(speed3);
  motor3.run(get_direction(power3));

  motor4.setSpeed(speed4);
  motor4.run(get_direction(power4));

}


uint8_t calculate_speed(double power) {
  uint8_t speed = static_cast<uint8_t>(round(abs(power) * 255.0));
  speed = min(max(speed, static_cast<uint8_t>(0)), static_cast<uint8_t>(255));
  return speed;
}


L298N::Direction get_direction(double power) {
  if (power > 0) {
    return L298N::FORWARD;
  } else if (power < 0) {
    return L298N::BACKWARD;
  } else {
    return L298N::STOP;
  }
}


void stop() {
  motor1.setSpeed(0);
  motor1.run(L298N::STOP);
  motor2.setSpeed(0);
  motor2.run(L298N::STOP);
  motor3.setSpeed(0);
  motor3.run(L298N::STOP);
  motor4.run(L298N::STOP);
}
