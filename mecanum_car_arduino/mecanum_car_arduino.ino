#include <math.h>
#include <SoftwareSerial.h>
#include <AFMotor.h>

#define TIMEOUT_MS 20000 //for in case the car disconnect

unsigned long last_cmd_time;

SoftwareSerial bluetoothSerial(9, 10);  // RX, TX and they should be conected reversly to module pins

//initial motors pin
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(4, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(2, MOTOR34_1KHZ);

// axes
float x = 0;
float y = 0;
float turn = 0;


void setup() {

  bluetoothSerial.begin(9600);

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
    x = data.substring(0, data.indexOf(",")).toFloat();
    y = data.substring(data.indexOf(",") + 1).toFloat();



  } else if (type == "rj") {
    turn = data.substring(0, data.indexOf(",")).toFloat();
  } else if (type == "axis") {
    int id = data.substring(0, data.indexOf(",")).toInt();
    float value = data.substring(data.indexOf(",") + 1).toFloat();

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
  float theta = atan2(y, x);
  float power = hypot(x, y);

  float p_sin = sin(theta - PI / 4);
  float p_cos = cos(theta - PI / 4);

  float normalizer = max(max(abs(p_sin), abs(p_cos)), 0.1);

  float power1 = power * p_cos / normalizer + turn;
  uint8_t speed1 =  calculate_speed(power1);

  float power2 = power * p_sin / normalizer - turn;
  uint8_t speed2 =  calculate_speed(power2);

  float power3 = power * p_cos / normalizer - turn;
  uint8_t speed3 =  calculate_speed(power3);

  float power4 = power * p_sin / normalizer + turn;
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


uint8_t calculate_speed(float power) {
  uint8_t speed = static_cast<uint8_t>(round(abs(power) * 255.0));
  speed = min(max(speed, 0), 255);
  return speed;
}


int get_direction(float power) {
  if (power > 0) {
    return FORWARD;
  } else if (power < 0) {
    return BACKWARD;
  } else {
    return RELEASE;
  }
}


void stop() {
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
