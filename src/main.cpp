#include <Arduino.h>
#include <pid.h>
#include <WiFi.h>
#include <MQTT.h>
#include <motor.h>
#include <NewPing.h>
#include <SimpleKalmanFilter.h>

// ? Define Values
/* Ultrasonic */
#define ECHO 32
#define TRIG 5
unsigned int distance;
NewPing sonar(TRIG, ECHO, 200);
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

/* Motor */
#define MOTOR_INA 0
#define MOTOR_INB 2
#define MOTOR_PWM 4
#define K_P 6
#define K_I 0.0173
#define K_D 0.000025
Motor motor_controller(20000, 10, false, MOTOR_PWM, MOTOR_INA, MOTOR_INB);
PID motor_pid((pow(2, 10) - 1) * -1, pow(2, 10) - 1, K_P, K_I, K_D);

int setpoint = 0;

/* Limit switch */
#define LIM_SWITCH 33
int pre_btn = 0;
int state = 0;

/* WIFI & MQTT*/
WiFiClient net;
MQTTClient client;

const char ssid[] = "@JumboPlusIoT";
const char pass[] = "12345678";

const char mqtt_broker[] = "test.mosquitto.org";
const char mqtt_height[] = "project1.21/heights";
const char mqtt_distance[] = "project1.21/distance";
const char mqtt_client_id[] = "arduino_project1.21_drone";
int MQTT_PORT = 1883;

// ? prototype fuction
/* Limit switch */
void IRAM_ATTR IO_INT_ISR();

/* WIFI & MQTT*/
void connect();
void messageReceived(String &topic, String &payload);

void setup()
{
  /* This method have 2 parts
      part 1 (Limit switch) : define pinmode of limit switch and attach interrupt with method call IO_INT_ISR
      part 2 (WIFI & MQTT) : connect wifi and mqtt
  */
  Serial.begin(115200);

  /* Limit switch */
  pinMode(LIM_SWITCH, INPUT_PULLUP);
  attachInterrupt(LIM_SWITCH, IO_INT_ISR, CHANGE);

  /* WIFI & MQTT*/
  WiFi.begin(ssid, pass);
  client.begin(mqtt_broker, MQTT_PORT, net);
  client.onMessage(messageReceived);
  connect();
}

void loop()
{
  /* This method have 3 parts
    part 1 (MQTT) : loop connection between mqtt client and mqtt broker than mqtt client publish value (distance) to topic (mqtt_distance)
    part 2 (Ultrasonic) : find distance of drone from base station and filter distance by SimpleKalmanFilter
    part 3 (Motor) : command motor with speed tuing by PID
      state 0 > command motor with speed tuing by PID
      state 1 > command motor with 60% of speed
  */
  /* MQTT*/
  client.loop();
  if (!client.connected())
  {
    connect();
  }

  /* Ultrasonic */
  unsigned int uS = sonar.ping();
  distance = constrain((uS / US_ROUNDTRIP_CM) - 2.8, 0, 100);
  distance = simpleKalmanFilter.updateEstimate(distance);
  if (setpoint == 0 && state == 1 && distance <= 1)
  {
    state = 0;
  }
  /* Motor */
  if (!state)
  {
    motor_controller.spin(motor_pid.compute(setpoint, distance));
  }
  else
  {
    motor_controller.spin(614);
  }
  Serial.println(String(distance) + " " + String(setpoint) + " " + String(motor_controller.getPwm()) + " " + String(state));

  /* MQTT*/
  client.publish(mqtt_distance, String(distance));
}

void IRAM_ATTR IO_INT_ISR()
{
  /*
    This method is called when lim_switch had pressed has 2 state
      state 0 : command motor with speed tuing by PID
      state 1 : command motor with 60% of speed
  */
  if (pre_btn != !digitalRead(LIM_SWITCH))
  {
    state = !digitalRead(LIM_SWITCH) ? 1 : 0;
    pre_btn = !digitalRead(LIM_SWITCH);
  }
}

void messageReceived(String &topic, String &payload)
{
  /*
    This method used to receive payload from mqtt server and assign values (setpoint) with payload has 2 state
      state 0 : command motor with speed tuing by PID
      state 1 : command motor with 60% of speed
  */
  Serial.println("incoming: " + topic + " - " + payload);
  if (payload.toInt() > 0 && payload.toInt() <= 80)
  {
    setpoint = payload.toInt();
  }
  else if (payload.toInt() == 0)
  {
    setpoint = 0;
    state = 1;
  }
}

void connect()
{
  /* This method have 2 parts
      part 1 (WIFI) : connect wifi
      part 2 (MQTT) : connect mqtt and subscribe topic with variable (mqtt_height)
  */
  /* WIFI */
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }

  /* MQTT */
  Serial.print("\nconnecting...");
  while (!client.connect(mqtt_client_id))
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");
  client.subscribe(mqtt_height);
}
