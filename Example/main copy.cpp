#include <Arduino.h>
#include <pid.h>
#include <WiFi.h>
#include <MQTT.h>
#include <motor.h>
#include <NewPing.h>
#include <ESP32Servo.h>
#include <SimpleKalmanFilter.h>
#define K_P 80.0
#define K_I 0.04
#define K_D 0

Servo esc1;
Servo esc2;

WiFiClient net;
MQTTClient client;

// NewPing sonar(4, 5, 200); // NewPing setup of pins and maximum distance.
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// Motor motor_controller(20000, 10, false, 17, 15, 16);

PID motor_pid((pow(2, 10) - 1) * -1, pow(2, 10) - 1, K_P, K_I, K_D);

const char ssid[] = "@JumboPlusIoT";
const char pass[] = "12345678";

const char mqtt_broker[] = "test.mosquitto.org";
const char mqtt_distance[] = "project1.21/distance";
const char mqtt_height[] = "project1.21/height";
const char mqtt_client_id[] = "arduino_project1.21_drone"; // must change this string to a unique value
int MQTT_PORT = 1883;

unsigned int distance;

void messageReceived(String &topic, String &payload)
{
  Serial.println("incoming: " + topic + " - " + payload);
  // motor_controller.spin(motor_pid.compute(payload.toInt(), distance));
  Serial.println(String(motor_pid.compute(payload.toInt(), distance)));
}

void connect()
{
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect(mqtt_client_id))
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe(mqtt_height);
  // client.unsubscribe("/hello");
}

void setup()
{
  Serial.begin(115200);
  esc1.setPeriodHertz(80);
  esc1.attach(25, 1000, 2000);
  // WiFi.begin(ssid, pass);
  // client.begin(mqtt_broker, MQTT_PORT, net);
  // client.onMessage(messageReceived);
  // connect();
}

void loop()
{
  for (int i = 0; i <= 90; i++)
  {
    esc1.write(i);
    // esc2.write(i);
    delay(10);
  }
  delay(1000);
  // client.loop();
  // if (!client.connected())
  // {
  //   connect();
  // }
  // unsigned int uS = sonar.ping();
  // distance = uS / US_ROUNDTRIP_CM;
  // distance = int(simpleKalmanFilter.updateEstimate(distance));
  // client.publish(mqtt_distance, String(distance));
}
