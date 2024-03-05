//// Assign library for base box control project
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <MQTT.h>
#include <WiFi.h>

//// Assign pin of actuator and variable
#define Button_Pin   2
#define Buzz 32
#define Vol     34
boolean sent = false;
boolean wait = false;
String msg;
String pre_msg;

//// Config topic from library
LiquidCrystal_I2C lcd(0x27, 16, 2); // set up LCD-I2C 16 Col 2 Row
WiFiClient net;
MQTTClient client;

//// Config wifi path
const char ssid[] = "@JumboPlusIoT";
const char pass[] = "12345678";

//// Config sever path
const char mqtt_broker[]="test.mosquitto.org";
const char mqtt_topic[]="project1.21/heights";
const char mqtt_sub[]="project1.21/#";
const char mqtt_client_id[]="arduino_project_1.21"; // must change this string to a unique value
int MQTT_PORT=1883;

//// Function Topic
void connect();
void messageReceived(String &topic, String &payload);
void task_arduino(void *arg);

//// Setup command
void setup(){
  Serial.begin(9600);
  pinMode(Vol,INPUT);
  pinMode(Button_Pin,INPUT);
	lcd.init();
	lcd.backlight();
  WiFi.begin(ssid, pass);
  client.begin(mqtt_broker, MQTT_PORT, net);
  client.onMessage(messageReceived);
  connect();

  //// Method for using multicore esp32
  xTaskCreatePinnedToCore(
      task_arduino,
      "Task1",
      8192,
      NULL,
      0,
      NULL,
      0);
}

void loop(){
  // connect client
  client.loop();
  if(!client.connected()){
    connect();
  }

  // Interface running while get high level for tuning
  if(!sent && !wait){
    lcd.setCursor(3,0);
    lcd.print("PID Tuning");
    lcd.setCursor(0,1);
    lcd.print("Height(cm): ");
    lcd.setCursor(12,1);
    lcd.print(String(map(analogRead(Vol), 0, 4095, 0, 80)));
  }

  // Interface running while tuning and showwing tuning point and current heights msg for tuning from base box
  if(wait){
    lcd.setCursor(0,0);
    lcd.print("Tuning(cm): "+msg);
    lcd.setCursor(0,1);
    lcd.print("Height(cm): "+String(map(analogRead(Vol), 0, 4095, 0, 80)));
  }
  delay(400);
  lcd.clear();
}

// Function detail
void connect()
{
  // Connect Wifi screen
  int i = 0;
  if(WiFi.status() == WL_CONNECTED){
    lcd.clear();
    i = 0;
    lcd.setCursor(1,0);
    lcd.print("Connected WIFI");
    delay(500);
    lcd.clear();
  }
  while (WiFi.status() != WL_CONNECTED)
  {
    lcd.setCursor(1,0);
    lcd.print("ConnectingWIFI");
    lcd.setCursor(i,1);
    lcd.print(".");
    i++;
    if(i>15){
      i = 0;
      lcd.clear();
    }
    delay(1000);
  }
  // Connect client
  if(client.connect(mqtt_client_id)){
    lcd.clear();
    i = 0;
    lcd.setCursor(0,0);
    lcd.print("Connected Client");
    delay(500);
    lcd.clear();
  }
  while (!client.connect(mqtt_client_id))
  {
    lcd.setCursor(0,0);
    lcd.print("ConnectingClient");
    lcd.setCursor(i,1);
    lcd.print(".");
    i++;
    if(i>15){
      i = 0;
      lcd.clear();
    }
    delay(1000);
  }
  // connect topic subscribe of mqtt
  client.subscribe(mqtt_sub);
}

void messageReceived(String &topic, String &payload)
{
  // get payload from heights to showing tuning stat from both of base box and server control(Node-red)
  if(String(topic)=="project1.21/heights"){
    if(pre_msg!=payload && payload.toInt()>=0){
      pre_msg = payload;
      msg = payload;
      wait = true;
    }
  }
  // get payload from distance to alert completed tuning
  if(String(topic)=="project1.21/distance"){
    if(payload==msg && wait){
      lcd.clear();
      lcd.setCursor(6,1);
      lcd.print("Done");
      delay(1000);
      wait = false;
    }
  }
}

//// task for multicore function
void task_arduino(void *arg)
{
  while (1)
  {
    if(digitalRead(Button_Pin) && !sent && !wait){
      tone(Buzz, 500, 1200);
      sent = true;
      msg = String(map(analogRead(Vol), 0, 4095, 0, 80));
    }
    if(sent){
      client.publish(mqtt_topic, msg);
      sent = false;
      wait = true;
    }
  }
}
