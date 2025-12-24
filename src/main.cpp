#include <WiFi.h>
#include <PubSubClient.h>

// --- Configuration ---
const char *ssid = "Livebox7-1EF4";
const char *password = "gWL6vK3ztUNG";
const char *mqtt_server = "192.168.1.2";
const char *mqtt_topic_cmd = "home/car/command";
const char *mqtt_topic_s1 = "home/car/sensor1";
const char *mqtt_topic_s2 = "home/car/sensor2";

// --- RX-2C Command Codes ---
#define CMD_FORWARD 10
#define CMD_BACKWARD 40

// --- Hardware Pins ---
#define SI_PIN 15
#define STATUS_LED 2
#define TRIG1 4
#define ECHO1 5
#define TRIG2 17
#define ECHO2 16

// --- Global Variables ---
volatile int current_cmd = 0;
volatile bool forward_blocked = false;
volatile bool backward_blocked = false;
float dist1 = 0, dist2 = 0;

WiFiClient espClient;
PubSubClient client(espClient);
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- MQTT Callback ---
void callback(char *topic, byte *payload, unsigned int length)
{
  char message[16];
  if (length < 15)
  {
    memcpy(message, payload, length);
    message[length] = '\0';
    current_cmd = atoi(message);
    Serial.print("MQTT CMD: ");
    Serial.println(current_cmd);
  }
}

// --- Distance Calculation ---
float getDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 25000);
  if (duration == 0)
    return 400.0;
  return (duration * 0.0343) / 2;
}

// --- CORE 1: Precision Control (Signal Generation) ---
void signalTask(void *pvParameters)
{
  pinMode(SI_PIN, OUTPUT);
  digitalWrite(SI_PIN, LOW);

  for (;;)
  {
    bool allow_move = true;

    // Directional Safety Check
    if (current_cmd == CMD_FORWARD && forward_blocked)
      allow_move = false;
    else if (current_cmd == CMD_BACKWARD && backward_blocked)
      allow_move = false;

    if (current_cmd > 0 && allow_move)
    {
      int cmd_to_send = current_cmd;
      portENTER_CRITICAL(&timerMux);
      for (int burst = 0; burst < 5; burst++)
      {
        // Preamble
        for (int i = 0; i < 4; i++)
        {
          digitalWrite(SI_PIN, LOW);
          ets_delay_us(1440);
          digitalWrite(SI_PIN, HIGH);
          ets_delay_us(680);
        }
        // Data
        for (int i = 0; i < cmd_to_send; i++)
        {
          digitalWrite(SI_PIN, LOW);
          ets_delay_us(320);
          digitalWrite(SI_PIN, HIGH);
          if (i < (cmd_to_send - 1))
            ets_delay_us(660);
          else
            ets_delay_us(460);
        }
        digitalWrite(SI_PIN, LOW);
        ets_delay_us(100);
      }
      portEXIT_CRITICAL(&timerMux);
      vTaskDelay(1);
    }
    else
    {
      digitalWrite(SI_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

// --- CORE 0: Network & Sensor Monitoring ---
void sensorNetworkTask(void *pvParameters)
{
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  unsigned long last_publish = 0;

  for (;;)
  {
    if (!client.connected())
    {
      String cid = "ESP32_Car_" + String(random(0xffff), HEX);
      if (client.connect(cid.c_str()))
        client.subscribe(mqtt_topic_cmd);
    }
    client.loop();

    dist1 = getDistance(TRIG1, ECHO1);
    dist2 = getDistance(TRIG2, ECHO2);

    // Update blocking flags independently
    forward_blocked = (dist1 < 20.0);
    backward_blocked = (dist2 < 20.0);

    if (millis() - last_publish > 500)
    {
      last_publish = millis();
      Serial.printf("DIST Fwd(S1): %.1f cm | Back(S2): %.1f cm | Active Cmd: %d\n",
                    dist1, dist2, current_cmd);

      if (current_cmd == CMD_FORWARD && forward_blocked)
        Serial.println(">>> FORWARD BLOCKED!");
      if (current_cmd == CMD_BACKWARD && backward_blocked)
        Serial.println("<<< BACKWARD BLOCKED!");

      char s1_str[8], s2_str[8];
      dtostrf(dist1, 1, 1, s1_str);
      dtostrf(dist2, 1, 1, s2_str);
      client.publish(mqtt_topic_s1, s1_str);
      client.publish(mqtt_topic_s2, s2_str);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(STATUS_LED, OUTPUT);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  }
  digitalWrite(STATUS_LED, HIGH);
  xTaskCreatePinnedToCore(sensorNetworkTask, "SensNet", 8192, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(signalTask, "Signal", 4096, NULL, 4, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}