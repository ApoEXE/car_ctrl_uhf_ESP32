#include <WiFi.h>
#include <PubSubClient.h>

// --- Configuration ---
const char *ssid = "Livebox7-1EF4";
const char *password = "gWL6vK3ztUNG";
const char *mqtt_server = "192.168.1.2";
const char *mqtt_topic_cmd = "home/car/command";
const char *mqtt_topic_s1 = "home/car/sensor1";
const char *mqtt_topic_s2 = "home/car/sensor2";

// --- Hardware Pins ---
#define SI_PIN 15 // Car Control Signal
#define STATUS_LED 2

// Sensor 1
#define TRIG1 4
#define ECHO1 5
// Sensor 2
#define TRIG2 17
#define ECHO2 16

// --- Global Variables ---
volatile int current_cmd = 0;
volatile bool safety_stop = false; // Flag for object detection
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
    Serial.print("New Command: ");
    Serial.println(current_cmd);
  }
}

// --- Function to measure HC-SR04 distance ---
float getDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Timeout of 25000us (approx 4 meters)
  long duration = pulseIn(echo, HIGH, 25000);
  if (duration == 0)
    return 400.0; // Return max range if no echo
  return (duration * 0.0343) / 2;
}

// --- CORE 1: High-Speed Signal Generation ---
void signalTask(void *pvParameters)
{
  pinMode(SI_PIN, OUTPUT);
  digitalWrite(SI_PIN, LOW);

  for (;;)
  {
    // Only send signal if command > 0 AND no obstacle is detected
    if (current_cmd > 0 && !safety_stop)
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

// --- CORE 0: Network & Sensor Task ---
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
    // 1. MQTT Connectivity
    if (!client.connected())
    {
      String cid = "ESP32_Car_" + String(random(0xffff), HEX);
      if (client.connect(cid.c_str()))
        client.subscribe(mqtt_topic_cmd);
    }
    client.loop();

    // 2. Measure Distances
    dist1 = getDistance(TRIG1, ECHO1);
    dist2 = getDistance(TRIG2, ECHO2);

    // 3. Safety Check
    if (dist1 < 20.0 || dist2 < 20.0)
    {
      if (!safety_stop)
        Serial.println("!!! SAFETY STOP ACTIVATED !!!");
      safety_stop = true;
    }
    else
    {
      safety_stop = false;
    }

    // 4. Publish & Serial Output (Every 500ms)
    if (millis() - last_publish > 500)
    {
      last_publish = millis();
      Serial.printf("S1: %.1f cm | S2: %.1f cm | Stop: %s\n",
                    dist1, dist2, safety_stop ? "YES" : "NO");

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

  // CORE 0: MQTT + Ultrasonic (Handles measurements and networking)
  xTaskCreatePinnedToCore(sensorNetworkTask, "SensNet", 8192, NULL, 5, NULL, 0);

  // CORE 1: Precision Control (Handles microsecond signal only)
  xTaskCreatePinnedToCore(signalTask, "Signal", 4096, NULL, 4, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}