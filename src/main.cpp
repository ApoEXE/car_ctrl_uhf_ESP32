#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h> // Include Bluetooth Serial library

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
BluetoothSerial SerialBT; // Create Bluetooth Serial Object
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- Helper: Dual Print (USB + Bluetooth) ---
void debugPrintln(String msg)
{
  Serial.println(msg); // Send to USB
  if (SerialBT.hasClient())
  {
    SerialBT.println(msg); // Send to Bluetooth if connected
  }
}

void debugPrint(String msg)
{
  Serial.print(msg);
  if (SerialBT.hasClient())
  {
    SerialBT.print(msg);
  }
}

// --- MQTT Callback ---
void callback(char *topic, byte *payload, unsigned int length)
{
  char message[16];
  if (length < 15)
  {
    memcpy(message, payload, length);
    message[length] = '\0';
    current_cmd = atoi(message);

    String logMsg = "MQTT CMD: " + String(current_cmd);
    debugPrintln(logMsg);
  }
}

// --- Distance Calculation (Helper: Single Shot) ---
float getRawDistance(int trig, int echo)
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

// --- Distance Calculation (Main: Median Filter) ---
float getMedianDistance(int trig, int echo)
{
  float readings[5];

  for (int i = 0; i < 5; i++)
  {
    readings[i] = getRawDistance(trig, echo);
    delay(5);
  }

  // Bubble Sort
  for (int i = 0; i < 4; i++)
  {
    for (int j = i + 1; j < 5; j++)
    {
      if (readings[i] > readings[j])
      {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  return readings[2];
}

// --- CORE 1: Precision Control (Signal Generation) ---
void signalTask(void *pvParameters)
{
  pinMode(SI_PIN, OUTPUT);
  digitalWrite(SI_PIN, LOW);

  for (;;)
  {
    bool allow_move = true;

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
    // Handle Wi-Fi Reconnect
    if (WiFi.status() != WL_CONNECTED)
    {
      // Optional: Add WiFi reconnect logic here if needed
      // usually WiFi.begin in setup handles auto-reconnect,
      // but explicit checks can be added.
    }

    if (!client.connected())
    {
      String cid = "ESP32_Car_" + String(random(0xffff), HEX);
      if (client.connect(cid.c_str()))
      {
        client.subscribe(mqtt_topic_cmd);
        debugPrintln("MQTT Reconnected");
      }
    }
    client.loop();

    dist1 = getMedianDistance(TRIG1, ECHO1);
    dist2 = getMedianDistance(TRIG2, ECHO2);

    forward_blocked = (dist1 < 20.0);
    backward_blocked = (dist2 < 20.0);

    if (millis() - last_publish > 500)
    {
      last_publish = millis();

      // Construct the log message
      String logData = "DIST Fwd: " + String(dist1, 1) +
                       " | Back: " + String(dist2, 1) +
                       " | Cmd: " + String(current_cmd);
      debugPrintln(logData);

      if (current_cmd == CMD_FORWARD && forward_blocked)
        debugPrintln(">>> FORWARD BLOCKED!");
      if (current_cmd == CMD_BACKWARD && backward_blocked)
        debugPrintln("<<< BACKWARD BLOCKED!");

      char s1_str[8], s2_str[8];
      dtostrf(dist1, 1, 1, s1_str);
      dtostrf(dist2, 1, 1, s2_str);
      client.publish(mqtt_topic_s1, s1_str);
      client.publish(mqtt_topic_s2, s2_str);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup()
{
  Serial.begin(115200);

  // --- Initialize Bluetooth ---
  SerialBT.begin("ESP32_Car_Debug"); // Bluetooth device name
  Serial.println("Bluetooth Started! Pair with 'ESP32_Car_Debug'");

  pinMode(STATUS_LED, OUTPUT);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    Serial.print(".");
  }
  digitalWrite(STATUS_LED, HIGH);
  Serial.println("\nWiFi Connected");
  Serial.println(WiFi.localIP());

  // Helper tasks
  xTaskCreatePinnedToCore(sensorNetworkTask, "SensNet", 8192, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(signalTask, "Signal", 4096, NULL, 4, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}