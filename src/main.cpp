#include <WiFi.h>
#include <PubSubClient.h>
#include <BluetoothSerial.h>

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

// We store history to fix the "Blind Zone" 400cm glitch
float last_valid_dist1 = 400.0;
float last_valid_dist2 = 400.0;

WiFiClient espClient;
PubSubClient client(espClient);
BluetoothSerial SerialBT;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- Helper: Dual Print ---
void debugPrintln(String msg)
{
  Serial.println(msg);
  if (SerialBT.hasClient())
  {
    SerialBT.println(msg);
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
    debugPrintln("MQTT CMD: " + String(current_cmd));
  }
}

// --- Distance Calculation (Single Shot) ---
float getRawDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // 12000 micros is approx 200cm.
  long duration = pulseIn(echo, HIGH, 12000);

  if (duration == 0)
    return 400.0; // Timeout
  return (duration * 0.0343) / 2;
}

// --- Distance Calculation (Improved Logic) ---
float getFilteredDistance(int trig, int echo, float &last_valid)
{
  int valid_count = 0;
  float sum_valid = 0;

  // Take 5 readings
  for (int i = 0; i < 5; i++)
  {
    float val = getRawDistance(trig, echo);

    // Filter Logic: Only count readings that are NOT timeouts (400)
    if (val < 390.0)
    {
      sum_valid += val;
      valid_count++;
    }

    // Small delay between burst pings to clear local noise
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  float result;

  // 1. Calculate Result based on valid readings
  if (valid_count > 0)
  {
    result = sum_valid / valid_count; // Average the good readings
  }
  else
  {
    result = 400.0; // All 5 readings were timeouts
  }

  // 2. Anti-Teleportation (Blind Zone Fix)
  // If result is 400, but we were recently < 150cm, it's a glitch.
  // We return the old value instead of jumping to 400.
  if (result >= 390.0 && last_valid < 150.0)
  {
    return last_valid;
  }

  // 3. Update history
  last_valid = result;
  return result;
}

// --- CORE 1: Precision Control ---
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
        for (int i = 0; i < 4; i++)
        { // Preamble
          digitalWrite(SI_PIN, LOW);
          ets_delay_us(1440);
          digitalWrite(SI_PIN, HIGH);
          ets_delay_us(680);
        }
        for (int i = 0; i < cmd_to_send; i++)
        { // Data
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

// --- CORE 0: Network & Sensor ---
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
    if (WiFi.status() == WL_CONNECTED && !client.connected())
    {
      String cid = "ESP32_Car_" + String(random(0xffff), HEX);
      if (client.connect(cid.c_str()))
        client.subscribe(mqtt_topic_cmd);
    }
    client.loop();

    // --- MEASUREMENT CYCLE ---

    // 1. Measure Sensor 1
    float d1 = getFilteredDistance(TRIG1, ECHO1, last_valid_dist1);

    // FIX: Wait 40ms before measuring Sensor 2.
    // This stops sound waves from Sensor 1 confusing Sensor 2.
    vTaskDelay(pdMS_TO_TICKS(40));

    // 2. Measure Sensor 2
    float d2 = getFilteredDistance(TRIG2, ECHO2, last_valid_dist2);

    // Update Blocks
    forward_blocked = (d1 < 20.0);
    backward_blocked = (d2 < 20.0);

    // Publish logic
    if (millis() - last_publish > 500)
    {
      last_publish = millis();

      String logData = "DIST Fwd: " + String(d1, 1) + " | Back: " + String(d2, 1) + " | Cmd: " + String(current_cmd);
      debugPrintln(logData);

      if (current_cmd == CMD_FORWARD && forward_blocked)
        debugPrintln(">>> FORWARD BLOCKED!");
      if (current_cmd == CMD_BACKWARD && backward_blocked)
        debugPrintln("<<< BACKWARD BLOCKED!");

      char s1_str[8], s2_str[8];
      dtostrf(d1, 1, 1, s1_str);
      dtostrf(d2, 1, 1, s2_str);
      client.publish(mqtt_topic_s1, s1_str);
      client.publish(mqtt_topic_s2, s2_str);
    }

    // Loop delay
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32_Car_Debug");
  Serial.println("BT Started");

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