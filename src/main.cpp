#include <Arduino.h> // Explicitly include Arduino for PlatformIO
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

// --- Speed & Safety Configuration ---
const float DIST_SLOW_START = 200.0; // Distance to start slowing down (cm)
const float DIST_STOP = 30.0;        // Distance to stop completely (cm)
const int MAX_SPEED_DELAY = 80;      // Max delay in ms (slower speed) when close to object

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

// Variable to control speed (PWM simulation via delay)
// 0 = Max Speed, >0 = Slowing down
volatile int current_speed_delay = 0;

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
    // Small delay between burst pings
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  float result;

  // 1. Calculate Result based on valid readings
  if (valid_count > 0)
  {
    result = sum_valid / valid_count;
  }
  else
  {
    result = 400.0;
  }

  // 2. Anti-Teleportation (Blind Zone Fix)
  if (result >= 390.0 && last_valid < 150.0)
  {
    return last_valid;
  }

  // 3. Update history
  last_valid = result;
  return result;
}

// --- Helper: Calculate Speed Delay ---
// Returns: 0 (Max speed) to MAX_SPEED_DELAY (Min speed)
int calculateSpeedDelay(float distance)
{
  if (distance >= DIST_SLOW_START)
  {
    return 0; // Max speed
  }
  else if (distance <= DIST_STOP)
  {
    return -1; // Should stop (handled by boolean flags, but -1 indicates block)
  }
  else
  {
    // Equation: Linear interpolation
    // closer to DIST_STOP = higher delay (slower)
    // closer to DIST_SLOW_START = lower delay (faster)

    float range = DIST_SLOW_START - DIST_STOP;
    float position = distance - DIST_STOP;

    // Normalized factor (0.0 = at stop line, 1.0 = at start of slow zone)
    float factor = position / range;

    // Invert: 1.0 (far) should be 0 delay. 0.0 (close) should be MAX delay.
    return (int)((1.0 - factor) * MAX_SPEED_DELAY);
  }
}

// --- CORE 1: Precision Control ---
void signalTask(void *pvParameters)
{
  pinMode(SI_PIN, OUTPUT);
  digitalWrite(SI_PIN, LOW);

  for (;;)
  {
    bool allow_move = true;

    // Safety Block Check
    if (current_cmd == CMD_FORWARD && forward_blocked)
      allow_move = false;
    else if (current_cmd == CMD_BACKWARD && backward_blocked)
      allow_move = false;

    if (current_cmd > 0 && allow_move)
    {
      int cmd_to_send = current_cmd;

      // Send the signal burst
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

      // --- SPEED CONTROL ---
      // If speed_delay is 0, we delay 1 tick (max speed).
      // If speed_delay is high (e.g. 50ms), we wait, effectively pulsing the motor.
      if (current_speed_delay > 0)
      {
        vTaskDelay(pdMS_TO_TICKS(current_speed_delay));
      }
      else
      {
        vTaskDelay(1);
      }
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

    // 1. Measure Sensor 1 (Forward)
    float d1 = getFilteredDistance(TRIG1, ECHO1, last_valid_dist1);

    vTaskDelay(pdMS_TO_TICKS(40));

    // 2. Measure Sensor 2 (Backward)
    float d2 = getFilteredDistance(TRIG2, ECHO2, last_valid_dist2);

    // 3. Update Blocking Logic (Stop completely)
    forward_blocked = (d1 <= DIST_STOP);
    backward_blocked = (d2 <= DIST_STOP);

    // 4. Update Speed Logic (Proportional Slow Down)
    int calculated_delay = 0;
    if (current_cmd == CMD_FORWARD)
    {
      calculated_delay = calculateSpeedDelay(d1);
    }
    else if (current_cmd == CMD_BACKWARD)
    {
      calculated_delay = calculateSpeedDelay(d2);
    }

    // Update global variable for Core 1
    current_speed_delay = calculated_delay;

    // Publish logic
    if (millis() - last_publish > 500)
    {
      last_publish = millis();

      String logData = "D1:" + String(d1, 0) + "cm D2:" + String(d2, 0) + "cm | Delay:" + String(current_speed_delay) + "ms";
      debugPrintln(logData);

      if (current_cmd == CMD_FORWARD && forward_blocked)
        debugPrintln(">>> BLOCKED FWD");
      if (current_cmd == CMD_BACKWARD && backward_blocked)
        debugPrintln("<<< BLOCKED BWD");

      char s1_str[8], s2_str[8];
      dtostrf(d1, 1, 1, s1_str);
      dtostrf(d2, 1, 1, s2_str);
      client.publish(mqtt_topic_s1, s1_str);
      client.publish(mqtt_topic_s2, s2_str);
    }

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