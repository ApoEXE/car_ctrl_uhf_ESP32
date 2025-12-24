#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- Configuration ---
#define WIFI_SSID "Livebox7-1EF4-WiFi7"
#define WIFI_PASS "gWL6vK3ztUNG"
#define MQTT_SERVER "192.168.1.2"
#define SIGNAL_PIN 15
#define STATUS_LED 2

// The exact topic from your original code
const char *MQTT_TOPIC = "home/car/command";

// --- Global Variables ---
volatile uint8_t current_cmd = 0;
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
    Serial.print("Command Received: ");
    Serial.println(current_cmd);
  }
}

// --- High Precision Signal Generation for RX-2C ---
void sendCarSignal(uint8_t cmd)
{
  if (cmd == 0)
    return;

  portENTER_CRITICAL(&timerMux);

  // 1. Start/Sync Phase: 4 Pulses of W2
  // W2 = 1500us HIGH, 500us LOW
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(SIGNAL_PIN, HIGH);
    ets_delay_us(1500);
    digitalWrite(SIGNAL_PIN, LOW);
    ets_delay_us(500);
  }

  // 2. Data Phase: 'n' Pulses of W1
  // W1 = 500us HIGH, 500us LOW
  for (int i = 0; i < cmd; i++)
  {
    digitalWrite(SIGNAL_PIN, HIGH);
    ets_delay_us(500);
    digitalWrite(SIGNAL_PIN, LOW);
    ets_delay_us(500);
  }

  portEXIT_CRITICAL(&timerMux);

  // 3. Dead Time / Blanking Period (CRITICAL)
  // The RX-2C needs this silence to process the pulse count
  vTaskDelay(pdMS_TO_TICKS(15));
}
void carSignalTask(void *pvParameters)
{
  pinMode(SIGNAL_PIN, OUTPUT);
  for (;;)
  {
    if (current_cmd > 0)
    {
      sendCarSignal(current_cmd);
      vTaskDelay(pdMS_TO_TICKS(15)); // Gap between telegrams
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(100)); // Sleep when idle
    }
  }
}

// --- MQTT & Network Task (Pinned to Core 0) ---
void mqttTask(void *pvParameters)
{
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);

  for (;;)
  {
    if (!client.connected())
    {
      Serial.print("Connecting to MQTT...");
      String clientId = "ESP32Car-" + String(random(0xffff), HEX);
      if (client.connect(clientId.c_str()))
      {
        Serial.println("Connected!");
        client.subscribe(MQTT_TOPIC); // Re-subscribe on every connect
      }
      else
      {
        Serial.print("Failed, rc=");
        Serial.println(client.state());
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(5)); // Give Core 0 background tasks time
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(STATUS_LED, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  // Set MQTT task to Priority 5, Signal to Priority 4
  // This ensures network packets are handled before generating the next wave cycle
  xTaskCreatePinnedToCore(mqttTask, "MQTT", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(carSignalTask, "Signal", 4096, NULL, 4, NULL, 1);
}

void loop()
{
  // Keep main loop alive with a slow pulse for status
  digitalWrite(STATUS_LED, HIGH);
  delay(1000);
  digitalWrite(STATUS_LED, LOW);
  delay(1000);
}