#include <WiFi.h>
#include <PubSubClient.h>

// --- Configuration ---
const char *ssid = "Livebox7-1EF4";
const char *password = "gWL6vK3ztUNG";
const char *mqtt_server = "192.168.1.2";
const char *mqtt_topic = "home/car/command";

#define SI_PIN 15 // Signal Injection Pin
#define STATUS_LED 2

// --- Shared Variables ---
volatile int current_cmd = 0;
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

// --- SIGNAL GENERATION (Using your validated timing) ---
void send_validated_signal(int cmd)
{
  if (cmd <= 0)
    return;

  // Disabling interrupts on this core for microsecond precision
  portENTER_CRITICAL(&timerMux);

  // 1. PREAMBLE (W2 Sequence) - Repeated 4 times
  // Timing from your working code: LOW 1440us, HIGH 680us
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(SI_PIN, LOW);
    ets_delay_us(1440);
    digitalWrite(SI_PIN, HIGH);
    ets_delay_us(680);
  }

  // 2. DATA COMMAND (W1 Sequence)
  // Logic: Low 320us, High 660us (Last pulse High 460us)
  for (int i = 0; i < cmd; i++)
  {
    // LOW Phase
    digitalWrite(SI_PIN, LOW);
    ets_delay_us(320);

    // HIGH Phase
    digitalWrite(SI_PIN, HIGH);
    if (i < (cmd - 1))
    {
      ets_delay_us(660); // Normal High
    }
    else
    {
      ets_delay_us(460); // Shorter final High
    }
  }

  // 3. Packet Gap (from your working code)
  digitalWrite(SI_PIN, LOW);
  ets_delay_us(100);

  portEXIT_CRITICAL(&timerMux);
}

// --- CORE 1: Dedicated to Signal Timing ---
void signalTask(void *pvParameters)
{
  pinMode(SI_PIN, OUTPUT);
  digitalWrite(SI_PIN, LOW);
  for (;;)
  {
    if (current_cmd > 0)
    {
      send_validated_signal(current_cmd);
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(50)); // Idle
    }
  }
}

// --- CORE 0: Dedicated to MQTT & WiFi ---
void networkTask(void *pvParameters)
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  for (;;)
  {
    if (!client.connected())
    {
      Serial.print("Connecting MQTT...");
      String cid = "ESP32_Car_" + String(random(0xffff), HEX);
      if (client.connect(cid.c_str()))
      {
        Serial.println("OK");
        client.subscribe(mqtt_topic);
      }
      else
      {
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(10));
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
    Serial.print(".");
  }
  Serial.println("\nWiFi Ready.");
  digitalWrite(STATUS_LED, HIGH);

  // Network on Core 0, Signal on Core 1
  xTaskCreatePinnedToCore(networkTask, "Net", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(signalTask, "Sig", 4096, NULL, 4, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}