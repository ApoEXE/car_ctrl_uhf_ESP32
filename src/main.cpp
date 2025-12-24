#include <WiFi.h>
#include <PubSubClient.h>

// --- Configuration ---
const char *ssid = "Livebox7-1EF4";
const char *password = "gWL6vK3ztUNG";
const char *mqtt_server = "192.168.1.2";
const char *mqtt_topic = "home/car/command";

#define SI_PIN 15
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
    Serial.print("MQTT CMD: ");
    Serial.println(current_cmd);
  }
}

// --- CORE 1: High-Speed Signal Generation ---
void signalTask(void *pvParameters)
{
  pinMode(SI_PIN, OUTPUT);
  digitalWrite(SI_PIN, LOW);

  for (;;)
  {
    if (current_cmd > 0)
    {
      int cmd_to_send = current_cmd;

      // We lock the core to send a 'Burst' of packets back-to-back
      // Sending 5 packets takes ~100ms, which keeps the RX-2C happy
      // but is short enough not to trigger the Watchdog.
      portENTER_CRITICAL(&timerMux);
      for (int burst = 0; burst < 5; burst++)
      {

        // 1. PREAMBLE (W2)
        for (int i = 0; i < 4; i++)
        {
          digitalWrite(SI_PIN, LOW);
          ets_delay_us(1440);
          digitalWrite(SI_PIN, HIGH);
          ets_delay_us(680);
        }

        // 2. DATA (W1)
        for (int i = 0; i < cmd_to_send; i++)
        {
          digitalWrite(SI_PIN, LOW);
          ets_delay_us(320);
          digitalWrite(SI_PIN, HIGH);
          if (i < (cmd_to_send - 1))
          {
            ets_delay_us(660);
          }
          else
          {
            ets_delay_us(460); // Special short last pulse
          }
        }

        // 3. MINIMAL GAP (Matching your simpleRC 100us)
        digitalWrite(SI_PIN, LOW);
        ets_delay_us(100);
      }
      portEXIT_CRITICAL(&timerMux);

      // Yield for 1ms to let the OS background tasks run
      vTaskDelay(1);
    }
    else
    {
      // Idle state: No pulses, ensure pin is low
      digitalWrite(SI_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

// --- CORE 0: Network Management ---
void networkTask(void *pvParameters)
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  for (;;)
  {
    if (!client.connected())
    {
      String cid = "ESP32_Car_" + String(random(0xffff), HEX);
      if (client.connect(cid.c_str()))
      {
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
  }
  digitalWrite(STATUS_LED, HIGH);

  // Network on Core 0, Signal on Core 1
  xTaskCreatePinnedToCore(networkTask, "Net", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(signalTask, "Sig", 4096, NULL, 4, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}