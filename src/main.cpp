#include <WiFi.h>
#include <PubSubClient.h>

// --- Network Configuration ---
const char *ssid = "Livebox7-1EF4";
const char *password = "gWL6vK3ztUNG";
const char *mqtt_server = "192.168.1.2";
const char *mqtt_topic = "home/car/command";

// --- Hardware Pins ---
#define SIGNAL_PIN 15 // D15 connected to RX-2C SI Pin
#define STATUS_LED 2  // Built-in LED

// --- Variables ---
volatile int current_cmd = 0; // Command shared between cores
WiFiClient espClient;
PubSubClient client(espClient);
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- MQTT Callback: Receives commands from publisher ---
void callback(char *topic, byte *payload, unsigned int length)
{
  char message[16];
  if (length < 15)
  {
    memcpy(message, payload, length);
    message[length] = '\0';
    current_cmd = atoi(message); // Update command globally
    Serial.print("MQTT Command Received: ");
    Serial.println(current_cmd);
  }
}

// --- RX-2C Protocol Logic (High Precision Timing) ---
void send_rx2c_signal(int cmd)
{
  if (cmd <= 0)
    return;

  // Enter Critical Section: Disables interrupts on this core to prevent jitter
  portENTER_CRITICAL(&timerMux);

  // 1. Send 4 W2 (Start/Sync) Pulses
  // W2 = 1.5ms (1500us) High, 0.5ms (500us) Low
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(SIGNAL_PIN, HIGH);
    ets_delay_us(1500);
    digitalWrite(SIGNAL_PIN, LOW);
    ets_delay_us(500);
  }

  // 2. Send 'n' W1 (Data) Pulses
  // W1 = 0.5ms (500us) High, 0.5ms (500us) Low
  for (int i = 0; i < cmd; i++)
  {
    digitalWrite(SIGNAL_PIN, HIGH);
    ets_delay_us(500);
    digitalWrite(SIGNAL_PIN, LOW);
    ets_delay_us(500);
  }

  // Exit Critical Section
  portEXIT_CRITICAL(&timerMux);

  // 3. Dead Time (Blanking) - Crucial for RX-2C to latch the count
  // This part does NOT need a critical section.
  vTaskDelay(pdMS_TO_TICKS(15));
}

// --- CORE 1 TASK: High-Speed Signal Generation ---
void signalTask(void *pvParameters)
{
  pinMode(SIGNAL_PIN, OUTPUT);
  digitalWrite(SIGNAL_PIN, LOW);

  for (;;)
  {
    if (current_cmd > 0)
    {
      send_rx2c_signal(current_cmd);
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(50)); // Idle if no command
    }
  }
}

// --- CORE 0 TASK: WiFi and MQTT Management ---
void networkTask(void *pvParameters)
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  for (;;)
  {
    if (!client.connected())
    {
      Serial.print("Connecting to MQTT...");
      String clientId = "ESP32_Car_Client_" + String(random(0xffff), HEX);
      if (client.connect(clientId.c_str()))
      {
        Serial.println("Connected!");
        client.subscribe(mqtt_topic);
      }
      else
      {
        Serial.print("Failed, rc=");
        Serial.println(client.state());
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield for background WiFi tasks
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(STATUS_LED, OUTPUT);

  // Connect to WiFi
  Serial.printf("Connecting to %s", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  }
  Serial.println("\nWiFi Connected!");
  digitalWrite(STATUS_LED, HIGH);

  // Create tasks on separate cores
  // Priority 5 (Network) on Core 0
  xTaskCreatePinnedToCore(networkTask, "Network", 4096, NULL, 5, NULL, 0);

  // Priority 4 (Signal) on Core 1
  xTaskCreatePinnedToCore(signalTask, "Signal", 4096, NULL, 4, NULL, 1);
}

void loop()
{
  // Keep main thread alive
  vTaskDelay(pdMS_TO_TICKS(1000));
}