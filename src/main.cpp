#include <Arduino.h>
#include <string>
// UHF
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

// WIFI
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

// MQTT
#include <PubSubClient.h>
#define MSG_BUFFER_SIZE (50)
// Update these with values suitable for your network.

// version
#define MAYOR 1
#define MINOR 0
#define PATCH 0
String version = String(MAYOR) + "." + String(MINOR) + "." + String(PATCH);
// WIFI PASSWORD
#define WIFI_SSID "MIWIFI_zmiz"
#define WIFI_PASS "3wRdPJut"
//  put function declarations here:
// LED DISPLAY
const int led1 = 2; // Pin of the LED
const int D15 = 15; // output car control
const int D4 = 4;   // RX output car control
const int D16 = 16; // TX output car control
const int D17 = 17; // PPT output car control
bool state = 1;
bool state2 = 1;
// MAIN LOOP
volatile uint32_t lastMillis = 0;

// MQTT
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
std::string device = "car_mod";
std::string TOPIC_IP = "home/" + device;
const char *mqtt_server = "192.168.1.2";
const char *TOPIC = TOPIC_IP.c_str();
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
// MQTT
void callback_mqtt(char *topic, byte *payload, unsigned int length);
void reconnect_mqtt();

// APP
#define END_CODE 4
#define FORWARD 10
// #define FORWARD 16
#define TURBO 22
#define FORWARD_LEFT 28
#define FORWARD_RIGHT 34
#define BACKWARD 40
#define BACKWARD_RIGHT 46
#define BACKWARD_LEFT 52
#define LEFT 58
#define RIGHT 64
uint8_t commands_test[] = {FORWARD, FORWARD_LEFT, FORWARD_RIGHT, BACKWARD, BACKWARD_RIGHT, BACKWARD_LEFT, LEFT, RIGHT};
uint8_t ite = 0;
uint8_t cmd = 0;
unsigned long lastMicro = 0; // main timer
unsigned long lastMicro_send = 0;
bool forward = true;
bool w1 = 1; // Initialize w1
enum State
{
  IDLE,
  W2_STEP1,
  W2_STEP2,
  W2_STEP3,
  W2_STEP4,
  W1_STEP
};
State currentState = IDLE;
size_t w2_counter = 0;
size_t w1_counter = 0;
uint8_t w1_code = 0; // cmd goes in here
uint8_t end_code_cnt = 0;

bool run_once_w21 = 1; // not used
bool run_once_w22 = 1; // not used
bool run_once_w23 = 1; // not used
bool run_once_w24 = 1; // not used
bool run_once_w11 = 1; // not used
bool run_once_w12 = 1; // not used

void mqtt_service(void *parameter);
void wait_us(uint64_t num);

// APP
uint8_t uhf_buf[12];
uint8_t buflen = sizeof(uhf_buf);
RH_ASK driver(9600, D4, D16, D17);
bool recv_flag = false;
void UHF_recv(void *parameter);

void setup()
{
  pinMode(led1, OUTPUT);
  pinMode(D4, OUTPUT);  // RX
  pinMode(D17, OUTPUT); // TX
  pinMode(D15, OUTPUT); // PPT
  Serial.begin(115200);

  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.println("ESP 32 WROOM");

  // WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // change it to your ussid and password
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
  }
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  // MULTUTASK
  xTaskCreate(mqtt_service, "MQTT Service", 2000, NULL, 1, NULL);

  xTaskCreate(UHF_recv, "UHF Receiver", 2000, NULL, 10, NULL);
}

void loop()
{
  //-----------------------
  for (size_t i = 0; i < 4; i++)
  {
    digitalWrite(D15, 0);
    wait_us(400);
    wait_us(400);
    wait_us(400);
    digitalWrite(D15, 1);
    wait_us(620);
    digitalWrite(D15, 0); // next w1
    wait_us(320);
  }
  uint8_t code = ((cmd * 2));

  for (size_t i = 0; i < code; i++)
  {
    digitalWrite(D15, w1);
    if (i < (code - 1) && w1 == 1)
      wait_us(620);
    if (i == (code - 1) && w1 == 1)
    {
      wait_us(620);
      break;
    }
    if (w1 == 0)
      wait_us(320);
    w1 = !w1;
  }
}
// ENDOFCODE
//------------------------
// UHF
void UHF_recv(void *parameter)
{
  if (run_once_w11)
  {
    Serial.println("ENTER UHF");
    run_once_w11 = 0;
  }
  for (;;)
  {
    recv_flag = driver.recv(uhf_buf, &buflen);
    if (recv_flag) // Non-blocking
    {
      // Message with a good checksum received, dump it.
      Serial.print("Message: ");
      Serial.println((char *)uhf_buf);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    yield();
  }
}
// MQTT
void mqtt_service(void *parameter)
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback_mqtt);
  for (;;)
  {

    if (!client.connected())
    {
      reconnect_mqtt();
    }
    else
    {
      client.loop();

      if (millis() - lastMillis >= 10000)
      {
        lastMillis = millis();

        /*cmd = commands_test[ite];
        // cmd = 10;
        ite++;
        if (ite >= sizeof(commands_test))
          ite = 0;
        Serial.printf("CMD: %d\n", cmd);
        */
        digitalWrite(led1, state);
        state = !state;
        // Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
        snprintf(msg, MSG_BUFFER_SIZE, "%s:%s", device.c_str(), WiFi.localIP().toString().c_str());
        client.publish(TOPIC_IP.c_str(), msg);
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    yield();
  }
}

void wait_us(uint64_t number_of_microseconds)
{
  uint64_t microseconds = esp_timer_get_time();
  if (0 != number_of_microseconds)
  {
    while (((uint64_t)esp_timer_get_time() - microseconds) <=
           number_of_microseconds)
    {
      // Wait
    }
  }
}
// MQTT
void callback_mqtt(char *topic, byte *payload, unsigned int length)
{

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.printf("Len:  %d msg: ", length);
  for (int i = 0; i < length; i++)
  {
    Serial.printf("%02x", payload[i]);
  }
  Serial.println();

  if (strcmp(topic, TOPIC) == 0 && (char)payload[0] != NULL)
  {
    if (length <= 2)
    {
      cmd = atoi((char *)payload);
      Serial.printf("CMD: %d \n", cmd);
    }
    else
    {
      // Serial.printf("Error on conversion MQTT msg \n");
    }
  }
}

void reconnect_mqtt()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(TOPIC, "car ctrl uhf");
      // ... and resubscribe
      client.subscribe(TOPIC);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}