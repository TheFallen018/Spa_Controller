//================================================================================
// Simple Spa Controller for ESP32 (Two-Relay Version)
//
// Controls a circulation pump and a heater via MQTT.
// Ensures the circulation pump is always active when the heater is on.
// Provides a web interface for status and OTA updates.
//================================================================================

// Required Libraries:
// https://arduinojson.org/
// http://pubsubclient.knolleary.net/
// https://github.com/ayushsharma82/ElegantOTA
// WebServer is part of the ESP32 core/libraries

#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>

//--- Pin Definitions ---
const int CIRCULATION_PUMP_PIN = 13;
const int HEATER_PIN = 12;

//--- WiFi & Web Server Configuration ---
const char* WIFI_SSID = "2.4G-Tower";
const char* WIFI_PASSWORD = "Network_Layer";
const char* HOSTNAME = "simple-spa-controller";
WebServer server(80);

//--- MQTT Configuration ---
const char* MQTT_BROKER = "192.168.0.180";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt_user";
const char* MQTT_PASSWORD = "mqtt_user_password_#1";
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT Topics
const char* MQTT_AVAILABILITY_TOPIC = "zado_spa/available";
const char* MQTT_CIRCULATION_PUMP_STATE_TOPIC = "zado_spa/circulation_pump/state";
const char* MQTT_CIRCULATION_PUMP_SET_TOPIC = "zado_spa/circulation_pump/set";
const char* MQTT_HEATER_STATE_TOPIC = "zado_spa/heater/state";
const char* MQTT_HEATER_SET_TOPIC = "zado_spa/heater/set";
const char* MQTT_DEBUG_TOPIC = "zado_spa/debug";

//--- Controller State Variables ---
bool circulation_pump_state = false;
bool heater_state = false;

//--- Timing Control (Non-Blocking) ---
unsigned long last_mqtt_publish_time = 0;
unsigned long last_reconnect_attempt_time = 0;
const long MQTT_PUBLISH_INTERVAL = 60000;  // Publish availability every 60 seconds
const long RECONNECT_INTERVAL = 30000;     // Attempt reconnect every 30 seconds

//================================================================================
//                             FORWARD DECLARATIONS
//================================================================================
void setup_wifi();
void handle_connections();
void setup_mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void update_relays_and_publish_states();
const char* bool_to_on_off(bool state);
bool is_payload_on(const byte* payload, unsigned int length);

//================================================================================
//                                  SETUP
//================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[INFO] Booting Simple Spa Controller...");

  //--- Initialize GPIO Pins ---
  pinMode(CIRCULATION_PUMP_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(CIRCULATION_PUMP_PIN, LOW);
  digitalWrite(HEATER_PIN, LOW);

  //--- Initialize Network & Services ---
  setup_wifi();
  setup_mqtt();

  //--- Initialize Web Server & OTA ---
  ElegantOTA.begin(&server);
  server.on("/", HTTP_GET, []() {
    String html = "<html><head><title>ESP32 Simple Spa Controller</title>";
    html += "<meta http-equiv='refresh' content='5'>"; // Auto-refresh every 5 seconds
    html += "<style>"
            "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; }"
            "h1 { color: #0056b3; }"
            "a { color: #007bff; text-decoration: none; font-size: 1.2em; }"
            "a:hover { text-decoration: underline; }"
            "table { width: 60%; border-collapse: collapse; margin-top: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
            "th, td { padding: 12px; border: 1px solid #ddd; text-align: left; }"
            "th { background-color: #007bff; color: white; }"
            ".state-on, .state-high { color: #28a745; font-weight: bold; }"
            ".state-off, .state-low { color: #dc3545; font-weight: bold; }"
            "</style></head><body>";
    html += "<h1>ESP32 Simple Spa Controller Status</h1>";
    html += "<p><a href='/update'>» Go to Firmware Update Page</a></p>";
    html += "<h2>Controller States</h2><table>";
    html += "<tr><th>Component</th><th>Requested State</th></tr>";
    html += "<tr><td>Circulation Pump</td><td><span class='state-" + String(circulation_pump_state ? "on" : "off") + "'>" + bool_to_on_off(circulation_pump_state) + "</span></td></tr>";
    html += "<tr><td>Heater</td><td><span class='state-" + String(heater_state ? "on" : "off") + "'>" + bool_to_on_off(heater_state) + "</span></td></tr>";
    html += "</table>";
    html += "<h2>GPIO Output States</h2><table>";
    html += "<tr><th>Device</th><th>GPIO Pin</th><th>Actual State</th></tr>";
    html += "<tr><td>CIRCULATION PUMP</td><td>" + String(CIRCULATION_PUMP_PIN) + "</td><td><span class='state-" + String(digitalRead(CIRCULATION_PUMP_PIN) ? "high" : "low") + "'>" + String(digitalRead(CIRCULATION_PUMP_PIN) ? "HIGH" : "LOW") + "</span></td></tr>";
    html += "<tr><td>HEATER</td><td>" + String(HEATER_PIN) + "</td><td><span class='state-" + String(digitalRead(HEATER_PIN) ? "high" : "low") + "'>" + String(digitalRead(HEATER_PIN) ? "HIGH" : "LOW") + "</span></td></tr>";
    html += "</table></body></html>";
    server.send(200, "text/html", html);
  });
  server.begin();
  Serial.println("[INFO] HTTP server started.");
}

//================================================================================
//                                 MAIN LOOP
//================================================================================
void loop() {
  unsigned long current_time = millis();

  handle_connections();
  
  if (client.connected()) {
    client.loop(); 

    if (current_time - last_mqtt_publish_time > MQTT_PUBLISH_INTERVAL) {
      last_mqtt_publish_time = current_time;
      client.publish(MQTT_AVAILABILITY_TOPIC, "online", true); // Publish with retain flag
      update_relays_and_publish_states(); // Periodically publish states
    }
  }
  
  server.handleClient();
  ElegantOTA.loop();
}

//================================================================================
//                            NETWORK FUNCTIONS
//================================================================================
void setup_wifi() {
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[INFO] Connecting to WiFi...");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[INFO] WiFi connected!");
    Serial.print("[INFO] IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[ERROR] WiFi connection failed. Please check credentials or network.");
  }
}

void setup_mqtt() {
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(mqtt_callback);
}

void handle_connections() {
  if (WiFi.status() != WL_CONNECTED) {
    // Non-blocking WiFi reconnect is handled internally by the ESP32 core
    // after the initial connection attempt. We just log it.
    if (millis() - last_reconnect_attempt_time > RECONNECT_INTERVAL) {
        last_reconnect_attempt_time = millis();
        Serial.println("[WARN] WiFi disconnected. ESP32 will attempt to reconnect automatically.");
    }
    return;
  }

  if (!client.connected()) {
    if (millis() - last_reconnect_attempt_time > RECONNECT_INTERVAL) {
      last_reconnect_attempt_time = millis();
      Serial.println("[WARN] MQTT disconnected. Attempting to reconnect...");
      String client_id = "simple-spa-client-";
      client_id += WiFi.macAddress();
      if (client.connect(client_id.c_str(), MQTT_USER, MQTT_PASSWORD, MQTT_AVAILABILITY_TOPIC, 0, true, "offline")) {
        Serial.println("[INFO] MQTT reconnected successfully!");
        client.publish(MQTT_AVAILABILITY_TOPIC, "online", true);
        // Resubscribe to topics upon reconnection
        client.subscribe(MQTT_CIRCULATION_PUMP_SET_TOPIC);
        client.subscribe(MQTT_HEATER_SET_TOPIC);
        // Publish current state to sync with Home Assistant or other clients
        update_relays_and_publish_states();
      } else {
        Serial.print("[ERROR] MQTT reconnection failed, rc=");
        Serial.println(client.state());
      }
    }
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Message arrived on topic: ");
  Serial.println(topic);

  // Convert payload to a string for easy comparison
  char payload_str[length + 1];
  memcpy(payload_str, payload, length);
  payload_str[length] = '\0';
  Serial.print("[MQTT] Payload: ");
  Serial.println(payload_str);

  bool is_on = is_payload_on(payload, length);

  if (strcmp(topic, MQTT_CIRCULATION_PUMP_SET_TOPIC) == 0) {
    circulation_pump_state = is_on;
  } else if (strcmp(topic, MQTT_HEATER_SET_TOPIC) == 0) {
    heater_state = is_on;
  }
  
  // Apply the new state immediately
  update_relays_and_publish_states();
}


//================================================================================
//                       RELAY LOGIC & STATE PUBLISHING
//================================================================================
void update_relays_and_publish_states() {
  bool final_pump_state;
  bool final_heater_state;

  // --- Core Safety Logic ---
  // 1. If the heater is requested to be ON, the pump MUST be ON.
  if (heater_state) {
    final_pump_state = true;
    final_heater_state = true;
  } else {
    // 2. If the heater is OFF, the pump can be on or off as requested.
    final_pump_state = circulation_pump_state;
    final_heater_state = false;
  }

  // --- Set GPIOs based on the final logic ---
  digitalWrite(CIRCULATION_PUMP_PIN, final_pump_state ? HIGH : LOW);
  digitalWrite(HEATER_PIN, final_heater_state ? HIGH : LOW);

  // --- Publish the actual states to MQTT ---
  // This tells Home Assistant what is really happening.
  client.publish(MQTT_CIRCULATION_PUMP_STATE_TOPIC, bool_to_on_off(final_pump_state), true);
  client.publish(MQTT_HEATER_STATE_TOPIC, bool_to_on_off(final_heater_state), true);

  // --- Print current status to Serial Monitor for debugging ---
  Serial.println("=== Spa Status Updated ===");
  Serial.printf("Requested States -> Pump: %s, Heater: %s\n", bool_to_on_off(circulation_pump_state), bool_to_on_off(heater_state));
  Serial.printf("Actual GPIO States -> Pump (%d): %s, Heater (%d): %s\n",
                CIRCULATION_PUMP_PIN, digitalRead(CIRCULATION_PUMP_PIN) ? "HIGH" : "LOW",
                HEATER_PIN, digitalRead(HEATER_PIN) ? "HIGH" : "LOW");
  Serial.println("==========================");
}


//================================================================================
//                               UTILITY FUNCTIONS
//================================================================================
const char* bool_to_on_off(bool state) {
  return state ? "ON" : "OFF";
}

bool is_payload_on(const byte* payload, unsigned int length) {
  // Case-insensitive check for "ON"
  if (length == 2 && toupper(payload[0]) == 'O' && toupper(payload[1]) == 'N') {
    return true;
  }
  return false;
}