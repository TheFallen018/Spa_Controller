//================================================================================
// Spa Controller for ESP32 (Corrected Version)
//
// A WiFi-enabled spa controller that integrates with Home Assistant via MQTT.
// It manages a pump, two heaters, and a blower based on temperature readings
// and MQTT commands. It also provides a web interface for status monitoring
// and Over-the-Air (OTA) firmware updates.
//================================================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <CircularBuffer.hpp> // NOTE: This is no longer used for logic, but kept for potential future use.
#include <WebServer.h>
#include <ElegantOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

#include "lut.h" // Assumes lut.h contains the ADC_LUT array

//--- Pin Definitions ---
const int PUMP_PIN = 13;
const int EXTERNAL_HEATER_PIN = 26;
const int INTERNAL_HEATER_PIN = 12;
const int BLOWER_PIN = 14;
const int LED_PIN = 27; // Unused in logic, but defined
const int THERMISTOR_PIN = 35;
const int ONBOARD_LED_PIN = 2;


//--- Thermistor Configuration ---
const int THERMISTORNOMINAL = 84000;      // Resistance at 25°C
const int TEMPERATURENOMINAL = 25;        // Temperature for nominal resistance
const int BCOEFFICIENT = 4193;            // Beta coefficient of the thermistor
const int SERIESRESISTOR = 10000;         // Value of the series resistor
const int NUMSAMPLES = 10;                // Number of samples to average for a reading

//--- WiFi & Web Server Configuration ---
//--- Access Point (AP) Configuration for Setup ---
const char* AP_SSID = "ESP32-Spa-Setup";
#define EEPROM_SIZE 256

const char* HOSTNAME = "spa-controller";
WebServer server(80);

// --- Factory Reset Configuration ---
const int RESET_EEPROM_ADDR = EEPROM_SIZE - 1;
const int RESET_CYCLE_TARGET = 5;
const unsigned long SUCCESSFUL_BOOT_TIMEOUT_MS = 20000;

// --- Global variables ---
unsigned long boot_time = 0;
bool reset_counter_cleared = false;
unsigned long last_wifi_reconnect_attempt = 0;
unsigned long last_time_debug_print = 0;

struct WifiConfig {
  char ssid[33] = "";
  char password[65] = "";
};

struct ControllerConfig {
  uint32_t magic_marker = 0xDEADBEEF;
  WifiConfig wifi;
};

ControllerConfig controller_config;



//--- MQTT Configuration ---
const char* MQTT_BROKER = "192.168.0.180";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt_user";
const char* MQTT_PASSWORD = "mqtt_user_password_#1";
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT Topics
const char* MQTT_SPA_AVAILABLE = "spa/available";
const char* MQTT_SPA_STATE_TOPIC = "spa/state";
const char* MQTT_SPA_SET_TOPIC = "spa/set";
const char* MQTT_STANDBY_WARMING_STATE_TOPIC = "spa/standby_warming/state";
const char* MQTT_STANDBY_WARMING_SET_TOPIC = "spa/standby_warming/set";
const char* MQTT_FAST_HEATING_STATE_TOPIC = "spa/fast_heating/state";
const char* MQTT_FAST_HEATING_SET_TOPIC = "spa/fast_heating/set";
const char* MQTT_CIRCULATION_ONLY_STATE_TOPIC = "spa/circulation_only/state";
const char* MQTT_CIRCULATION_ONLY_SET_TOPIC = "spa/circulation_only/set";
const char* MQTT_BLOWER_STATE_TOPIC = "spa/blower/state";
const char* MQTT_BLOWER_SET_TOPIC = "spa/blower/set";
const char* MQTT_TEMPERATURE_STATE_TOPIC = "spa/temperature/state";
const char* MQTT_TEMPERATURE_SET_TOPIC = "spa/temperature/set";
const char* MQTT_DEBUG_TOPIC = "spa/debug";

//--- Controller Logic & State Variables ---
bool spa_state = false;
bool standby_warming_state = false;
bool fast_heating_state = false;
bool blower_state = false;
bool circulation_only_state = false;
float temperature_target = 43.0;
float temperature_tolerance = 0.2;
float standby_tolerance = 2.0;
float standby_temp_delta = 2.0;
float temperature = 0.0;
float voltage_read = 0.0;

// Exponential Moving Average alpha for temperature smoothing. Lower value = more smoothing.
const float EMA_ALPHA = 0.2; 

//--- Global Objects & Buffers ---
JsonDocument doc;
char msg_buffer[256]; // General purpose buffer for MQTT messages etc.

//--- Timing Control (Non-Blocking) ---
unsigned long last_temp_read_time = 0;
unsigned long last_mqtt_publish_time = 0;
unsigned long last_reconnect_attempt_time = 0;
const long TEMP_READ_INTERVAL = 5000;      // Read temperature every 5 seconds
const long MQTT_PUBLISH_INTERVAL = 60000;  // Publish availability every 60 seconds
const long RECONNECT_INTERVAL = 30000;     // Attempt reconnect every 30 seconds if disconnected
bool in_config_mode = false;

//================================================================================
//                             FORWARD DECLARATIONS
//================================================================================
void setup_wifi();
void handle_connections();
void setup_mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
float get_temp();
void update_states();
const char* bool_to_on_off(bool state);
bool is_payload_on(const byte* payload, unsigned int length);
void handle_config_root();
void handle_save_wifi();
void load_config_from_eeprom();
void save_config_to_eeprom();
void print_config();
void clear_eeprom_and_restart();
bool connect_to_wifi();
void setup_ap_mode();

//================================================================================
//                                  SETUP
//================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[INFO] Booting Spa Controller...");

  EEPROM.begin(EEPROM_SIZE);

  byte boot_counter = EEPROM.read(RESET_EEPROM_ADDR);
  Serial.printf("[RESET] Boot counter is: %d\n", boot_counter);
  boot_counter++;

  if (boot_counter >= RESET_CYCLE_TARGET) {
    Serial.printf("[RESET] Reached target of %d boots. Wiping EEPROM.\n", RESET_CYCLE_TARGET);
    clear_eeprom_and_restart();
  }

  EEPROM.write(RESET_EEPROM_ADDR, boot_counter);
  EEPROM.commit();
  boot_time = millis();

  //--- Initialize GPIO Pins ---
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(EXTERNAL_HEATER_PIN, OUTPUT);
  pinMode(INTERNAL_HEATER_PIN, OUTPUT);
  pinMode(BLOWER_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(EXTERNAL_HEATER_PIN, LOW);
  digitalWrite(INTERNAL_HEATER_PIN, LOW);
  digitalWrite(BLOWER_PIN, LOW);

  //--- Initialize Network & Services ---
  load_config_from_eeprom();

  if (!connect_to_wifi()) {
    in_config_mode = true;
    setup_ap_mode();
  } else {
    in_config_mode = false;
  

    setup_mqtt();
    
    //--- Initialize Web Server & OTA ---
    ElegantOTA.begin(&server);
    server.on("/", HTTP_GET, []() {
      String html = "<html><head><title>ESP32 Spa Controller</title>";
      html += "<meta http-equiv='refresh' content='5'>"; // Auto-refresh every 5 seconds
      html += "<style>"
              "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; }"
              "h1, h2 { color: #0056b3; }"
              "a { color: #007bff; text-decoration: none; font-size: 1.2em; }"
              "a:hover { text-decoration: underline; }"
              "table { width: 60%; border-collapse: collapse; margin-top: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
              "th, td { padding: 12px; border: 1px solid #ddd; text-align: left; }"
              "th { background-color: #007bff; color: white; }"
              ".state-on, .state-high { color: #28a745; font-weight: bold; }"
              ".state-off, .state-low { color: #dc3545; font-weight: bold; }"
              "</style></head><body>";
      html += "<h1>ESP32 Spa Controller Status</h1>";
      html += "<p><a href='/update'>» Go to Firmware Update Page</a></p>";
      html += "<h2>Controller States</h2><table>";
      html += "<tr><th>Parameter</th><th>State</th></tr>";
      html += "<tr><td>Spa Power</td><td><span class='state-" + String(spa_state ? "on" : "off") + "'>" + bool_to_on_off(spa_state) + "</span></td></tr>";
      html += "<tr><td>Standby Warming</td><td><span class='state-" + String(standby_warming_state ? "on" : "off") + "'>" + bool_to_on_off(standby_warming_state) + "</span></td></tr>";
      html += "<tr><td>Fast Heating</td><td><span class='state-" + String(fast_heating_state ? "on" : "off") + "'>" + bool_to_on_off(fast_heating_state) + "</span></td></tr>";
      // FIX: Added circulation only state to the web page
      html += "<tr><td>Circulation Only</td><td><span class='state-" + String(circulation_only_state ? "on" : "off") + "'>" + bool_to_on_off(circulation_only_state) + "</span></td></tr>";
      html += "<tr><td>Blower</td><td><span class='state-" + String(blower_state ? "on" : "off") + "'>" + bool_to_on_off(blower_state) + "</span></td></tr>";
      html += "</table>";
      html += "<h2>Temperature</h2><table>";
      html += "<tr><th>Parameter</th><th>Value</th></tr>";
      html += "<tr><td>Current Temperature</td><td>" + String(temperature, 2) + "°C</td></tr>";
      html += "<tr><td>Voltage Read</td><td>" + String(voltage_read, 2) + "</td></tr>";
      html += "<tr><td>Target Temperature</td><td>" + String(temperature_target, 2) + "°C</td></tr>";
      html += "</table>";
      html += "<h2>GPIO Output States</h2><table>";
      html += "<tr><th>Device</th><th>GPIO Pin</th><th>State</th></tr>";
      html += "<tr><td>PUMP</td><td>" + String(PUMP_PIN) + "</td><td><span class='state-" + String(digitalRead(PUMP_PIN) ? "high" : "low") + "'>" + String(digitalRead(PUMP_PIN) ? "HIGH" : "LOW") + "</span></td></tr>";
      html += "<tr><td>EXTERNAL_HEATER</td><td>" + String(EXTERNAL_HEATER_PIN) + "</td><td><span class='state-" + String(digitalRead(EXTERNAL_HEATER_PIN) ? "high" : "low") + "'>" + String(digitalRead(EXTERNAL_HEATER_PIN) ? "HIGH" : "LOW") + "</span></td></tr>";
      html += "<tr><td>INTERNAL_HEATER</td><td>" + String(INTERNAL_HEATER_PIN) + "</td><td><span class='state-" + String(digitalRead(INTERNAL_HEATER_PIN) ? "high" : "low") + "'>" + String(digitalRead(INTERNAL_HEATER_PIN) ? "HIGH" : "LOW") + "</span></td></tr>";
      html += "<tr><td>BLOWER</td><td>" + String(BLOWER_PIN) + "</td><td><span class='state-" + String(digitalRead(BLOWER_PIN) ? "high" : "low") + "'>" + String(digitalRead(BLOWER_PIN) ? "HIGH" : "LOW") + "</span></td></tr>";
      html += "</table></body></html>";
      server.send(200, "text/html", html);
    });
    server.begin();
    Serial.println("[INFO] HTTP server started.");
  }
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
      client.publish(MQTT_SPA_AVAILABLE, "online", true); // Publish with retain flag
      update_states(); 
    }

    if (current_time - last_temp_read_time > TEMP_READ_INTERVAL) {
      last_temp_read_time = current_time;
      float new_temp = get_temp();

      if (!isnan(new_temp)) {
        // Apply EMA smoothing
        if (temperature == 0.0) { // Initial reading
            temperature = new_temp;
        } else {
            temperature = (temperature * (1.0 - EMA_ALPHA)) + (new_temp * EMA_ALPHA);
        }
        
        // Publish temperature state
        snprintf(msg_buffer, sizeof(msg_buffer), "%.2f", temperature);
        client.publish(MQTT_TEMPERATURE_STATE_TOPIC, msg_buffer);
        
        // After getting a new temperature, re-evaluate the system state
        update_states();
      } else {
        Serial.println("[ERROR] Invalid temperature reading from sensor.");
        client.publish(MQTT_DEBUG_TOPIC, "ERROR: Invalid temp reading");
      }
    }
  }
  
  server.handleClient();
  ElegantOTA.loop();
}


//================================================================================
//                            NETWORK FUNCTIONS
//================================================================================

bool connect_to_wifi() {
  if (String(controller_config.wifi.ssid).length() == 0) return false;

  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(controller_config.wifi.ssid, controller_config.wifi.password);
  Serial.print("[INFO] Connecting to WiFi: ");
  Serial.print(controller_config.wifi.ssid);
  
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[INFO] WiFi connected!");
    Serial.print("[INFO] IP Address: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin(HOSTNAME)) {
      Serial.printf("[mDNS] Responder started. Access at http://%s.local\n", HOSTNAME);
      MDNS.addService("http", "tcp", 80);
    } else {
      Serial.println("[mDNS] Error setting up MDNS responder!");
    }
    return true;
  } else {
    Serial.println("\n[ERROR] WiFi connection failed.");
    return false;
  }
}

void handle_config_root() {
    String html = R"rawliteral(
<!DOCTYPE HTML><html><head>
<title>ESP32 WiFi Setup</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style> body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; text-align: center; } h1 { color: #0056b3; } .container { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); display: inline-block; } input[type=text], input[type=password] { width: 90%; padding: 12px; margin: 8px 0; display: inline-block; border: 1px solid #ccc; border-radius: 4px; } input[type=submit] { background-color: #007bff; color: white; padding: 14px 20px; margin: 8px 0; border: none; border-radius: 4px; cursor: pointer; width: 95%; } input[type=submit]:hover { background-color: #0056b3; } </style>
</head><body><div class="container"><h1>Spa Controller WiFi Setup</h1><p>Connect the ESP32 to your local WiFi network.</p>
<form action="/savewifi" method="post"><label for="ssid">WiFi Network Name (SSID)</label><input type="text" id="ssid" name="ssid" placeholder="Your network name"><label for="pass">Password</label><input type="password" id="pass" name="password" placeholder="Your network password"><input type="submit" value="Save and Restart"></form>
</div></body></html>)rawliteral";
  server.send(200, "text/html", html);
}

void setup_ap_mode() {
  Serial.println("[INFO] Starting Access Point mode for configuration.");
  WiFi.softAP(AP_SSID);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("[INFO] AP IP address: ");
  Serial.println(ip);
  server.on("/", HTTP_GET, handle_config_root);
  server.on("/savewifi", HTTP_POST, handle_save_wifi);
  server.begin();
  Serial.println("[INFO] HTTP server started for configuration.");
}

void setup_mqtt() {
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(mqtt_callback);
}

void handle_connections() {
  if (in_config_mode) {
    return; 
  }
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - last_reconnect_attempt_time > RECONNECT_INTERVAL) {
      last_reconnect_attempt_time = millis();
      Serial.println("[WARN] WiFi disconnected. Attempting to reconnect...");
      WiFi.reconnect();
    }
    return;
  }

  if (!client.connected()) {
    if (millis() - last_reconnect_attempt_time > RECONNECT_INTERVAL) {
      last_reconnect_attempt_time = millis();
      Serial.println("[WARN] MQTT disconnected. Attempting to reconnect...");
      String client_id = "spa-controller-client-";
      client_id += WiFi.macAddress();
      if (client.connect(client_id.c_str(), MQTT_USER, MQTT_PASSWORD, MQTT_SPA_AVAILABLE, 0, true, "offline")) {
        Serial.println("[INFO] MQTT reconnected successfully!");
        client.publish(MQTT_SPA_AVAILABLE, "online", true);
        // Resubscribe to topics upon reconnection
        client.subscribe(MQTT_SPA_SET_TOPIC);
        client.subscribe(MQTT_BLOWER_SET_TOPIC);
        client.subscribe(MQTT_FAST_HEATING_SET_TOPIC);
        client.subscribe(MQTT_STANDBY_WARMING_SET_TOPIC);
        client.subscribe(MQTT_TEMPERATURE_SET_TOPIC);
        // FIX: Added missing semicolon and ensured subscription
        client.subscribe(MQTT_CIRCULATION_ONLY_SET_TOPIC);
        // Publish current state to sync with HA
        update_states();
      } else {
        Serial.print("[ERROR] MQTT reconnection failed, rc=");
        Serial.println(client.state());
      }
    }
  }
}

void load_config_from_eeprom() {
  EEPROM.get(0, controller_config);
  Serial.println("[DEBUG] Just loaded the following from EEPROM:");
  print_config();
  if (controller_config.magic_marker != 0xDEADBEEF) {
    Serial.println("[WARN] EEPROM not initialized. Loading defaults.");
    controller_config.magic_marker = 0xDEADBEEF;
    strcpy(controller_config.wifi.ssid, "");
    strcpy(controller_config.wifi.password, "");

    save_config_to_eeprom();
  } else {
    Serial.println("[INFO] Successfully loaded configuration from EEPROM.");
  }
}

void save_config_to_eeprom() {
  EEPROM.put(0, controller_config);
  if (EEPROM.commit()) {
    Serial.println("[INFO] Configuration saved to EEPROM successfully.");
    Serial.println("[DEBUG] Just saved the following to EEPROM:");
    print_config();
  } else {
    Serial.println("[ERROR] Failed to save configuration to EEPROM.");
  }
}

void print_config() {
  Serial.println("--- DUMPING CURRENT CONFIGURATION ---");
  Serial.printf("Magic Marker: 0x%X\n", controller_config.magic_marker);
  Serial.printf("WiFi SSID: '%s'\n", controller_config.wifi.ssid);

  Serial.println("------------------------------------");
}

void clear_eeprom_and_restart() {
  Serial.println("[RESET] Factory reset triggered!");
  for (int i = 0; i < 10; i++) {
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    delay(50);
    digitalWrite(ONBOARD_LED_PIN, LOW);
    delay(50);
  }
  Serial.println("[RESET] Clearing EEPROM...");
  for (int i = 0; i < EEPROM_SIZE; i++) { 
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  Serial.println("[RESET] EEPROM cleared. Restarting device.");
  delay(1000);
  ESP.restart();
}

void handle_save_wifi() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  strncpy(controller_config.wifi.ssid, ssid.c_str(), sizeof(controller_config.wifi.ssid));
  controller_config.wifi.ssid[sizeof(controller_config.wifi.ssid) - 1] = '\0';
  strncpy(controller_config.wifi.password, password.c_str(), sizeof(controller_config.wifi.password));
  controller_config.wifi.password[sizeof(controller_config.wifi.password) - 1] = '\0';
  save_config_to_eeprom();
  String html = "<html><body><h1>Credentials Saved!</h1><p>The device will now restart.</p></body></html>";
  server.send(200, "text/html", html);
  delay(2000);
  ESP.restart();
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Message arrived on topic: ");
  Serial.println(topic);

  char payload_buffer[length + 1];
  memcpy(payload_buffer, payload, length);
  payload_buffer[length] = '\0';
  Serial.print("[MQTT] Payload: ");
  Serial.println(payload_buffer);

  bool is_on = is_payload_on(payload, length);

  if (strcmp(topic, MQTT_SPA_SET_TOPIC) == 0) {
    spa_state = is_on;
  } else if (strcmp(topic, MQTT_FAST_HEATING_SET_TOPIC) == 0) {
    fast_heating_state = is_on;
  } else if (strcmp(topic, MQTT_STANDBY_WARMING_SET_TOPIC) == 0) {
    standby_warming_state = is_on;
  } else if (strcmp(topic, MQTT_BLOWER_SET_TOPIC) == 0) {
    blower_state = is_on;
  // FIX: Added the missing handler for the circulation_only state
  } else if (strcmp(topic, MQTT_CIRCULATION_ONLY_SET_TOPIC) == 0) {
    circulation_only_state = is_on;
  } else if (strcmp(topic, MQTT_TEMPERATURE_SET_TOPIC) == 0) {
    if (isDigit(payload_buffer[0]) || (payload_buffer[0] == '-' && isDigit(payload_buffer[1]))) {
        temperature_target = atof(payload_buffer);
        // Also publish the new target back to the state topic to confirm
        client.publish(MQTT_TEMPERATURE_SET_TOPIC, payload_buffer);
    } else {
        Serial.println("[WARN] Received non-numeric temperature target.");
    }
  }
  
  update_states(); // Apply the new state immediately
}


//================================================================================
//                             SENSOR & LOGIC FUNCTIONS
//================================================================================

/**
 * @brief Reads the thermistor and calculates the temperature in Celsius.
 * @return The calculated temperature, or NAN if the reading is invalid.
 */
float get_temp() {
  float avg_adc = 0;
  for (int i = 0; i < NUMSAMPLES; i++) {
    avg_adc += analogRead(THERMISTOR_PIN);
    delay(10);
  }
  avg_adc /= NUMSAMPLES;

  size_t lut_size = sizeof(ADC_LUT) / sizeof(ADC_LUT[0]);
  if (avg_adc < 0 || avg_adc >= lut_size) {
    Serial.printf("[ERROR] ADC reading %.2f is out of LUT bounds (0-%d).\n", avg_adc, lut_size - 1);
    return NAN;
  }

  float linearized_adc = ADC_LUT[int(avg_adc)];
  
  float v_out = linearized_adc / 4095.0 * 3.3;
  // voltage_read = v_out;
  // Serial.println("Voltage read: ");
  Serial.print(v_out);
  if (v_out == 0) return NAN; 
  float thermistor_resistance = SERIESRESISTOR * ((3.3 / v_out) - 1.0);

  float steinhart;
  steinhart = thermistor_resistance / THERMISTORNOMINAL;
  steinhart = log(steinhart);
  steinhart /= BCOEFFICIENT;
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  // FIX: Removed the redundant moving average from the circular buffer.
  // The EMA in loop() is sufficient and more effective.

  return steinhart;
}

/**
 * @brief The core logic function. Sets GPIOs based on current state variables.
 * Also publishes the current states to MQTT.
 */
void update_states() {
  // --- Publish current states to MQTT for HA ---
  client.publish(MQTT_SPA_STATE_TOPIC, bool_to_on_off(spa_state));
  client.publish(MQTT_STANDBY_WARMING_STATE_TOPIC, bool_to_on_off(standby_warming_state));
  client.publish(MQTT_FAST_HEATING_STATE_TOPIC, bool_to_on_off(fast_heating_state));
  client.publish(MQTT_BLOWER_STATE_TOPIC, bool_to_on_off(blower_state));
  // FIX: Changed topic from MQTT_CIRCULATION_ONLY_TOPIC to the correct one
  client.publish(MQTT_CIRCULATION_ONLY_STATE_TOPIC, bool_to_on_off(circulation_only_state));

  // --- Control Logic ---
  if (spa_state) {
    float current_target = temperature_target;
    float current_tolerance = temperature_tolerance;

    if (standby_warming_state) {
      current_target += standby_temp_delta;
      current_tolerance = standby_tolerance;
    }

    // Determine if heating should be enabled at all
    // FIX: If circulation_only is true, heating is NEVER allowed.
    bool heating_needed = (temperature < current_target - current_tolerance) && !circulation_only_state;

    // Pump Logic: Pump should run if heating is needed, or if the spa is on and not in standby.
    // In standby, the pump only runs when heating. Otherwise, it circulates continuously.
    bool pump_on = heating_needed || !standby_warming_state || circulation_only_state;
    digitalWrite(PUMP_PIN, pump_on ? HIGH : LOW);

    // Heater Logic
    if (heating_needed) {
      // External heater is the primary heater
      digitalWrite(EXTERNAL_HEATER_PIN, HIGH);
      // Internal heater is only for fast heating mode
      digitalWrite(INTERNAL_HEATER_PIN, fast_heating_state ? HIGH : LOW);
    } else {
      // Turn off both heaters if temp is OK, too high, or in circulation_only mode
      digitalWrite(EXTERNAL_HEATER_PIN, LOW);
      digitalWrite(INTERNAL_HEATER_PIN, LOW);
    }

    // Blower is independent of temperature
    digitalWrite(BLOWER_PIN, blower_state ? HIGH : LOW);

  } else {
    // Spa is OFF, turn everything off
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(INTERNAL_HEATER_PIN, LOW);
    digitalWrite(EXTERNAL_HEATER_PIN, LOW);
    digitalWrite(BLOWER_PIN, LOW);
  }

  // --- Print current status to Serial Monitor for debugging ---
  Serial.println("=== Current Spa States ===");
  Serial.printf("Spa Power: %s, Standby: %s, Fast Heat: %s, Blower: %s, Circ Only: %s\n", 
                bool_to_on_off(spa_state), bool_to_on_off(standby_warming_state), 
                bool_to_on_off(fast_heating_state), bool_to_on_off(blower_state), bool_to_on_off(circulation_only_state));
  Serial.printf("Temp: %.2f C, Target: %.2f C\n", temperature, temperature_target);
  Serial.println("--- GPIO Output States ---");
  Serial.printf("PUMP (%d): %s, EXT_HEATER (%d): %s, INT_HEATER (%d): %s, BLOWER (%d): %s\n",
                PUMP_PIN, digitalRead(PUMP_PIN) ? "HIGH" : "LOW",
                EXTERNAL_HEATER_PIN, digitalRead(EXTERNAL_HEATER_PIN) ? "HIGH" : "LOW",
                INTERNAL_HEATER_PIN, digitalRead(INTERNAL_HEATER_PIN) ? "HIGH" : "LOW",
                BLOWER_PIN, digitalRead(BLOWER_PIN) ? "HIGH" : "LOW");
  Serial.println("==========================");
}


//================================================================================
//                               UTILITY FUNCTIONS
//================================================================================

const char* bool_to_on_off(bool state) {
  return state ? "ON" : "OFF";
}

bool is_payload_on(const byte* payload, unsigned int length) {
  char temp[length + 1];
  memcpy(temp, payload, length);
  temp[length] = '\0'; // Null terminate
  
  if (strcasecmp(temp, "ON") == 0) return true;
  if (strcasecmp(temp, "TRUE") == 0) return true;
  if (strncmp(temp, "1", 1) == 0) return true;
  
  return false;
}