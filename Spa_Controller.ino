//================================================================================
// Standalone Spa Controller for ESP32 with WiFi Configuration Portal
//
// VERSION 6: CLEANED UP AND FULLY ROBUST
// - Removed conflicting NTPClient library, using only ezTime.
// - Fixed web page to display the correct, timezone-aware time from ezTime.
// - Re-integrated logic to correctly handle overnight schedules (e.g., 22:00 to 06:00).
//================================================================================

#include <ESPmDNS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <ezTime.h>      // The ONLY time library we need
#include <EEPROM.h>

//--- Pin Definitions ---
const int CIRCULATION_PUMP_PIN = 13;
const int HEATER_PIN = 12;
const int ONBOARD_LED_PIN = 2;

//--- WiFi & Web Server Configuration ---
const char* HOSTNAME = "spa-controller";
WebServer server(80);

//--- Access Point (AP) Configuration for Setup ---
const char* AP_SSID = "ESP32-Spa-Setup";

//--- Timezone Configuration ---
Timezone myTimezone;

//--- Persistent Storage (EEPROM) ---
#define EEPROM_SIZE 256

// --- Factory Reset Configuration ---
const int RESET_EEPROM_ADDR = EEPROM_SIZE - 1;
const int RESET_CYCLE_TARGET = 5;
const unsigned long SUCCESSFUL_BOOT_TIMEOUT_MS = 20000;

// --- Global variables ---
unsigned long boot_time = 0;
bool reset_counter_cleared = false;
unsigned long last_wifi_reconnect_attempt = 0;
unsigned long last_time_debug_print = 0;

//--- Data Structures ---
struct Schedule {
  int start_hour;
  int start_minute;
  int end_hour;
  int end_minute;
};

struct WifiConfig {
  char ssid[33] = "";
  char password[65] = "";
};

struct ControllerConfig {
  uint32_t magic_marker = 0xDEADBEEF;
  WifiConfig wifi;
  Schedule pump_schedules[3];
  Schedule heater_schedules[3];
};

ControllerConfig controller_config;

//--- Controller State Variables ---
bool circulation_pump_state = false;
bool heater_state = false;
bool manual_override = false;
bool in_config_mode = false;

//================================================================================
//                             FORWARD DECLARATIONS
//================================================================================
bool connect_to_wifi();
void setup_ap_mode();
void update_relays();
void handle_root();
void handle_schedule();
void handle_manual_control();
void handle_config_root();
void handle_save_wifi();
void load_config_from_eeprom();
void save_config_to_eeprom();
void print_config();
void clear_eeprom_and_restart();
const char* bool_to_on_off(bool state);
String format_time(int hour, int minute);



//================================================================================
//                                  SETUP
//================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[INFO] Booting Spa Controller (Version 6)...");

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

  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW);
  pinMode(CIRCULATION_PUMP_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(CIRCULATION_PUMP_PIN, LOW);
  digitalWrite(HEATER_PIN, LOW);

  load_config_from_eeprom();

  if (!connect_to_wifi()) {
    in_config_mode = true;
    setup_ap_mode();
  } else {
    in_config_mode = false;
    
  Serial.println("[NTP] Attempting initial time synchronization (15s timeout)...");
  bool time_synced = waitForSync(15); // Capture the result of the sync attempt

  if (time_synced) {
    Serial.println("[NTP] Initial time sync SUCCESS!");
  } else {
    Serial.println("[NTP] WARNING: Initial time sync FAILED / TIMED OUT. Will retry in background.");
  }

myTimezone.setLocation("Australia/Adelaide");
Serial.println("[INFO] Timezone set to Australia/Adelaide.");
    Serial.println("[INFO] Timezone set to Australia/Adelaide.");
    
    server.on("/", HTTP_GET, handle_root);
    server.on("/schedule", HTTP_POST, handle_schedule);
    server.on("/control", HTTP_POST, handle_manual_control);
    ElegantOTA.begin(&server);
    server.begin();
    Serial.println("[INFO] HTTP server started for main application.");
  }
}

//================================================================================
//                                 MAIN LOOP
//================================================================================
void loop() {
  if (!reset_counter_cleared && millis() - boot_time > SUCCESSFUL_BOOT_TIMEOUT_MS) {
    Serial.println("[RESET] Successful boot. Resetting boot counter.");
    EEPROM.write(RESET_EEPROM_ADDR, 0);
    EEPROM.commit();
    reset_counter_cleared = true;
  }

  events(); // Let ezTime handle background events
  
  if (in_config_mode) {
    server.handleClient();
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - last_wifi_reconnect_attempt > 30000) {
        Serial.println("[WiFi] Connection lost. Attempting to reconnect...");
        WiFi.begin(controller_config.wifi.ssid, controller_config.wifi.password);
        last_wifi_reconnect_attempt = millis();
      }
    }

    if (!manual_override) {
      int current_hour = myTimezone.hour();
      int current_minute = myTimezone.minute();
      long current_time_in_minutes = current_hour * 60 + current_minute;

      // --- Check Pump Schedules (Corrected for Overnight Logic) ---
      bool pump_should_be_on = false;
      for (int i = 0; i < 3; i++) {
        long start_time = controller_config.pump_schedules[i].start_hour * 60 + controller_config.pump_schedules[i].start_minute;
        long end_time = controller_config.pump_schedules[i].end_hour * 60 + controller_config.pump_schedules[i].end_minute;
        if (start_time == end_time) continue; // Skip disabled schedules
        bool is_active = false;
        if (start_time < end_time) { // Normal same-day schedule
          if (current_time_in_minutes >= start_time && current_time_in_minutes < end_time) is_active = true;
        } else { // Overnight schedule
          if (current_time_in_minutes >= start_time || current_time_in_minutes < end_time) is_active = true;
        }
        if (is_active) {
          pump_should_be_on = true;
          break;
        }
      }
      circulation_pump_state = pump_should_be_on;

      // --- Check Heater Schedules (Corrected for Overnight Logic) ---
      bool heater_should_be_on = false;
      for (int i = 0; i < 3; i++) {
        long start_time = controller_config.heater_schedules[i].start_hour * 60 + controller_config.heater_schedules[i].start_minute;
        long end_time = controller_config.heater_schedules[i].end_hour * 60 + controller_config.heater_schedules[i].end_minute;
        if (start_time == end_time) continue; // Skip disabled schedules
        bool is_active = false;
        if (start_time < end_time) { // Normal same-day schedule
          if (current_time_in_minutes >= start_time && current_time_in_minutes < end_time) is_active = true;
        } else { // Overnight schedule
          if (current_time_in_minutes >= start_time || current_time_in_minutes < end_time) is_active = true;
        }
        if (is_active) {
          heater_should_be_on = true;
          break;
        }
      }
      heater_state = heater_should_be_on;
    }

    update_relays();
    server.handleClient();
    ElegantOTA.loop();
  }
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

//================================================================================
//                    MAIN APPLICATION WEB HANDLERS
//================================================================================
void handle_root() {
  // Use chunked transfer to send the page in pieces, saving memory
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  server.sendContent("<html><head><title>ESP32 Spa Controller</title><meta http-equiv='refresh' content='10'>");
  server.sendContent(R"rawliteral(<style>
    body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; }
    h1, h2 { color: #0056b3; } table { width: 80%; border-collapse: collapse; margin-top: 20px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
    th, td { padding: 12px; border: 1px solid #ddd; text-align: left; } th { background-color: #007bff; color: white; }
    .state-on { color: #28a745; font-weight: bold; } .state-off { color: #dc3545; font-weight: bold; }
    form { margin-top: 20px; padding: 15px; background-color: #fff; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
    input[type='time'], input[type='submit'], button { padding: 8px; margin: 5px; border-radius: 4px; border: 1px solid #ccc; background-color: #007bff; color: white; cursor: pointer; }
    button:hover, input[type='submit']:hover { background-color: #0056b3; } input[type='time'] { background-color: #fff; color: #333; }
    .info { font-style: italic; color: #555; }
  </style></head><body>)rawliteral");

  String temp;
  temp = "<h1>ESP32 Spa Controller</h1>";
  // --- FIXED: Display time from the correct ezTime object ---
  temp += "<p><strong>Current Time:</strong> " + myTimezone.dateTime("d-M-Y H:i:s") + "</p>";
  server.sendContent(temp);

  temp = "<h2>Live Status</h2><table><tr><th>Component</th><th>Current State</th></tr>";
  temp += "<tr><td>Circulation Pump</td><td class='state-" + String(digitalRead(CIRCULATION_PUMP_PIN) ? "on" : "off") + "'>" + bool_to_on_off(digitalRead(CIRCULATION_PUMP_PIN)) + "</td></tr>";
  temp += "<tr><td>Heater</td><td class='state-" + String(digitalRead(HEATER_PIN) ? "on" : "off") + "'>" + bool_to_on_off(digitalRead(HEATER_PIN)) + "</td></tr></table>";
  server.sendContent(temp);

  temp = "<h2>Manual Control</h2>";
  if (manual_override) { temp += "<p style='color:red;'>Manual override is active. Scheduling is paused.</p>"; }
  temp += R"rawliteral(<form action='/control' method='post'>
    <button type='submit' name='pump' value='1'>Pump ON</button><button type='submit' name='pump' value='0'>Pump OFF</button> | 
    <button type='submit' name='heater' value='1'>Heater ON</button><button type='submit' name='heater' value='0'>Heater OFF</button> | 
    <button type='submit' name='schedule' value='1'>Resume Schedule</button></form>)rawliteral";
  server.sendContent(temp);
  
  server.sendContent("<h2>Set Schedules (24-hour format)</h2><p class='info'>To disable a schedule, set both start and end times to 00:00.</p><form action='/schedule' method='post'>");
  
  temp = "<h3>Pump Schedules</h3><table><tr><th>Schedule #</th><th>Start Time</th><th>End Time</th></tr>";
  for (int i = 0; i < 3; i++) {
    temp += "<tr><td>" + String(i + 1) + "</td>";
    temp += "<td><input type='time' name='p_start_" + String(i) + "' value='" + format_time(controller_config.pump_schedules[i].start_hour, controller_config.pump_schedules[i].start_minute) + "'></td>";
    temp += "<td><input type='time' name='p_end_" + String(i) + "' value='" + format_time(controller_config.pump_schedules[i].end_hour, controller_config.pump_schedules[i].end_minute) + "'></td></tr>";
  }
  temp += "</table>";
  server.sendContent(temp);

  temp = "<h3>Heater Schedules</h3><table><tr><th>Schedule #</th><th>Start Time</th><th>End Time</th></tr>";
  for (int i = 0; i < 3; i++) {
    temp += "<tr><td>" + String(i + 1) + "</td>";
    temp += "<td><input type='time' name='h_start_" + String(i) + "' value='" + format_time(controller_config.heater_schedules[i].start_hour, controller_config.heater_schedules[i].start_minute) + "'></td>";
    temp += "<td><input type='time' name='h_end_" + String(i) + "' value='" + format_time(controller_config.heater_schedules[i].end_hour, controller_config.heater_schedules[i].end_minute) + "'></td></tr>";
  }
  temp += "</table>";
  server.sendContent(temp);

  server.sendContent("<br><input type='submit' value='Save All Schedules'></form></body></html>");
  server.sendContent("");
}


// =============================================================================
// The functions below this line are unchanged and are included for completeness.
// =============================================================================

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

void handle_manual_control() {
    manual_override = true;
    if (server.hasArg("pump")) { circulation_pump_state = server.arg("pump").toInt() == 1; }
    if (server.hasArg("heater")) { heater_state = server.arg("heater").toInt() == 1; }
    if (server.hasArg("schedule")) { manual_override = false; }
    server.sendHeader("Location", "/");
    server.send(303);
}

void handle_schedule() {
  for (int i = 0; i < 3; i++) {
    String p_start = server.arg("p_start_" + String(i));
    controller_config.pump_schedules[i] = { p_start.substring(0, 2).toInt(), p_start.substring(3, 5).toInt(), 0, 0 };
    String p_end = server.arg("p_end_" + String(i));
    controller_config.pump_schedules[i].end_hour = p_end.substring(0, 2).toInt();
    controller_config.pump_schedules[i].end_minute = p_end.substring(3, 5).toInt();
    String h_start = server.arg("h_start_" + String(i));
    controller_config.heater_schedules[i] = { h_start.substring(0, 2).toInt(), h_start.substring(3, 5).toInt(), 0, 0 };
    String h_end = server.arg("h_end_" + String(i));
    controller_config.heater_schedules[i].end_hour = h_end.substring(0, 2).toInt();
    controller_config.heater_schedules[i].end_minute = h_end.substring(3, 5).toInt();
  }
  save_config_to_eeprom();
  server.sendHeader("Location", "/");
  server.send(303);
}

void update_relays() {
  bool final_pump_state = heater_state ? true : circulation_pump_state;
  bool final_heater_state = heater_state;
  digitalWrite(CIRCULATION_PUMP_PIN, final_pump_state);
  digitalWrite(HEATER_PIN, final_heater_state);
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
    controller_config.pump_schedules[0] = {8, 0, 10, 0};
    controller_config.pump_schedules[1] = {0, 0, 0, 0};
    controller_config.pump_schedules[2] = {0, 0, 0, 0};
    controller_config.heater_schedules[0] = {8, 15, 9, 45};
    controller_config.heater_schedules[1] = {0, 0, 0, 0};
    controller_config.heater_schedules[2] = {0, 0, 0, 0};
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
  for (int i = 0; i < 3; i++) {
    Serial.printf("Pump Schedule %d: %02d:%02d to %02d:%02d\n", i+1, controller_config.pump_schedules[i].start_hour, controller_config.pump_schedules[i].start_minute, controller_config.pump_schedules[i].end_hour, controller_config.pump_schedules[i].end_minute);
  }
  for (int i = 0; i < 3; i++) {
    Serial.printf("Heater Schedule %d: %02d:%02d to %02d:%02d\n", i+1, controller_config.heater_schedules[i].start_hour, controller_config.heater_schedules[i].start_minute, controller_config.heater_schedules[i].end_hour, controller_config.heater_schedules[i].end_minute);
  }
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

const char* bool_to_on_off(bool state) {
  return state ? "ON" : "OFF";
}

String format_time(int hour, int minute) {
  char time_str[6];
  sprintf(time_str, "%02d:%02d", hour, minute);
  return String(time_str);
}
