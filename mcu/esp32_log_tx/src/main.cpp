/*
 * ESP32 UART to UDP Forwarder with OTA and LED Indicator
 * - Connects to ESP8266 Access Point
 * - Reads data from UART (4800 baud)
 * - Forwards data to ESP8266 via UDP port 5555
 * - Includes OTA update functionality
 * - WS2812 LED status indicator using FastLED
 */

 #include <Arduino.h>
 #include <WiFi.h>
 #include <WiFiUdp.h>
 #include <ArduinoOTA.h>
 #include <ESPmDNS.h>
 #include <FastLED.h>
 
 // Configuration
 #define AP_SSID "ESP-Display"         // AP name from ESP8266
 #define AP_PASS "12345678"            // AP password from ESP8266
 #define HOSTNAME "ESP32-UART-Fwd"     // mDNS hostname for OTA
 #define ESP8266_IP "192.168.4.1"      // Default IP of ESP8266 in AP mode
 #define UDP_PORT 5555                 // UDP port (same as ESP8266 is listening on)
 #define UART_RX 27                    // UART RX pin
 #define UART_TX 26                    // UART TX pin
 #define UART_BAUD 4800                // UART baud rate
 #define BUFFER_SIZE 1024              // Size of receive buffer
 #define WIFI_TIMEOUT 10000            // WiFi connection timeout (ms)
 #define WIFI_CHECK_INTERVAL 500       // How often to check WiFi (ms)
 #define LED_PIN 25                    // GPIO pin for WS2812 LED
 #define NUM_LEDS 1                    // Number of LEDs in the strip
 
 // Function declarations
 void setupLED();
 void updateLED(CRGB color);
 void setupOTA();
 void connectToWiFi();
 void checkWiFi();
 void readUartAndForward();
 void sendUdpPacket(String &data);
 
 // FastLED
 CRGB leds[NUM_LEDS];
 
 // Global variables
 WiFiUDP udp;
 char buffer[BUFFER_SIZE];
 unsigned long lastWiFiCheck = 0;
 unsigned long lastMsgTime = 0;
 unsigned long lastLedUpdate = 0;
 bool initialMessage = true;
 bool otaInProgress = false;
 int ledState = 0;
 
 void setup() {
   // Initialize primary serial for debugging
   Serial.begin(115200);
   Serial.println("\nESP32 UART to UDP Forwarder starting...");
   
   // Initialize UART on specified pins
   Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
   Serial.printf("UART initialized on pins RX:%d, TX:%d at %d baud\n", UART_RX, UART_TX, UART_BAUD);
   
   // Initialize LED strip
   setupLED();
   
   // Connect to WiFi
   connectToWiFi();
   
   // Setup OTA
   setupOTA();
 }
 
 void loop() {
   // Handle OTA updates
   ArduinoOTA.handle();
   
   // Check and maintain WiFi connection
   checkWiFi();
   
   // Read from UART and forward to UDP
   readUartAndForward();
   
   // Small delay to prevent CPU hogging
   delay(10);
 }
 
 void setupLED() {
   // Initialize FastLED
   FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
   FastLED.setBrightness(50); // 50% brightness to reduce current draw
   
   Serial.println("LED strip initialized");
   // Set initial LED color to red (not connected)
   updateLED(CRGB::Red);
 }
 
 void updateLED(CRGB color) {
   leds[0] = color;
   FastLED.show();
 }
 
 void setupOTA() {
   // Set hostname for OTA and mDNS
   ArduinoOTA.setHostname(HOSTNAME);
   
   // OTA callbacks
   ArduinoOTA.onStart([]() {
     String type;
     if (ArduinoOTA.getCommand() == U_FLASH) {
       type = "sketch";
     } else {  // U_SPIFFS
       type = "filesystem";
     }
     Serial.println("OTA: Start updating " + type);
     updateLED(CRGB::Orange);
     otaInProgress = true;
   });
   
   ArduinoOTA.onEnd([]() {
     Serial.println("\nOTA: End");
     otaInProgress = false;
   });
   
   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     Serial.printf("OTA: Progress: %u%%\r", (progress / (total / 100)));
   });
   
   ArduinoOTA.onError([](ota_error_t error) {
     Serial.printf("OTA: Error[%u]: ", error);
     if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
     else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
     else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
     else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
     else if (error == OTA_END_ERROR) Serial.println("End Failed");
     updateLED(CRGB::Red);
     otaInProgress = false;
   });
   
   // Start OTA service
   ArduinoOTA.begin();
   
   // Setup mDNS
   if (MDNS.begin(HOSTNAME)) {
     Serial.println("mDNS responder started");
     Serial.printf("You can update this device using: %s.local\n", HOSTNAME);
   } else {
     Serial.println("Error setting up mDNS responder!");
   }
   
   Serial.println("OTA ready");
 }
 
 void connectToWiFi() {
   Serial.printf("Connecting to %s...", AP_SSID);
   
   WiFi.mode(WIFI_STA);
   WiFi.begin(AP_SSID, AP_PASS);
   
   unsigned long startAttemptTime = millis();
   
   while (WiFi.status() != WL_CONNECTED && 
          millis() - startAttemptTime < WIFI_TIMEOUT) {
     Serial.print(".");
     delay(500);
   }
   
   if (WiFi.status() == WL_CONNECTED) {
     Serial.println("\nConnected!");
     Serial.print("IP address: ");
     Serial.println(WiFi.localIP());
     
     // Update LED to green (connected)
     updateLED(CRGB::Green);
     
     // Send initial connection message
     udp.beginPacket(ESP8266_IP, UDP_PORT);
     String msg = "ESP32 UART Forwarder Connected\n";
     udp.write((uint8_t*)msg.c_str(), msg.length());
     udp.endPacket();
     lastMsgTime = millis();
   } else {
     Serial.println("\nFailed to connect!");
     // Keep LED red to indicate connection failure
     updateLED(CRGB::Red);
   }
 }
 
 void checkWiFi() {
   unsigned long currentMillis = millis();
   
   // Check WiFi connection every WIFI_CHECK_INTERVAL
   if (currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
     lastWiFiCheck = currentMillis;
     
     if (WiFi.status() != WL_CONNECTED) {
       Serial.println("WiFi disconnected. Reconnecting...");
       updateLED(CRGB::Red);
       connectToWiFi();
     }
   }
 }
 
 void readUartAndForward() {
   static String uartLine = "";
   static bool hasData = false;
   
   // Check if there's data available from UART
   while (Serial2.available()) {
     char c = Serial2.read();
     uartLine += c;
     hasData = true;
     
     // Process complete line or when buffer is getting full
     if (c == '\n' || uartLine.length() >= BUFFER_SIZE - 1) {
       sendUdpPacket(uartLine);
       uartLine = "";
     }
   }
   
   // If we have collected data but no newline for a while, send anyway
   if (hasData && uartLine.length() > 0 && millis() - lastMsgTime > 1000) {
     sendUdpPacket(uartLine);
     uartLine = "";
     hasData = false;
   }
 }
 
 void sendUdpPacket(String &data) {
   if (WiFi.status() == WL_CONNECTED) {
     udp.beginPacket(ESP8266_IP, UDP_PORT);
     udp.write((uint8_t*)data.c_str(), data.length());
     udp.endPacket();
     
     Serial.print("Sent: ");
     Serial.println(data);
     
     lastMsgTime = millis();
     
     // Briefly flash LED blue to indicate data transmission
     updateLED(CRGB::Blue);
     delay(50);
     updateLED(CRGB::Green);
     
     // If this was the first message after boot, add a newline to
     // ensure proper display formatting on the ESP8266
     if (initialMessage) {
       delay(100);
       String newline = "\n";
       udp.beginPacket(ESP8266_IP, UDP_PORT);
       udp.write((uint8_t*)newline.c_str(), newline.length());
       udp.endPacket();
       initialMessage = false;
     }
   } else {
     Serial.println("WiFi not connected, can't send UDP");
   }
 }