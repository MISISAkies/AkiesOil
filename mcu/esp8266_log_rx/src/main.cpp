/* 
  ESP8266 UDP Display Console
  - Sets up a WiFi AP
  - Listens on UDP port 5555
  - Displays received data on OLED
  - Supports scrolling for large texts
  - Has reset screen button
  - Includes OTA updates
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <GyverOLED.h>
#include <CircularBuffer.h>
#include <EncButton.h>
#include <LittleFS.h>

/* ======== Configuration ======== */
#define AP_SSID "ESP-Display"         // AP name
#define AP_PASS "12345678"            // AP password (min 8 chars)
#define UDP_PORT 5555                 // UDP port to listen on
#define MAX_PACKET_SIZE 1024          // UDP packet size
#define DISPLAY_LINES 8               // Lines on display
#define DISPLAY_CHARS 21              // Characters per line
#define SCROLL_BUFFER_LINES 100       // Buffer size in lines
#define UP_BTN_PIN 14                 // Scroll up button
#define OK_BTN_PIN 12                 // Reset/clear screen button
#define DOWN_BTN_PIN 13               // Scroll down button
#define IIC_SDA_PIN 4                 // OLED SDA pin
#define IIC_SCL_PIN 5                 // OLED SCL pin

/* ======== Global Objects ======== */
GyverOLED<SSD1306_128x64> oled;
WiFiUDP udp;
Button up(UP_BTN_PIN);
Button ok(OK_BTN_PIN);
Button down(DOWN_BTN_PIN);

/* ======== Global Variables ======== */
CircularBuffer<String, SCROLL_BUFFER_LINES> textBuffer;
int currentLine = 0;
char packetBuffer[MAX_PACKET_SIZE];
bool needDisplay = false;

void displayText();
void clearScreen();
void processIncomingText(char* text);

void setup() {
  Serial.begin(115200);
  
  // Initialize buttons
  pinMode(UP_BTN_PIN, INPUT_PULLUP);
  pinMode(OK_BTN_PIN, INPUT_PULLUP);
  pinMode(DOWN_BTN_PIN, INPUT_PULLUP);
  
  // Initialize OLED with proper pin parameters
  oled.init(IIC_SDA_PIN, IIC_SCL_PIN);
  
  oled.clear();
  oled.setScale(1);
  oled.setCursor(0, 0);
  oled.print("Starting...");
  oled.update();
  
  // Initialize filesystem (for OTA)
  if (!LittleFS.begin()) {
    LittleFS.format();
    LittleFS.begin();
  }
  
  // Setup WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress myIP = WiFi.softAPIP();
  
  // Start UDP
  udp.begin(UDP_PORT);
  
  // Setup OTA
  ArduinoOTA.setHostname("ESP-Display");
  ArduinoOTA.begin();
  
  // Show ready screen
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("UDP Display Ready");
  oled.setCursor(0, 2);
  oled.print("AP: ");
  oled.print(AP_SSID);
  oled.setCursor(0, 3);
  oled.print("IP: ");
  oled.print(myIP.toString());
  oled.setCursor(0, 4);
  oled.print("Port: ");
  oled.print(UDP_PORT);
  oled.setCursor(0, 6);
  oled.print("Waiting for data...");
  oled.update();
  
  delay(3000);
  clearScreen();
}

void loop() {
  ArduinoOTA.handle();  // Handle OTA updates
  
  up.tick();
  ok.tick();
  down.tick();
  
  // Check for UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read packet
    int len = udp.read(packetBuffer, MAX_PACKET_SIZE - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
      processIncomingText(packetBuffer);
      needDisplay = true;
    }
  }
  
  // Handle buttons
  if (ok.click()) {
    clearScreen();
    needDisplay = true;
  }
  
  if (up.click() || up.step()) {
    if (currentLine > 0) {
      currentLine--;
      needDisplay = true;
    }
  }
  
  if (down.click() || down.step()) {
    if (currentLine < textBuffer.size() - DISPLAY_LINES) {
      currentLine++;
      needDisplay = true;
    }
  }
  
  // Update display when needed
  if (needDisplay) {
    displayText();
    needDisplay = false;
  }
}

void processIncomingText(char* text) {
  // Process text by line
  String currentText = String(text);
  int startPos = 0;
  int endPos = 0;
  
  while (startPos < currentText.length()) {
    endPos = currentText.indexOf('\n', startPos);
    
    // If no newline found, use the rest of the string
    if (endPos == -1) {
      endPos = currentText.length();
    }
    
    // Extract the line
    String line = currentText.substring(startPos, endPos);
    
    // Process long lines
    while (line.length() > DISPLAY_CHARS) {
      textBuffer.push(line.substring(0, DISPLAY_CHARS));
      line = line.substring(DISPLAY_CHARS);
    }
    
    // Add remaining text or short line
    textBuffer.push(line);
    
    // Move to next line position (after newline)
    startPos = endPos + 1;
  }
  
  // Auto-scroll to bottom
  currentLine = max(0, (int)textBuffer.size() - DISPLAY_LINES);
}

void displayText() {
  oled.clear();
  
  int lines = min(DISPLAY_LINES, (int)textBuffer.size());
  int startLine = min(currentLine, max(0, (int)textBuffer.size() - DISPLAY_LINES));
  
  for (int i = 0; i < lines; i++) {
    int bufferIndex = startLine + i;
    if (bufferIndex < textBuffer.size()) {
      oled.setCursor(0, i);
      oled.print(textBuffer[bufferIndex]);
    }
  }
  
  // Show scrollbar if needed
  if (textBuffer.size() > DISPLAY_LINES) {
    int barHeight = max(1, (DISPLAY_LINES * 8) / textBuffer.size());
    int barPos = (startLine * (DISPLAY_LINES * 8 - barHeight)) / max(1, textBuffer.size() - DISPLAY_LINES);
    oled.rect(127, 0, 127, 63, 1);
    oled.rect(127, barPos, 127, barPos + barHeight, 1);
  }
  
  oled.update();
}

void clearScreen() {
  while (!textBuffer.isEmpty()) {
    textBuffer.pop();
  }
  currentLine = 0;
  oled.clear();
  oled.setCursor(0, 3);
  oled.print("  Screen cleared");
  oled.update();
  delay(1000);
  oled.clear();
  oled.update();
}