/**
 * Arduino Logger Class Implementation
 */

 #include "logger.h"

 // Constructor
 Logger::Logger(SoftwareSerial* softSerial) {
   _softSerial = softSerial;
   _logLevel = LOG_INFO;
   _useHardwareSerial = true;
   _useSoftwareSerial = (softSerial != NULL);
   _startTime = 0;
 }
 
 // Initialize the logger
 void Logger::begin(unsigned long baudRate) {
   if (_useHardwareSerial) {
     Serial.begin(baudRate);
     while (!Serial && millis() < 3000); // Wait for Serial to connect, max 3 seconds
   }
   
   if (_useSoftwareSerial && _softSerial) {
     _softSerial->begin(baudRate);
   }
   
   _startTime = millis();
   info("Logger initialized");
 }
 
 // Set the minimum log level
 void Logger::setLogLevel(LogLevel level) {
   _logLevel = level;
   char buffer[50];
   sprintf(buffer, "Log level set to %d", level);
   info(buffer);
 }
 
 // Enable/disable hardware Serial
 void Logger::useHardwareSerial(bool enable) {
   _useHardwareSerial = enable;
 }
 
 // Enable/disable SoftwareSerial
 void Logger::useSoftwareSerial(bool enable) {
   _useSoftwareSerial = enable && (_softSerial != NULL);
 }
 
 // Internal logging function
 void Logger::logMessage(LogLevel level, const char* levelStr, const char* message) {
   if (level > _logLevel) return;
   
   // Format: [TIME][LEVEL] Message
   char buffer[128];
   unsigned long timeMs = millis() - _startTime;
   sprintf(buffer, "[%lu][%s] %s", timeMs, levelStr, message);
   
   if (_useHardwareSerial && Serial) {
     Serial.println(buffer);
   }
   
   if (_useSoftwareSerial && _softSerial) {
     _softSerial->println(buffer);
   }
 }
 
 // Logging methods for C-style strings
 void Logger::debug(const char* message) {
   logMessage(LOG_DEBUG, "DEBUG", message);
 }
 
 void Logger::info(const char* message) {
   logMessage(LOG_INFO, "INFO", message);
 }
 
 void Logger::warning(const char* message) {
   logMessage(LOG_WARNING, "WARN", message);
 }
 
 void Logger::error(const char* message) {
   logMessage(LOG_ERROR, "ERROR", message);
 }
 
 // Logging methods for String objects
 void Logger::debug(const String& message) { 
   debug(message.c_str()); 
 }
 
 void Logger::info(const String& message) { 
   info(message.c_str()); 
 }
 
 void Logger::warning(const String& message) { 
   warning(message.c_str()); 
 }
 
 void Logger::error(const String& message) { 
   error(message.c_str()); 
 }