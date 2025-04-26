/**
 * Arduino Logger Class
 * A simple logging utility for Arduino that outputs to both 
 * hardware Serial and SoftwareSerial
 */

 #ifndef LOGGER_H
 #define LOGGER_H
 
 #include <Arduino.h>
 #include <SoftwareSerial.h>
 
 // Log levels
 enum LogLevel {
   LOG_NONE = 0,
   LOG_ERROR = 1,
   LOG_WARNING = 2, 
   LOG_INFO = 3,
   LOG_DEBUG = 4
 };
 
 class Logger {
   private:
     SoftwareSerial* _softSerial;
     LogLevel _logLevel;
     bool _useHardwareSerial;
     bool _useSoftwareSerial;
     unsigned long _startTime;
     
     // Internal logging function
     void logMessage(LogLevel level, const char* levelStr, const char* message);
     
   public:
     // Constructor
     Logger(SoftwareSerial* softSerial = NULL);
     
     // Initialize the logger
     void begin(unsigned long baudRate = 4800);
     
     // Set the minimum log level
     void setLogLevel(LogLevel level);
     
     // Enable/disable hardware Serial
     void useHardwareSerial(bool enable);
     
     // Enable/disable SoftwareSerial
     void useSoftwareSerial(bool enable);
     
     // Logging methods - for C-style strings
     void debug(const char* message);
     void info(const char* message);
     void warning(const char* message);
     void error(const char* message);
     
     // Logging methods - for String objects
     void debug(const String& message);
     void info(const String& message);
     void warning(const String& message);
     void error(const String& message);
 };
 
 #endif // LOGGER_H