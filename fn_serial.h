#ifdef ENABLE_SERIAL
  #define SERIAL_DEBUG      true
  #define SERIAL_START      ( Serial.begin(115200) )
  #define SERIAL_FLUSH      ( Serial.flush() )
  #define SERIAL_PRINT(a)   ( Serial.print(a) )
  #define SERIAL_PRINTLN(a) ( Serial.println(a) )
#else
  #define SERIAL_DEBUG      false
  #define SERIAL_START
  #define SERIAL_FLUSH
  #define SERIAL_PRINT(a)
  #define SERIAL_PRINTLN(a)
#endif
