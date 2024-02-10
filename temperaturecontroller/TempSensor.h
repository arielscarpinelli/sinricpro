#define TEMP_SENSOR_CONTINUE -1
#define TEMP_SENSOR_FAILURE 0
#define TEMP_SENSOR_OK 1

class TempSensor {

  private:
    OneWire ds;
    int pin;
    byte addr[8];
    byte type_s;
    unsigned long awaitUntil = 0;

  public:
    float value;
#ifdef DEBUG_ESP_PORT
    bool mock = false;
#endif

    TempSensor(int pin) : ds(OneWire(pin)), pin(pin) {}

    int search() {

      byte i;

      ds.reset_search();
      if (!ds.search(this->addr)) {
        DEBUG_MSG_("Can't find temp sensor ");
        DEBUG_MSG(pin);
        return false;
      }

      /*
      DEBUG_MSG_("Found temp sensor at addr =");
      for( i = 0; i < 8; i++) {
        DEBUG_MSG_(' ');
        DEBUG_MSG_(this->addr[i], HEX);
      }
      DEBUG_MSG("");
      */

      if (OneWire::crc8(this->addr, 7) != this->addr[7]) {
          DEBUG_MSG("CRC is not valid!");
          return false;
      }
     
      // the first ROM byte indicates which chip
      switch (addr[0]) {
        case 0x10:
          type_s = 1;
          //DEBUG_MSG("Chip = DS18S20");
          break;
        case 0x28:
          type_s = 0;
          //DEBUG_MSG("Chip = DS18B20");
          break;
        case 0x22:
          type_s = 0;
          //DEBUG_MSG("Chip = DS1822");
          break;
        default:
          DEBUG_MSG("Device is not a DS18x20 family device.");
          return false;
      }

      return true;
      
    }

    int read(unsigned long now) {
      byte i;
      byte present = 0;
      byte data[12];

      if (this->awaitUntil == 0) {

#ifdef DEBUG_ESP_PORT
        if (this->mock) {
          this->awaitUntil = now + 1500;
          return TEMP_SENSOR_CONTINUE;
        }
#endif
  
        if(!this->search()) {
          return TEMP_SENSOR_FAILURE;
        }

        ds.reset();
        ds.select(this->addr);
        ds.write(0x44, 1);        // start conversion, with parasite power on at the end

        this->awaitUntil = now + 1500;
        
        return TEMP_SENSOR_CONTINUE;

      } else if(this->awaitUntil > now)  {
        
        return TEMP_SENSOR_CONTINUE;
        
      }

      this->awaitUntil = 0;
      
      present = ds.reset();
      ds.select(addr);    
      ds.write(0xBE);         // Read Scratchpad

      /*
      DEBUG_MSG_("  Data = ");
      DEBUG_MSG_(present, HEX);
      DEBUG_MSG_(" ");
      */
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
        //DEBUG_MSG_(data[i], HEX);
        //DEBUG_MSG_(" ");
      }

      // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
      // even when compiled on a 32 bit processor.
      int16_t raw = (data[1] << 8) | data[0];
      if (this->type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }
      float convertedValue = (float)raw / 16.0;
      if (convertedValue == 85.0) {
        return TEMP_SENSOR_FAILURE;
      }
      value = convertedValue;
      //DEBUG_MSG_("Temperature ");
      //DEBUG_MSG_(value);
      //DEBUG_MSG("");
      return TEMP_SENSOR_OK;
    }
  
};
