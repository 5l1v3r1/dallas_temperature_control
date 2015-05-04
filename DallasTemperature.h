#ifndef DallasTemperature_h
#define DallasTemperature_h

#define DALLASTEMPLIBVERSION "4.0.0"

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// set to true to include code for new and delete operators
#ifndef REQUIRESNEW
#define REQUIRESNEW false
#endif

#include <inttypes.h>
#include <DS2480B.h>

// Model IDs
#define DS18S20MODEL 0x10
#define DS18B20MODEL 0x28
#define DS1822MODEL  0x22

// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

// Error Codes
#define DEVICE_DISCONNECTED -127

#define MAX_DEVICES	6 //Max # of 1-wire temperature sensors to track.

typedef uint8_t DeviceAddress[8];

typedef struct 
{
	int16_t minTemp, maxTemp, avgTemp, currentTemp;	
	int32_t avgTempAccumulator;
	uint16_t avgTempReadings;
	int16_t offset;
	int8_t lowTempFault;
	int8_t highTempFault;
	uint8_t faults;
	DeviceAddress address;
} TemperatureSensor;

class DallasTemperature
{
  public:

  DallasTemperature(DS2480B*);

  // initalise bus
  void begin(void);

  // returns the number of devices found on the bus
  uint8_t getDeviceCount(void);
  
  // Is a conversion complete on the wire?
  bool isConversionComplete(void);
  
  // returns true if address is valid
  bool validAddress(uint8_t*);

  bool readSensor(uint8_t);
  
  // attempt to determine if the device at the given address is connected to the bus
  bool isConnected(uint8_t);

  // attempt to determine if the device at the given address is connected to the bus
  // also allows for updating the read scratchpad
  bool isConnected(uint8_t, uint8_t*);

  // read device's scratchpad
  void readScratchPad(uint8_t, uint8_t*);

  // write device's scratchpad
  void writeScratchPad(uint8_t, const uint8_t*);

  // read device's power requirements
  bool readPowerSupply(uint8_t);

  // get global resolution
  uint8_t getResolution();
  
  // set global resolution to 9, 10, 11, or 12 bits
  void setResolution(uint8_t);

  // returns the device resolution, 9-12
  uint8_t getResolution(uint8_t);

  // set resolution of a device to 9, 10, 11, or 12 bits
  bool setResolution(uint8_t, uint8_t);
  
  // sets/gets the waitForConversion flag
  void setWaitForConversion(bool);
  bool getWaitForConversion(void);
  
  // sets/gets the checkForConversion flag
  void setCheckForConversion(bool);
  bool getCheckForConversion(void);
  
  // sends command for all devices on the bus to perform a temperature conversion 
  void requestTemperatures(void);

  // returns temperature in degrees C
  float getTempC(uint8_t);

  // returns temperature in degrees F
  float getTempF(uint8_t);

  float getMaxTempC(uint8_t);
  float getMinTempC(uint8_t);
  float getAvgTempC(uint8_t);
  void resetStats(uint8_t);
  void resetStats();
  void setLowFaultTemp(uint8_t, int8_t);
  void setHighFaultTemp(uint8_t, int8_t);
  uint8_t isFaulted(uint8_t);


  //Get temperature in hundredths of a degree C
  int16_t getCelsius(uint8_t);

  //Get Temperature in hundredths of a degree F
  int16_t getFahrenheit(uint8_t);
  
  // returns true if the bus requires parasite power
  bool isParasitePowerMode(void);
  
  bool isConversionAvailable(uint8_t);

  // convert from celcius to farenheit
  static float toFahrenheit(const float);

  // convert from farenheit to celsius
  static float toCelsius(const float);

  #if REQUIRESNEW

  // initalize memory area
  void* operator new (unsigned int);

  // delete memory reference
  void operator delete(void*);
  
  #endif

  private:
  typedef uint8_t ScratchPad[9];

  TemperatureSensor sensors[MAX_DEVICES];
  
  // parasite power on or off
  bool parasite;

  // used to determine the delay amount needed to allow for the
  // temperature conversion to take place
  uint8_t bitResolution;
  
  // used to requestTemperature with or without delay
  bool waitForConversion;
  
  // used to requestTemperature to dynamically check if a conversion is complete
  bool checkForConversion;
  
  // count of devices on the bus
  uint8_t devices;
  
  // Take a pointer to one wire instance
  DS2480B* _wire;

  // reads scratchpad and returns the temperature in degrees C
  float calculateTemperature(uint8_t, uint8_t*);
  
  void	blockTillConversionComplete(uint8_t*,uint8_t);
    
};
#endif
