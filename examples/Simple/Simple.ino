#include <DS2480B.h>
#include <DallasTemperature.h>
#include <AltSoftSerial.h>

AltSoftSerial altSerial; //pins 8 and 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DS2480B ds(altSerial);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&ds);

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  altSerial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();
}

void loop(void)
{ 
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  
  Serial.print("Temperature for the device 2 (index 1) is: ");
  Serial.println(sensors.getTempCByIndex(1));  
}
