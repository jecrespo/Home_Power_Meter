// HOME POWER METER v0.0
// Enrique Crespo <aprendiendoarduino@gmail.com> --> https://github.com/jecrespo
// FUNCTION: A portable home power meter to check power consumption at home
// Designed for Arduino Ethernet http://arduino.cc/en/Main/arduinoBoardEthernet with emonTx Shield V2 http://openenergymonitor.org/emon/emontxshield/smt
// or emonTx Shield V1 http://openenergymonitor.org/emon/emontxshield
// With support for connect a serial LCD display
// Save power data on microSD card in csv format
//
//  ----- [clip-on CurrentSensor CT (up to 4 sensors)] -----
//                            |
//                            |
//            ----- [emonTx Arduino Shield] -----
//               ----- [Arduino Ethernet] -----
//
// Power consumption reading based on library: https://github.com/openenergymonitor/EmonLib
// 
// This
//
// For Arduino Ethernet:
// Microcontroller	ATmega328
// Flash Memory			32 KB, de los cuales 0.5KB se usan por el bootloader
// SRAM           		2 KB
// EEPROM         		1 KB
// CLOCK SPEED			16 MHz
// Digital i/o Pins		14
// Analog input Pins	6
// On board microSD Pin	4 (SS for the SD) used to store files, is accessible through the SD Library. 
// Reserved Pins		Pins 10, 11, 12 y 13 are reserved for interfacing with the Ethernet module.
// Sensor reading Pins	Analog Pins A1, A2, A3 and A4 (emonTx Arduino Shield up to 4 clip-on CurrentSensor CT)
// LCD Serial Pins 		Digital pins 2(RX), 3(TX) --> Connect to RX pin in LCD display
//
// TODO List:
// * Maximun 2 currente sensor, run out memory
// * Not updating LCD display with softwareserial library
// * calculate kWh consumed
// * SD write data
// * Ethernet support
// * Clock synchronization via NTP protocol
// * embedded web server: display power data (files stored in sd card or flash memory)
// * embedded web server: reset counter and select used current sensors
// * embedded web server: download data file
// * support for selecting sensors, now selected current sensors in order from 1 to 4
// * EEPROM support for time and kWh
//
////////// LIBRARIES //////////
#include <SPI.h>                  // For communicating with devices using the Serial Peripheral Interface (SPI) Bus
#include <EmonLib.h>       	  // Include Emon Library para el c�lculo de Irms. https://github.com/openenergymonitor/EmonLib
#include <FlexiTimer2.h>	  // Very easy to use library to interface Timer2 with humans. https://github.com/PaulStoffregen/FlexiTimer2
#include <Ethernet.h>             // For connecting to the internet with ethernet port 
#include <SoftwareSerial.h>       // Allow serial communication on other digital pins of the Arduino
////////// DEFINITIONS //////////
#define SERIAL_OUTPUT 1    		     //1 for serial output information, debug purpose
#define VOLTAGE 230			//for calculate kW
#define NUM_SAMPLES 1480
#define NUM_CURRENT_SENSORS 2	//used current sensors
#define UPDATE_TIME 5000  //in ms
////////// CONSTANTS AND VARIABLES //////////
const int AnalogInputPin[4] = {A1, A2, A3, A4};	//Analog pins for current sensor
double Irms[NUM_CURRENT_SENSORS];	  //array for store measured Irms
EnergyMonitor emon[NUM_CURRENT_SENSORS];  //energy monitor objects array
SoftwareSerial LCDserial(3, 2); // RX, TX
////////// ETHERNET CONFIGURATION//////////
//static uint8_t mac[]     = {0x90, 0xA2, 0xDA, 0x0E, 0x9E, 0x76};
//static uint8_t ip[]      = {192,168,1,2}; 
//static uint8_t gateway[] = {192,168,1,1};
//static uint8_t subnet[]  = {255,255,255,0};   
//static uint8_t servidor[]= {192,168,1,1};
////////// FUNCTIONS //////////
void SensorReading()	//function executed every period UPDATE_TIME
{
  for (int i=0; i < NUM_CURRENT_SENSORS; i++){
	Irms[i] = emon[i].calcIrms(NUM_SAMPLES);  // Calculate Irms only, value =  number of samples
  }
  #if SERIAL_OUTPUT
  //Show measured current and time in ms
  Serial.print(millis());
  for (int i=0; i < NUM_CURRENT_SENSORS; i++){
	Serial.print("\tIrms Sonda ");
	Serial.print(i+1);
	Serial.print(": ");
	Serial.print(Irms[i]);		       // Irms
	Serial.print(" A");
  }
  Serial.println();
  #endif
  //show Irms for LCD display
  LCDserial.write(0xFE);	//clear LCD display
  LCDserial.write(0x01);
  for (int i=0; i < NUM_CURRENT_SENSORS; i++){
        LCDserial.begin(9600);	
        LCDserial.print("Irms");
	LCDserial.print(i);
	LCDserial.print(": ");
	LCDserial.print(Irms[i]);
	LCDserial.print(" A");
  }
}
////////// SETUP //////////
void setup(){
  //initialize serial ports
  Serial.begin(9600);
  Serial.println("-------------------- Begin: HOME POWER METER v0.0 --------------------");
  LCDserial.begin(9600);	//Serial port for LCD display
  LCDserial.write(0xFE);	//clear LCD display
  LCDserial.write(0x01);
  LCDserial.print("POWR METER v0.0");
  //initialize energy monitor CurrentSensor
  for (int i=0; i <= NUM_CURRENT_SENSORS; i++){
	 emon[i].current(AnalogInputPin[i], 60.61);     // Current: input pin, calibration =  60.61V/a for STC013 (100A:50mA) and Burden Resistance = 33ohms
	 emon[i].calcIrms(NUM_SAMPLES);  // First calculation to init sensor
  }
  //Activate timer
  FlexiTimer2::set(UPDATE_TIME, SensorReading);
  FlexiTimer2::start();
  //Ethernet.begin(mac, ip, gateway, gateway, subnet);
}
////////// LOOP //////////
void loop(){
}
