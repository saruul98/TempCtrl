/*
Class for I2C communication of error signals. 
The GND, 5V, A4 and A5 pins of the Arduinos should be connected in series respectively.
This program is uploaded to the master Arduino. Program for slave Arduinos is in comment at the end of this file.

When the readError() function is called, the master Arduino requests a 16 byte packet from each slave Arduino.
The packet consists of the following data:

The error signal is measured in the slaves Arduinos using the analog.signedRead function from analogShield.h in differential mode.
The AnalogShield's ADC resolution is 16bit, the signal goes from -32767 to 32767.
Each error signal is saved in a signed long int variable and converted to bytes.
Upon request, the slave Arduinos send their packets to the master.

Instructions:
-------------
Create a member of this class for each error signal, i.e. two per slave Arduino if all ADC channels are used.

ArduinoAnalogI2C(int nodeAddress, int payloadSize, int requestDelay, int signalSelect)

nodeAddress: Unique I2C address of slave Arduino.
payloadSize: Number of bytes sent, should be left 9 unless the slave program is modified.
requestDelay: Time to wait after each request before making a new one.
signalSelect: 1 to select the slave's ADC 0-1 error signal, 2 to select the slave's ADC 2-3 error signal.

E.g. if there are two slave Arduinos and each is connected to two thermistor bridges, create four members:

	I2C_Interface1 = new ArduinoAnalogI2C(1, 9, 200, 1);
	I2C_Interface2 = new ArduinoAnalogI2C(1, 9, 200, 2);
	I2C_Interface3 = new ArduinoAnalogI2C(2, 9, 200, 1);
	I2C_Interface4 = new ArduinoAnalogI2C(2, 9, 200, 2);

*/



#pragma once
#include "IOInterface.h"
#include "ArduinoAnalog.h"
#include "analogShield.h"
#include <Wire.h>

namespace NewTempCtrl {


class ArduinoAnalogI2C :
public ArduinoAnalog
{
public:

ArduinoAnalogI2C(int nodeAddress, int payloadSize, int requestDelay, int signalSelect);
ArduinoAnalogI2C(ArduinoAnalogI2C * rhs);
~ArduinoAnalogI2C();


double readError();


protected:
int _nodeAddress, _payloadSize, _requestDelay, _signalSelect;

};

}




/*
****************************************
Program for slave Arduino+AnalogShield 

Arduino+AnalogShield receives 2 error signals:
E2+ @ ADC 0
E2- @ ADC 1
E3+ @ ADC 2
E3- @ ADC 3

The signals are read in differential mode (ADCs 0-1 & 2-3) and results are signed int.
The data is converted to bytes and transmitted to Master using the I2C protocol upon request.

****************************************


#include <Wire.h>
#include <analogShield.h>

#define NODE_ADDRESS 2  // Arduino slave 2 unique address
#define PAYLOAD_SIZE 16 // Packet size for transmission
#define ADC_AVERAGES 10000 // Number of ADC measurement averages
#define ADC_0to1_CAL 0
#define ADC_2to3_CAL 0

double _ADC_0to1_CAL = ADC_0to1_CAL * 32767.0;
double _ADC_2to3_CAL = ADC_2to3_CAL * 32767.0;

byte nodePayload[PAYLOAD_SIZE];
union T {byte b[8]; double d;} T;
union X {byte b[8]; double d;} X;

void setup()
{

  Serial.begin(57600);  
  Serial.println("SLAVE SENDER NODE");
  Serial.print("Node address: ");
  Serial.println(NODE_ADDRESS);
  Serial.print("Payload size: ");
  Serial.println(PAYLOAD_SIZE);
  Serial.println("***********************");

  Wire.begin(NODE_ADDRESS);  // Activate I2C network
  Wire.onRequest(requestEvent); // On request, execute requestEvent

  analog.write(0, 65535);
  analog.write(1, 65535);
  analog.write(2, 65535);
  analog.write(3, 65535);

}

void loop()
{ 
  long errors1 = 0, errors2 = 0;
  double errorSignal_1, errorSignal_2;
  long referenceVal1 = analog.signedRead(0, true);
  long referenceVal2 = analog.signedRead(2, true);
  for (int i = 1; i< ADC_AVERAGES; i++) {
      // Sum the differences from the ref value
      errors1 += analog.signedRead(0, true) - referenceVal1;
      errors2 += analog.signedRead(2, true) - referenceVal2;
  }
  errorSignal_1 = errors1 / ADC_AVERAGES + referenceVal1 + _ADC_0to1_CAL;
  errorSignal_2 = errors2 / ADC_AVERAGES + referenceVal2 + _ADC_2to3_CAL;

  floatToBytes(errorSignal_1, errorSignal_2);
  
  // Debugging...  
  Serial.print("Error signal ADC 0-1: "); 
  Serial.println(errorSignal_1);
  Serial.print("Error signal ADC 2-3: "); 
  Serial.println(errorSignal_2);
}

void requestEvent() { 
  Wire.write(nodePayload, PAYLOAD_SIZE); // Send packet when requested
}

void floatToBytes(float t, float x){
  T.d = t;
  X.d = x;
  byte j;
  for (byte i = 0; i < 8; i++){
    j = i * 2;
    nodePayload[j]     = T.b[i];
    nodePayload[j + 1] = X.b[i];
  }
}


*/

