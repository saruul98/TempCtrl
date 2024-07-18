/*
****************************************
Program for slave Arduino+AnalogShield 

Arduino+AnalogShield receives 2 error signals:
E2+ @ ADC 0
E2- @ ADC 1
E3+ @ ADC 2
E3- @ ADC 3

1. The signals are read in differential mode (ADCs 0-1 & 2-3) and results are signed int.
2. The data is converted to bytes and transmitted to Master using the I2C protocol upon request.

Each transmitted packet is 9 bytes:
[0] : Slave address
[1] : Error signal 1 (1/4)
[2] : Error signal 1 (2/4)
[3] : Error signal 1 (3/4)
[4] : Error signal 1 (4/4)
[5] : Error signal 2 (1/4)
[6] : Error signal 2 (2/4)
[7] : Error signal 2 (3/4)
[8] : Error signal 2 (4/4)

****************************************
*/


#include <Wire.h>
#include <analogShield.h>

#define NODE_ADDRESS 1  // Arduino slave 2 unique address
#define PAYLOAD_SIZE 9 // Packet size for transmission
#define ADC_AVERAGES 1 // Number of ADC measurement averages
#define ADC_0to1_CAL -0.0028681
#define ADC_2to3_CAL -0.0023711

byte nodePayload[PAYLOAD_SIZE];
byte data_1[4];
byte data_2[4];

double _ADC_0to1_CAL = ADC_0to1_CAL * 32767.0;
double _ADC_2to3_CAL = ADC_2to3_CAL * 32767.0;


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
}


void loop()
{ 
  
  long errors1=0, errors2=0, errorSignal_1, errorSignal_2;
  long referenceVal1 = analog.signedRead(0, true);
  long referenceVal2 = analog.signedRead(2, true);
  for (int i = 1; i< ADC_AVERAGES; i++) {
      // Sum the differences from the ref value
      errors1 += analog.signedRead(0, true) - referenceVal1;
      errors2 += analog.signedRead(2, true) - referenceVal2;
    }
    errorSignal_1 = errors1 / ADC_AVERAGES + referenceVal1 + _ADC_0to1_CAL;
    errorSignal_2 = errors2 / ADC_AVERAGES + referenceVal2 + _ADC_2to3_CAL;

  integerToBytes(errorSignal_1, data_1); // Create byte arrays with error signals
  integerToBytes(errorSignal_2, data_2);
  
  // Debugging...  
  Serial.print("Error signal ADC 0-1: "); 
  Serial.println(errorSignal_1);
  Serial.print("Error signal ADC 2-3: "); 
  Serial.println(errorSignal_2);

  nodePayload[0] = NODE_ADDRESS;
  nodePayload[1] = data_1[0];
  nodePayload[2] = data_1[1];
  nodePayload[3] = data_1[2];
  nodePayload[4] = data_1[3];
  nodePayload[5] = data_2[0];
  nodePayload[6] = data_2[1];
  nodePayload[7] = data_2[2];
  nodePayload[8] = data_2[3];

}


void requestEvent() { 
  Wire.write(nodePayload,PAYLOAD_SIZE); // Send packet when requested
}


void integerToBytes(long val, byte b[4]) {
  // Creates byte[4] array from integer value
 b[0] = (byte )((val >> 24) & 0xff);
 b[1] = (byte )((val >> 16) & 0xff);
 b[2] = (byte )((val >> 8) & 0xff);
 b[3] = (byte )(val & 0xff);
}


long bytesToInteger(byte b[4]) {
  // Returns long value from byte[4] array
 long val = 0;
 val = ((long )b[0]) << 24;
 val |= ((long )b[1]) << 16;
 val |= ((long )b[2]) << 8;
 val |= b[3];
 return val;
}

