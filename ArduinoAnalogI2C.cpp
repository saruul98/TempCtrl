#include "ArduinoAnalog.h"
#include "ArduinoAnalogI2C.h"
#include "analogShield.h"
#include <Wire.h>

double _data_1, _data_2;

union T {byte b[16]; double d;} T;
union X {byte b[16]; double d;} X;

namespace NewTempCtrl {

	ArduinoAnalogI2C::ArduinoAnalogI2C(int nodeAddress, int payloadSize, int requestDelay, int signalSelect) :
		_nodeAddress(nodeAddress), _payloadSize(payloadSize), _requestDelay(requestDelay), _signalSelect(signalSelect),  
		ArduinoAnalog(10, 10, false, 1) {}
	
	ArduinoAnalogI2C::ArduinoAnalogI2C(ArduinoAnalogI2C * rhs) :
		ArduinoAnalogI2C(rhs->_nodeAddress, rhs->_payloadSize, rhs->_requestDelay, rhs->_signalSelect) {}


	double ArduinoAnalogI2C::readError()
	{

    	Wire.requestFrom(_nodeAddress, _payloadSize); // Request packet from slave
	
      if (Wire.available() == _payloadSize){
        for (byte i = 0; i < _payloadSize; i++){
          T.b[i] = Wire.read();
          X.b[i] = Wire.read();
        }
        _data_1 = T.d;
        _data_2 = X.d;
      }

  	delay(_requestDelay); // Delay so that we don't saturate slaves with requests

  		double error_1 = _data_1 / 32767.0; // Convert error signal to (-1, 1) range
  		double error_2 = _data_2 / 32767.0;

  		if (_signalSelect == 1) return error_1; // Return the selected error signal
  		if (_signalSelect == 2) return error_2;
	}


		ArduinoAnalogI2C::~ArduinoAnalogI2C(){}
}
