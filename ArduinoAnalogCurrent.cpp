#include "ArduinoAnalogCurrent.h"

#define MAX_CURRENT 0.9
#define MIN_CURRENT 0

// This object will accept values from -1 -> +1, and output current from MIN_CURRENT A to MAX_CURRENT A
// OPA limits are -3A to +3A

namespace NewTempCtrl {
	ArduinoAnalogCurrent::ArduinoAnalogCurrent(int ADCport, int DACport, int DACCurrentLimitPort, bool ADCDifferential, int ADCaverages) :
		_DACCurrentLimitPort(DACCurrentLimitPort), ArduinoAnalog(ADCport, DACport, ADCDifferential, ADCaverages)
	{
		// Max limits for the OPA to start with:
		_minimumLimit = -1;
		_maximumLimit = 1;
	}

	ArduinoAnalogCurrent::ArduinoAnalogCurrent(ArduinoAnalogCurrent * rhs) :
		ArduinoAnalog(rhs->_ADCport, rhs->_DACport, rhs->_ADCDifferential, rhs->_ADCaverages),
		_DACCurrentLimitPort(rhs->_DACCurrentLimitPort) {}


	// Write a constant current of the given value (Scaled to MAX_CURRENT amps)
	void ArduinoAnalogCurrent::writeCtrl(double val) {
		// Coerce between limits
		if (val > _maximumLimit) val = _maximumLimit;
		if (val < _minimumLimit) val = _minimumLimit;

		_currentOutput = val;

		double valInAmps = (MAX_CURRENT - MIN_CURRENT) * (val + 1.0) / 2.0 + MIN_CURRENT;

		// Calculate voltage for v_set to achieve this current
		double v_set;

		if (valInAmps < 3.0 || valInAmps >= 0.0) {
			v_set = 4.90 + (-1.070204632507655)*valInAmps + 1.095962592230871*pow(valInAmps,2.0) + (-0.8724662320180986)*pow(valInAmps,3.0) + 0.3056609584291347*pow(valInAmps,4.0) + (-0.03887862476536842)*pow(valInAmps,5.0);
		}
		else if (valInAmps < 0) v_set = 4.90;
		else v_set = 4.75 - 7500.0 * abs(valInAmps) / 15800;

		// Write +15V for positive current & vice versa
		ArduinoAnalog::writeToDAC(_DACport, (valInAmps>=0 ? 1.0 : -1.0));
		// Write the limit voltage
		ArduinoAnalog::writeToDAC(_DACCurrentLimitPort, v_set / 5.0);
	}

	ArduinoAnalogCurrent::~ArduinoAnalogCurrent()
	{
	}
}
