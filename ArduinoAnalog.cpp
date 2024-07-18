#include "ArduinoAnalog.h"
#include "analogShield.h"

namespace NewTempCtrl {

	ArduinoAnalog::ArduinoAnalog(int ADCport, int DACport, bool ADCDifferential, int ADCaverages) :
		_ADCport(ADCport), _DACport(DACport),
		_ADCDifferential(ADCDifferential), _ADCaverages(ADCaverages)
	{
		// Set no software limits on output
		_minimumLimit = -1;
		_maximumLimit = 1;
	}

	ArduinoAnalog::ArduinoAnalog(ArduinoAnalog * rhs) :
		_ADCport(rhs->_ADCport), _DACport(rhs->_DACport),
		_ADCDifferential(rhs->_ADCDifferential), _ADCaverages(rhs->_ADCaverages),
		_minimumLimit(rhs->_minimumLimit), _maximumLimit(rhs->_maximumLimit)
	{}
	
	// Reads the current value on the ADC
	// Virtual Specs:
	//	     "Reads the current value of the error signal. Values from -1 to 1"
	double ArduinoAnalog::readError()
	{
		// Get the first reading as a starting point
		// Other readings will be taken w.r.t. this point so that the average calculation can 
		// use integer arithmetic & therefore be quicker and more accurate
		// than floating point maths
		//
		// LONGs overflow at about 2E9 so this is the maximum value our error sum can get to.
		// This means that, for a pessimistic ADC error of 10mv on 5V max (= 131 LSBs)
		// we can take a maximum of ~15E6 readings before we hit the buffer. 
		// 15 million is excessive anyway. Don't. 
		long referenceVal = analog.signedRead(_ADCport, _ADCDifferential);
		
		// Sum _ADCaverages measurements
		long errors = 0;
		// Start on reading 2 since reading 1 is already done
		// Its difference is 0 by definition
		for (int i = 1; i< _ADCaverages; i++) {
			// Sum the differences from the ref value
			errors += analog.signedRead(_ADCport, _ADCDifferential) - referenceVal;
		}

		// Get the average reading by adding back on the reference
		_lastError = (double)errors / (double)_ADCaverages + (double)referenceVal;

		// Scale to +- 1
		_lastError = _lastError / 32767.0;
		
		// Apply calibration if we're in differential mode
		if (_ADCDifferential) {
			if (_ADCport==0)
				_lastError = _lastError + ADC_0to1_CAL;
			else if (_ADCport==2)
				_lastError = _lastError + ADC_2to3_CAL;
		}
		

		// Return result
		return _lastError;
	}

	// Write to the DAC
	// Virtual specs.:
	//		"Writes to the control signal. Values from -1 to 1"
	void ArduinoAnalog::writeCtrl(double val)
	{
		// Coerce the requsted value within bounds
		if (val > _maximumLimit) val = _maximumLimit;
		if (val < _minimumLimit) val = _minimumLimit;

		_currentOutput = val;
		ArduinoAnalog::writeToDAC(_DACport, val);
	}

	// Return the current output
	// Virtual specs.:
	//		"Get the current output. Values from -1 to 1."
	double ArduinoAnalog::recallCtrl()
	{
		return _currentOutput;
	}

	// Set software limits on the max/min ctrl signal
	void ArduinoAnalog::setLimits(double minimum, double maximum) {

		// Check sanity of inputs
		if (maximum > 1) maximum = 1;
		if (minimum < -1) minimum = -1;
		if (minimum > maximum) minimum = maximum;

		// Store limits
		_minimumLimit = minimum;
		_maximumLimit = maximum;
	}

	// Return the current output
	// Virtual specs.:
	//		"Read out the last measured error. Values from -1 to 1."
	double ArduinoAnalog::recallError()
	{
		return _lastError;
	}

	// Write to an arbitary port on the DAC
	void ArduinoAnalog::writeToDAC(int port, double val) {

		// Filter input
		if (val < -1)
			val = -1;
		else if (val > 1)
			val = 1;

		// Convert -1 to 1 input => 0 to 65535 for the DAC
		unsigned int data = (val + 1) * 65535 / 2;

		// Takes an unsigned int from 0 to 65535 corresponding to 0-5V output
		analog.write(port, data);
	}

}
