#pragma once
#include "IOInterface.h"
#include "analogShield.h"

/*
-  The two calibration values below are used to compensate for 
	zero offsets of the error readings. 
-  They are measured by connection the error signal inputs
	together and getting the output (should be zero with no error)
-  Values are scaled from -1 to 1 (i.e. a +1.25V correction is "0.25")
*/
#define ADC_0to1_CAL +0.00095859
#define ADC_2to3_CAL +0.0027989

namespace NewTempCtrl {
	class ArduinoAnalog :
		public IOInterface
	{
	public:
		// Save the ADC and DAC ports (0, 1, 2 or 3)
		ArduinoAnalog(int ADCport, int DACport, bool ADCDifferential = false, int ADCaverages = 10000);

		ArduinoAnalog(ArduinoAnalog * rhs);

		~ArduinoAnalog(){}

		// Reads the current value of the error signal
		virtual double readError();

		// Writes to the control signal
		virtual void writeCtrl(double val);

		// Return the current output
		virtual double recallCtrl();

		// Return the last measured error
		virtual double recallError();

		// Set software limits on the max/min ctrl signal
		virtual void setLimits(double minimum, double maximum);

		// Write to an arbitary port on the DAC
		// This should NOT be used to write to the error signal, that's what writeCtrl() is for. 
		// This function is for writing to ports other than those used for control, e.g. to control the current limits on an OPA549.
		// As a static member, it can be called without creating an instance of the class. 
		static void writeToDAC(int port, double val);

	protected:
		double _currentOutput;
		double _lastError;

		double _minimumLimit, _maximumLimit;

		int _ADCport, _DACport, _ADCaverages;
		bool _ADCDifferential;
	};

}
