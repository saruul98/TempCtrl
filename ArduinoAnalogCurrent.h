#pragma once
#include "ArduinoAnalog.h"
namespace NewTempCtrl {

	class ArduinoAnalogCurrent :
		public ArduinoAnalog
	{
	public:
		ArduinoAnalogCurrent(int ADCport, int DACport, int DACCurrentLimitPort, bool ADCDifferential = false, int ADCaverages = 3);
		ArduinoAnalogCurrent(ArduinoAnalogCurrent * rhs);
		~ArduinoAnalogCurrent();

		// Write a constant current of the given value (in amps)
		void writeCtrl(double val);

	protected:
		int _DACCurrentLimitPort;
	};

}
