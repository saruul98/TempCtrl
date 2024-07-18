#pragma once
#include "Algorithm.h"

namespace NewTempCtrl {

	// An Alorithm that will output simply a constant, ignoring input. 
	class ConstantOut :
		public Algorithm
	{
	public:

		ConstantOut(double level) : _constLevel(level), Algorithm() {}

		// Do the locking calculation. Input should be a double from -1 to 1
		double output(double input) { return _constLevel; }

		// Report state. Send a string that identifies the currently running algorithm
		int reportState(char* ptr) { strcpy(ptr, "Algo:Const"); }

		// Set the output to a specified level, smoothly if possible
		void setOutput(double output) {
			_constLevel = output;
		}

		// Change setpoint and gain: these do nothing since this object does not lock
		void setSetpoint(double setpoint) {}
		void setGain(double gain) {}

		void resetAlgo() {}

		// Get setpoint: return 0
		double getSetpoint() { return 0; }

	protected:

		// The level to be output
		double _constLevel;

		~ConstantOut()
		{
		}
	};

}