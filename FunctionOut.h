#pragma once
#include "Algorithm.h"
#include "Arduino.h"

namespace NewTempCtrl {
	// An Algorithm that will output a function, ignoring input. 
	class FunctionOut :
		public Algorithm
	{
	public:

		FunctionOut(int type, double frequency, double amplitude);

		~FunctionOut();


		// Do the locking calculation. Input should be a double from -1 to 1
		double output(double input);

		// Report state. Send a string that identifies the currently running algorithm
		int reportState(char* ptr) { strcpy(ptr, "Algo:Func"); }

		void setOutput(double output);
		bool lockingAlgo() { return false; }
		void setSetpoint(double setpoint);
		double getSetpoint() { return 0; }
		void setGain(double gain);
		void resetAlgo();

	protected:

		// The level to be output
		double _type, _freq, _amp;
		double pi = 3.14159265359;
		bool _firstRun;
		double _output;
		int _timingCount;
		unsigned long _timerStart, _timerEnd;


	};

}