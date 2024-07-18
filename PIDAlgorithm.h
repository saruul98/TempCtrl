#pragma once
#include "Algorithm.h"
#include "Arduino.h"

namespace NewTempCtrl {
	class PIDAlgorithm :
		public Algorithm
	{
	public:
		PIDAlgorithm(double K, double Ti, double Td, double initialOutput, double target = 0, double N = 10, bool disableProportional = false, int timingCycles = 100, bool debug = false);

		~PIDAlgorithm();

		// This algorithm DOES lock
		bool lockingAlgo() { return true; }

		// Do the PID calculation
		double output(double input);

		// Set the output manually
		void setOutput(double output);

		// Change the setpoint
		void setSetpoint(double setpoint);

		// Return the setpoint
		double getSetpoint() { return _target; }

		// Change the gain K
		void setGain(double gain);

		// Reset algorithm
		void resetAlgo();

	protected:
		// Calculate the combined constants based on the number of us per cycle
		int recalculateConsts(unsigned long microseconds);

		// Local variables for the calculation
		double _output, _deltaDold;
		double _old, _old2;
		bool _debug, windup_mode;
		double _initOut;
		double error, error_old, _I_old;
		double deltaU, deltaP, deltaI, deltaD, P, I, D;

		// Counter: the cycle is timed every 100 loops
		int _timingCount;
		// Start and end of loop
		unsigned long _timerStart, _timerEnd;

		// Is this the first run?
		bool _firstRun;

		// Calculated, combined values of the constants
		// These are calculated in recalculateConsts() and depend on the time for one cycle
		double _Kp, _Ki, _Kd1, _Kd2;

		// These are the constants input by the user to use for the PID calc:
		double _K, _Ti, _Td, _N, _target;
		int _timingCycles;
		bool _disableProportional;

		// Report state. Send a string that identifies the currently running algorithm
		int reportState(char* ptr) { strcpy(ptr, "Algo:PID"); }
	};


}
