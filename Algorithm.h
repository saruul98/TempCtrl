#pragma once

namespace NewTempCtrl {

	class Algorithm
	{
	public:

		Algorithm()
		{}

		// Do the locking calculation. Input should be a double from -1 to 1
		virtual double output(double input) = 0;
		
		// Report state. Send a string that identifies the currently running algorithm
		virtual int reportState(char* ptr) = 0;

		// Set the output to a specified level, smoothly if possible
		virtual void setOutput(double output) = 0;

		// Change setpoint, smoothly if possible
		virtual void setSetpoint(double setpoint) = 0;

		// Change gain, smoothly if possible
		virtual void setGain(double gain) = 0;

		// Reset algorithm
		virtual void resetAlgo() = 0;

		// Return setpoint
		virtual double getSetpoint() = 0;

		// Does this algorithm lock? Default no
		virtual bool lockingAlgo() { return false; }
	};

}
