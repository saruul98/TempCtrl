#pragma once

#include "IOInterface.h"
#include "LabInterface.h"

namespace NewTempCtrl {

	/// <summary>
	///		Class for defining actions that need to take place instead of the loop, for a short time, and then
	///		the controller reverts to locking as normal.
	/// </summary>
	class TemporaryLooper
	{

	public:
		// Initiate the temporary loop with the IOInterface and LabInterface
		TemporaryLooper(IOInterface * ControlInterface, LabInterface * LInterface) :
			_ControlInterface(ControlInterface), _LabInterface(LInterface)
		{}

		// Does one loop of this temporary function
		virtual int doLoop() =0;

		// Returns true once the temporary loop has finished & control should return to the normal locking loop
		virtual bool finished() =0;

		// Report state. Send a string that identifies the currently running algorithm
		virtual int reportState(char* ptr) = 0;

		// Print any relevant data to the output
		virtual void reportData() {}

		virtual ~TemporaryLooper()
		{}

	protected:

		// IO interface to deal with the ADC / DAC
		IOInterface * _ControlInterface;

		LabInterface * _LabInterface;
	};

}
