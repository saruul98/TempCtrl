#include "Controller.h"

namespace NewTempCtrl {

	// Initial number of controllers = 0
	int Controller::numberOfControllers = 0;

	Controller::Controller(IOInterface * interface, Algorithm * algorithm) :
		_interface(interface), _algorithm(algorithm)
	{
		// Initialise the temporary looper to the null pointer (i.e. no temporary looper at first)
		_tempLooper = 0;

		// Set the ID to the next available number
		_ID = ++numberOfControllers;
	}


	/// <summary>
	/// Does one cycle of the loop.
	/// Reads error, performs calculation and outputs result
	/// </summary>
	/// <returns></returns>
	int Controller::doLoop() {

		// Is there a temporaryLooper object waiting to be executed?
		// If so, run it
		if (temporaryLooperPresent()) {

			// Has it finished? If so, delete it
			if (temporaryLooperFinished())
			{
				deleteTemporaryLooper();
				return 2;
			}
			else
			{
				runTemporaryLooper();
				return 1;
			}

		}
		else { // If not, run the normal loop:

			// Read the error signal
			double error = _interface->readError();

			// Calculate the new control signal
			double signal = _algorithm->output(error);

			// Output the new value of the control signal
			_interface->writeCtrl(signal);

			return 0;
		}
	}

}
