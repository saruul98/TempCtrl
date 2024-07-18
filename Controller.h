	#pragma once

#include "IOInterface.h"
#include "Algorithm.h"
#include "TemporaryLooper.h"

namespace NewTempCtrl {

	///<summary>
	/// Object to manage one locking loop. 
	/// This object contains pointers to the <see cref="IOInterface"/> used to send / receive error
	/// signals and control signals respectivly, and to the <see cref="Algorithm"/> used to do the
	/// calculation. 
	///</summary>
	class Controller
	{
	public:

		// Global var to count how many controllers exist (for setting their ID)
		static int numberOfControllers;

		Controller(IOInterface * interface, Algorithm * algorithm);

		// Do one cycle for this controller
		int doLoop();

		/// <summary>
		/// Gets the interface.
		/// </summary>
		/// <returns></returns>
		IOInterface * getInterface() { return _interface; }

		/// <summary>
		/// Gets the algorithm.
		/// </summary>
		/// <returns></returns>
		Algorithm * getAlgorithm() { return _algorithm; }

		void replaceInterface(IOInterface * interface) {
			delete(_interface);
			_interface = interface;
		}

		void replaceAlgorithm(Algorithm * algorithm) {
			delete(_algorithm);
			_algorithm = algorithm;
		}

		// Add a temporaryLooper object to this controller so that it will intercept the next loop runthrough
		TemporaryLooper * addTemporaryLooper(TemporaryLooper * newLooper) {
			return _tempLooper = newLooper;
		}

		// Is there a temporaryLooper present?
		bool temporaryLooperPresent() { return (bool)(_tempLooper); }

		// Has the looper finished?
		bool temporaryLooperFinished() { return _tempLooper->finished(); }

		// Delete the looper
		void deleteTemporaryLooper() { delete(_tempLooper); _tempLooper = 0; }

		// Do a loop of the TemporaryLooper
		void runTemporaryLooper() { _tempLooper->doLoop(); }

		// Return a pointer to the temporary looper
		TemporaryLooper * getTemporaryLooper() { return _tempLooper; }

		// Report state. Write into ptr a string that identifies the currently running loop. Returns -1 for an error. 
		// This is delegated to whichever object is currently 
		//		running: the temporary looper or the algorithm
		int reportState(char* ptr) { return (temporaryLooperPresent() ? _tempLooper->reportState(ptr) : _algorithm->reportState(ptr)); }

		// Set the output manually
		void setOutput(double output) { _algorithm->setOutput(output); _interface->writeCtrl(output); }

		// Change the setpoint
		void setSetpoint(double setpoint) { _algorithm->setSetpoint(setpoint); }

		// Report the Ctrl and Error signals
		void reportData();

		// Report the ID
		int getID() { return _ID; }

	protected:

		// Temporary looper, to be given by user if needed
		// Initialised to the null pointer
		TemporaryLooper * _tempLooper;

		// IO interface to deal with the ADC / DAC
		IOInterface * _interface;

		// Algorithm object to handle the calculation
		Algorithm * _algorithm;

		// integer ID
		int _ID;
	};

}
