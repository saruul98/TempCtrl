#include "StepOutputAndRecord.h"
#include <Arduino.h>

namespace NewTempCtrl {

	// Overload. See header file. 
	StepOutputAndRecord::StepOutputAndRecord(double delta, IOInterface * ControlInterface, LabInterface * LInterface, long periodMillis, long cycles, bool report) :
		TemporaryLooper(ControlInterface, LInterface), _periodMillis(periodMillis), _cycles(cycles), _report(report)
	{
		double currentLevel = ControlInterface->recallCtrl();

		_topLevel = currentLevel + delta;
		_bottomLevel = currentLevel - delta;

		init();
	}

	StepOutputAndRecord::StepOutputAndRecord(double bottomLevel, double topLevel, IOInterface * ControlInterface, LabInterface * LInterface, long periodMillis, long cycles, bool report) :
		TemporaryLooper(ControlInterface, LInterface), _bottomLevel(bottomLevel), _topLevel(topLevel), _periodMillis(periodMillis), _cycles(cycles), _report(report)
	{
		init();
	}

	void StepOutputAndRecord::init()
	{
		// Check validity
		if (_bottomLevel < -1) _bottomLevel = -1;
		if (_topLevel > 1) _topLevel = 1;

		// Initialise _finished to FALSE
		_finished = false;

		// Start the timer
		_startTime = millis();

		// Set _toggles to contain the number of toggles that need to be done, including the first. 
		_toggles = _cycles * 2;

		// Tell the controller that we're currently in the down state so it will flip us to up on the first run
		_up = false;

		// Set _loopStart to millis() + _halfPeriod so that doLoop() will trigger the update
		_loopStart = millis() + _halfPeriod;

    
		// Calculate the time required for a single step
		_halfPeriod = _periodMillis / 2;
	}

	// This is the function that will be run every cycle until finished() returns TRUE
	int StepOutputAndRecord::doLoop() {

		// Record the error signal
		double signal = _ControlInterface->readError();

		// Output the time, control signal and error to the lab if requested
		if (_report) {
			reportData();
		}

		// If we've been in the current state long enough now, toggle to the next
		if ((millis() - _loopStart) >= _halfPeriod) {

			// If there are no toggles left to be done, quit this loop and tell the controller to take over again. 
			if (_toggles <= 0) {
				_finished = true;

				// Report finished if this was a query
				if (_report)
					_LabInterface->sendOut("Done");

				return 1;
			}

			// Toggle state:
			if (_up) {
				goDOWN();
				_up = false;
			}
			else {
				goUP();
				_up = true;
			}

			// Count the toggle
			_toggles--;

			// Reset timer
			_loopStart = millis();
		}
	}

	// Print the current data
	void StepOutputAndRecord::reportData() {

		long time = millis() - _startTime;
		double signal = _ControlInterface->recallError();
		double level = (_up ? _topLevel : _bottomLevel); // Top level or bottom level depending on status (stored in _up)

		_LabInterface->print(time);
		_LabInterface->print(", ");
		_LabInterface->printFlt(level);
		_LabInterface->print(", ");
		_LabInterface->printFlt(signal);

		_LabInterface->sendOut("");
	}

	void StepOutputAndRecord::goUP() {
		_ControlInterface->writeCtrl(_topLevel);
	}
	void StepOutputAndRecord::goDOWN() {
		_ControlInterface->writeCtrl(_bottomLevel);
	}

	StepOutputAndRecord::~StepOutputAndRecord()
	{
	}
}
