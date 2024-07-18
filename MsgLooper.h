#pragma once
#include "TemporaryLooper.h"
#include "IOInterface.h"
#include "LabInterface.h"

namespace NewTempCtrl {

	class MsgLooper :
		public TemporaryLooper
	{
	public:
		MsgLooper(IOInterface * ControlInterface, LabInterface * LInterface) :
			TemporaryLooper(ControlInterface, LInterface)
		{
			_finished = false;
			_delay = 3000;
		}

		~MsgLooper() {
			IOInterface * test = _ControlInterface; 
		};

		// Report state. Send a string that identifies the currently running algorithm
		int reportState(char* ptr) { strcpy(ptr, "Looper:TestMsg"); }

	protected:
		
		int _counter;
		int _delay;
		bool _finished;
		
		public:
		int doLoop() {
	
			_LabInterface->sendOut("Intercepted! Running tests:");
		
			// Just a test float
			_LabInterface->print("Float: ");
			_LabInterface->printFlt((float)-1.2345);
			_LabInterface->sendOut(""); // newline
		
			// And a test int!
			_LabInterface->print("Pausing for (int): ");
			_LabInterface->print(_delay);
			_LabInterface->sendOut(""); // newline
		
			delay(_delay);
		
			_LabInterface->sendOut("Returning");
			_finished = true;

		}
		
		bool finished() { return _finished; }

	};

}
