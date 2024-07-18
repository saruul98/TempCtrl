#pragma once
#include "TemporaryLooper.h"
#include "IOInterface.h"
#include "LabInterface.h"

namespace NewTempCtrl {

	// TemporaryLooper child that provides methods for stepping the control variable from one value to another then returning it
	//    and outputting the error signal as this happens. 
	class StepOutputAndRecord :
		public TemporaryLooper
	{
	public:
		// Initialise by passing upper level and lower level
		StepOutputAndRecord(double bottomLevel, double topLevel, IOInterface * ControlInterface, LabInterface * LInterface, long periodMillis, long cycles = 1, bool report = true);

		// Alternatively, initialise by passing delta: the amount to go above and below the current control level
		StepOutputAndRecord(double delta, IOInterface * ControlInterface, LabInterface * LInterface, long periodMillis, long cycles = 1, bool report = true);

		// Report state. Write into str a string that identifies the currently running algorithm. -1 for error
		int reportState(char* ptr) { strcpy(ptr, "Looper:Tuning"); }

		// Print data.
		void reportData();

		~StepOutputAndRecord();

	private:
		// Initiate. Annoying that I have to have this, but overloading of constructors is rubbish in c++
		void init();
		long _halfPeriod; // To be calculated at initiation, saving recalculation each loop. 

	protected:

		bool _report;
		int doLoop();
		bool finished() { return _finished; }

		/// <summary>
		/// The bottom level for the step function
		/// </summary>
		double _bottomLevel;
		/// <summary>
		/// The top level for the step function
		/// </summary>
		double _topLevel;
		/// <summary>
		/// The period of the step function in milliseconds
		/// </summary>
		long _periodMillis;

		/// <summary>
		/// The number of cycles to do. One cycle is up, down. 
		/// </summary>
		long _cycles;

		/// <summary>
		/// When the current step began
		/// </summary>
		unsigned long _loopStart;

		/// <summary>
		/// t=0 reference point for the sequence: when the first step began in microseconds. 
		/// </summary>
		unsigned long _startTime;

		/// <summary>
		/// Have we finished?
		/// </summary>
		bool _finished;

		/// <summary>
		/// How many toggles remain to be done
		/// </summary>
		long _toggles;
		/// <summary>
		/// Are we currently on the up step (TRUE) or the down step (FALSE)?
		/// </summary>
		bool _up;

		// Go to the up state
		void goUP();
		// Go to the down state
		void goDOWN();

	};

}
