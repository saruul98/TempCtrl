#pragma once

namespace NewTempCtrl {
	/// <summary>
	///		Object to allow a <see cref="Controller"/> object to interface with the world.
	///		Supplies methods for reading an error signal and writing a control signal.
	///		N.B. This is a virtual class: it defines the required characteristics but for actual usage 
	///		a child class must be created specific to a particular implementation. See <see cref="ArduinoAnalog"/> for an example.	
	/// </summary>
	class IOInterface
	{
	public:

		// Default constructor
		IOInterface(){}

		// Make this abstract
		virtual ~IOInterface(){}

		// Reads the current value of the error signal. Values from -1 to 1
		virtual double readError() = 0;

		// Writes to the control signal. Values from -1 to 1
		virtual void writeCtrl(double val) = 0;

		// Read out the current output. Values from -1 to 1.
		virtual double recallCtrl() = 0;

		// Read out the last measured error. Values from -1 to 1.
		virtual double recallError() = 0;

		// Set software limits on the max/min ctrl signal
		virtual void setLimits(double minimum, double maximum) = 0;

	};
}
