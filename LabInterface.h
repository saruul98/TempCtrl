#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

#define DEFAULT_TIMEOUT 1000

namespace NewTempCtrl {

	/// <summary>
	///		This object defines an interface from the controller to the lab's computer systems. 
	///     By default this is a serial connection but all methods are virtual so that this object can be
	///		reimplemented as needed. 
	/// </summary>
	class LabInterface
	{

	public:

		LabInterface(long baudrate = 9600) { Serial.begin(baudrate); _timeout = DEFAULT_TIMEOUT; }
		~LabInterface() {}

		// Is there an incoming command from Labview?
		virtual bool commandWaiting() { return Serial.available(); }

		// Read one byte from the stack
		virtual char readFirst() {

			if (!Serial.available()) waitForAvailable();

			return Serial.read();
		}

		// Peek at the first byte from the stack (without removing it).
		// Wait timeout milliseconds (default 1000) if there's nothing there then return -1 if still nothing
		virtual char peekFirst() {

			if (!Serial.available()) waitForAvailable();

			// No data, so return -1
			return Serial.peek();
		}

		// Read x bytes from the stack into ptr. Returns the number of bytes read
		virtual int readBytes(char * ptr, int x) { return (int)Serial.readBytes(ptr, x); }

		// Read up to x bytes from the stack into ptr, until the terminator char is found. Returns the number of bytes read
		virtual int readBytesUntil(char termChar, char * ptr, int x) { return (int)Serial.readBytesUntil(termChar, ptr, x); }

		// Read the next number on the input, formatted as an integer string. All other data will be discarded
		virtual int readInt() { return Serial.parseInt(); }

		// Read the next number on the input, formatted as a long integer string. All other data will be discarded
		virtual long readLong();

		// Read the next number on the input, formatted as a float string. All other data will be discarded
		virtual double readFlt() { return Serial.parseFloat(); }

		// Send a string to the computer, terminated by a newline
		template <class t>
		int sendOut(t output) { print(output); print('\n'); }

		// Send a string to the computer, no termination
		template <class t>
		int print(t output) { Serial.print(output); }

		// Send a double, formatted as a string, no termination
		virtual int printFlt(double output, int len = 9, int precision = 7) {
			// Allocate space for string
			char * str = (char*)malloc(sizeof(char) * (len + 3));

			// Do conversion
			dtostrf(output, len, precision, str);

			// Send the output
			LabInterface::print(str);

			// Free the used space
			free(str);
		}

		// Send a float, formatted as a string, no termination
		virtual int printFlt(float output, int len = 9, int precision = 7) { printFlt((double)output, len, precision); }

		// Flush the input buffer, discarding any unread data
		virtual void flushAll() { while (Serial.available()) { Serial.read(); } Serial.flush(); }

		// Send raw data to the computer (no termination)
		template <class t>
		int sendRaw(t output) {
			// Write the raw data bitwise
			byte * cast;
			cast = (byte*)&output;
			Serial.write(cast, sizeof(t));
		}

		// Initiate stdout so that printf prints to the serial connection
		virtual int setupStdout(void);

		void setTimeout(int timeout) { _timeout = timeout; }

	private:

		int _timeout;

		// Wait for _timeout milliseconds or until data is available, whichever is sooner
		void waitForAvailable() {
			int endTime = millis() + _timeout;

			while (millis() < endTime) {
				if (Serial.available()) return;
			}
		}
	};


}
