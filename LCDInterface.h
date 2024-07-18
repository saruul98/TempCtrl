#pragma once

#include <TFT.h>
#include <SPI.h>

namespace NewTempCtrl {

	/// <summary>
	///		Object for interfacing with an Arduino TFT screen
	/// </summary>
	class LCDInterface
	{
	public:
		
		// Initiate the TFT screen
		LCDInterface(int backlightPin, int CS, int DC, int RST);
		// Initiate the TFT screen with a hardwired reset pin. 
		LCDInterface(int backlightPin, int CS, int DC) {
			LCDInterface(backlightPin, CS, DC, 0);
		}

		~LCDInterface(){}

		/// <summary>
		///		Writes text to the screen.
		/// </summary>
		/// <param name="text">The text to write.</param>
		void writeText(char* text);

	private:
		int _backlightPin;
		// Screen object
		TFT * _myScreen;

	};

}
