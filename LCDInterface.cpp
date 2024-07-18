#include "LCDInterface.h"

namespace NewTempCtrl {

	LCDInterface::LCDInterface(int backlightPin, int CS, int DC, int RST)
	{
		// Store the backlight pin
		_backlightPin = backlightPin;

		// Set the backlight pin to an output and turn it on
		pinMode(_backlightPin, OUTPUT);
		digitalWrite(_backlightPin, HIGH);

		// Initiate the screen object
		_myScreen = new TFT(CS, DC, RST);
		_myScreen->begin();

		// Black bg
		_myScreen->background(0,0,0);

		// White text
		_myScreen->fill(255, 255, 255);

		// Write to the screen
		_myScreen->text("Hello!", 0, 0);

	}

	void LCDInterface::writeText(char* text) {
		// Write to the screen
		_myScreen->text(text, 0, 0);
	}

}