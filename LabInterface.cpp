#include "LabInterface.h"

namespace NewTempCtrl {

	// Global stuff:

	// Function that printf and related will use to print
	int serial_putchar(char c, FILE* f) {
		if (c == '\n') serial_putchar('\r', f);
		return Serial.write(c) == 1 ? 0 : 1;
	}

	// Stream for stdout
	FILE serial_stdout;

	// End globals

	int LabInterface::setupStdout(void) {

		// Setup the stream so that it calls serial_stdout for char output
		fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);

		// Redefine stdout
		stdout = &serial_stdout;
	}

	long LabInterface::readLong() {

		bool stop = false;
		char c;

		while (!stop) {
			if (0 == Serial.readBytes(&c, 1)) { return -1; }

			if (isdigit(c) || c == '-') { // If digit or minus sign
				stop = true;
				// Stop and move to number processing
			}
		}

		char str[100];
		int i = 0;
		stop = false;

		while (!stop) {

			str[i] = c;

			if ((0 == Serial.readBytes(&c, 1)) || !(isdigit(c) || c == '-') || i > 98) { // If NOT digit or minus sign
				stop = true;
				str[i + 1] = 0; // Write null char
				// Stop
			}

			i++;
		}

		// Get long
		return strtol(str, 0, 0);
	}

}
