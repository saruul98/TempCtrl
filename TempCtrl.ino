/*-------------------------------------------------------------------
-																	-
-		PID Temperature Controller									-
-			Saruul Nasanjargal										-
-																	-
-		Version:													-
-*/
#define REV_VERSION "v2.10"									
#define IDENTIFIER "ARDUINO PID"
/*																	-
-		Controller to manage several processes, correcting them		-
-		via a PID algorithm.										-
-																	-
-		By default, this controller communicates by USB-serial		-
-		connection. The Serial connection expects:					-
-			Baud rate: 57600										-
-			Data: 8 bits, Parity: NO, Stop bits: 1					-
-		Comms must be terminated by the '\n' char.					-
-																	-
--------------------------------------------------------------------*/


#include "Controller.h"
#include "LabInterface.h"
#include "PIDAlgorithm.h"
#include "ArduinoAnalog.h"
#include "ArduinoAnalogCurrent.h"
#include "ArduinoAnalogI2C.h"
#include "MsgLooper.h"
#include "StepOutputAndRecord.h"
#include "ConstantOut.h"
#include "FunctionOut.h"

#include <avr/pgmspace.h>

// Not used here, but these are needed by class files so must be included here first
#include "analogShield.h"
#include <Wire.h>
#include <TFT.h>
#include <SPI.h>


using namespace NewTempCtrl;

LabInterface* LInterface;
ArduinoAnalog *DAC_Interface1, *DAC_Interface2, *DAC_Interface3;
ArduinoAnalogCurrent *Current_Interface1, *Current_Interface2, *Current_Interface3;
ArduinoAnalogI2C *I2C_Interface1, *I2C_Interface2, *I2C_Interface3, *I2C_Interface4, *I2C_Interface5, *I2C_Interface6;
Controller *sensor1, *sensor2, *sensor3, *sensor4, *sensor5, *sensor6, *sensor7, *sensor8;
Controller *peltier1, *peltier2, *peltier3;

// Declare a reset function. This is equivalent to pressing the RST button
void(*resetFunc) (void) = 0;

// Declare the time variable.
unsigned long time;

// Declare variable for intermitent control signal generation.
int n = 0;


// Read in the target controller as an int from the serial connection
// and return an actual controller object, or NULLPTR if invalid
Controller * getController(uint8_t interface);

void setup()
{
LInterface = new LabInterface(57600); // Start the serial connection
Wire.begin(); // Activate I2C link

DAC_Interface1 = new ArduinoAnalog(0, 0, true, 10000);
DAC_Interface2 = new ArduinoAnalog(2, 1, true, 10000);
DAC_Interface3 = new ArduinoAnalog(2, 2, true, 10000);

I2C_Interface1 = new ArduinoAnalogI2C(1, 16, 50, 1);
I2C_Interface2 = new ArduinoAnalogI2C(1, 16, 50, 2);
I2C_Interface3 = new ArduinoAnalogI2C(2, 9, 50, 1);
I2C_Interface4 = new ArduinoAnalogI2C(2, 9, 50, 2);
I2C_Interface5 = new ArduinoAnalogI2C(3, 9, 50, 1);
I2C_Interface6 = new ArduinoAnalogI2C(3, 9, 50, 2);

Current_Interface1 = new ArduinoAnalogCurrent(0, 3, 0, true, 10000);
Current_Interface2 = new ArduinoAnalogCurrent(2, 3, 1, true, 10000);
Current_Interface3 = new ArduinoAnalogCurrent(2, 3, 2, true, 10000);

PIDAlgorithm * algoPID = new PIDAlgorithm( /* K = */ -4.0, 
											/* Ti = */ 100,
											/* Td = */ 0.0,
											/* initialOutput=*/ 0.0,
											/* target=*/ 17.0,
											/* N=*/ 10,
											/* disableProportional=*/ false);

ConstantOut * algoConst0 = new ConstantOut(-1.0);
FunctionOut * algoSine = new FunctionOut(1, 0.0008333333, 1.0);

peltier1 = new Controller(Current_Interface1, algoPID);
peltier2 = new Controller(Current_Interface2, algoPID);
peltier3 = new Controller(Current_Interface3, algoPID);

sensor1 = new Controller(DAC_Interface1, algoConst0); // E1
sensor2 = new Controller(DAC_Interface2, algoConst0); // E2
sensor7 = new Controller(I2C_Interface1, algoConst0); // E7
sensor4 = new Controller(I2C_Interface2, algoConst0); // E4
sensor5 = new Controller(I2C_Interface3, algoConst0); // V_bridge
sensor6 = new Controller(I2C_Interface4, algoConst0); // E6
sensor3 = new Controller(I2C_Interface5, algoConst0); // E3
sensor8 = new Controller(I2C_Interface6, algoConst0); // E8

}




void loop()
{

time = millis();

// OBTAIN ERROR SIGNALS:
double error1 = sensor1->getInterface()->readError();
double error2 = sensor2->getInterface()->readError();
double error3 = sensor3->getInterface()->readError();
double error4 = sensor4->getInterface()->readError();
double error5 = sensor5->getInterface()->readError(); // V_bridge
double error6 = sensor6->getInterface()->readError();
double error7 = sensor7->getInterface()->readError();
double error8 = sensor8->getInterface()->readError();
// double error9 = sensor9->getInterface()->readError();

// CONVERT -1 to +1 SIGNAL to -5V to +5V SIGNAL:
double error1volts = error1*5.0;
double error2volts = error2*5.0;
double error3volts = error3*5.0;
double error4volts = error4*5.0;
double error5volts = error5*5.0; // V_bridge
double error6volts = error6*5.0;
double error7volts = error7*5.0;
double error8volts = error8*5.0;
// double error9volts = error9*5.0;

// BRIDGE THERMISTOR CALCULATION:
double v_bridge = error5volts ;
double r_1 = 10000.0, r_s = 32200.0 ;
double res1 = r_1*(v_bridge*r_s+(error1volts)*(r_1+r_s) ) / ( v_bridge*r_1-(error1volts)*(r_1+r_s) );
double res2 = r_1*(v_bridge*r_s+(error2volts)*(r_1+r_s) ) / ( v_bridge*r_1-(error2volts)*(r_1+r_s) );
double res3 = r_1*(v_bridge*r_s+(error3volts)*(r_1+r_s) ) / ( v_bridge*r_1-(error3volts)*(r_1+r_s) );
double res4 = r_1*(v_bridge*r_s+(error4volts)*(r_1+r_s) ) / ( v_bridge*r_1-(error4volts)*(r_1+r_s) );
double res5 = r_1*(v_bridge*r_s+(error5volts)*(r_1+r_s) ) / ( v_bridge*r_1-(error5volts)*(r_1+r_s) );
double res6 = r_1*(v_bridge*r_s+(error6volts)*(r_1+r_s) ) / ( v_bridge*r_1-(error6volts)*(r_1+r_s) );
double res7 = r_1*(v_bridge*r_s+(error7volts)*(r_1+r_s) ) / ( v_bridge*r_1-(error7volts)*(r_1+r_s) );
double res8 = r_1*(v_bridge*r_s+(error8volts)*(r_1+r_s) ) / ( v_bridge*r_1-(error8volts)*(r_1+r_s) );

// INA330 THERMISTOR CALCULATION:
// double v_excite = 1.0, r_g = 200000.0, r_s = 32200 ;
// double res1 = (v_excite * r_g * r_s) / (v_excite * r_g + error1volts * r_s) ;
// double res2 = (v_excite * r_g * r_s) / (v_excite * r_g + error2volts * r_s) ;
// double res3 = (v_excite * r_g * r_s) / (v_excite * r_g + error3volts * r_s) ;
// double res4 = (v_excite * r_g * r_s) / (v_excite * r_g + error4volts * r_s) ;
// double res5 = (v_excite * r_g * r_s) / (v_excite * r_g + error5volts * r_s) ;
// double res6 = (v_excite * r_g * r_s) / (v_excite * r_g + error6volts * r_s) ;
// double res7 = (v_excite * r_g * r_s) / (v_excite * r_g + error7volts * r_s) ;
// double res8 = (v_excite * r_g * r_s) / (v_excite * r_g + error8volts * r_s) ;

// TEMPERATURE FROM THERMISTOR RESISTANCE 5-TH ORDER FIT:
double a0 = 66.85950083863566;
double a1 = -0.006759863400544356;
double a2 = 3.461791618973437e-7;
double a3 = -1.024460739681071e-11;
double a4 = 1.539400049567137e-16;
double a5 = -9.06549180355199e-22;
double temp1 = a0 + a1*res1 + a2*pow(res1,2.0) + a3*pow(res1,3.0)+ a4*pow(res1,4.0) + a5*pow(res1,5.0);
double temp2 = a0 + a1*res2 + a2*pow(res2,2.0) + a3*pow(res2,3.0)+ a4*pow(res2,4.0) + a5*pow(res2,5.0);
double temp3 = a0 + a1*res3 + a2*pow(res3,2.0) + a3*pow(res3,3.0)+ a4*pow(res3,4.0) + a5*pow(res3,5.0);
double temp4 = a0 + a1*res4 + a2*pow(res4,2.0) + a3*pow(res4,3.0)+ a4*pow(res4,4.0) + a5*pow(res4,5.0);
double temp5 = a0 + a1*res5 + a2*pow(res5,2.0) + a3*pow(res5,3.0)+ a4*pow(res5,4.0) + a5*pow(res5,5.0);
double temp6 = a0 + a1*res6 + a2*pow(res6,2.0) + a3*pow(res6,3.0)+ a4*pow(res6,4.0) + a5*pow(res6,5.0);
double temp7 = a0 + a1*res7 + a2*pow(res7,2.0) + a3*pow(res7,3.0)+ a4*pow(res7,4.0) + a5*pow(res7,5.0);
double temp8 = a0 + a1*res8 + a2*pow(res8,2.0) + a3*pow(res8,3.0)+ a4*pow(res8,4.0) + a5*pow(res8,5.0);

// CONTROL SIGNAL DEFINITIONS:
double control1 = peltier1->getAlgorithm()->output(temp1);
double control2 = peltier2->getAlgorithm()->output(temp2);
double control3 = peltier3->getAlgorithm()->output(temp3);
double control_ = peltier3->getAlgorithm()->output((temp1+temp2+temp3)/3);


// O AMPS CONTROL:
// peltier1->getInterface()->writeCtrl(-1.0);
// peltier2->getInterface()->writeCtrl(-1.0);
// peltier3->getInterface()->writeCtrl(-1.0);

// // 50% CURRENT CONTROL:
// peltier1->getInterface()->writeCtrl(0.0);
// peltier2->getInterface()->writeCtrl(0.0);
// peltier3->getInterface()->writeCtrl(0.0);

// PID CONTROL:
peltier1->getInterface()->writeCtrl(control_);
peltier2->getInterface()->writeCtrl(control_);
peltier3->getInterface()->writeCtrl(control_);


// Debugging (1): Output volts to DAC channels in master.
// float volts = 4.90;
// int dac_port = 3;
// unsigned int dac_data = (((volts/5)+1)/2)*65535;
// analog.write(dac_port, dac_data);

// Debugging (2): Intermitent control signal to watch analog crosstalk.
// float control_ = pow((-1),n);
// n++;
// peltier2->getInterface()->writeCtrl(control_);


Serial.print(time);	Serial.print("\t");

// Serial.print(error1volts, 8);	Serial.print("\t");
// Serial.print(error2volts, 8);	Serial.print("\t");
// Serial.print(error3volts, 8);	Serial.print("\t");
// Serial.print(error4volts, 8);	Serial.print("\t");
// Serial.print(error5volts, 8);	Serial.print("\t");
// Serial.print(error6volts, 8);	Serial.print("\t");
// Serial.print(error7volts, 8); 	Serial.print("\t");
// Serial.print(error8volts, 8); 	Serial.print("\t");

Serial.print(error1volts, 8); Serial.print("\t");
Serial.print(error2volts, 8); Serial.print("\t");
Serial.print(error3volts, 8); Serial.print("\t");
Serial.print(error4volts, 8); Serial.print("\t");
Serial.print(error6volts, 8); Serial.print("\t");
Serial.print(error7volts, 8); Serial.print("\t");
Serial.print(error8volts, 8); Serial.print("\t");
Serial.print(error5volts, 8);  Serial.print("\t");
// Serial.print(control1, 8);  Serial.print("\t");
// Serial.print(control2, 8);  Serial.print("\t");
Serial.print(control_, 8);  Serial.println("");

}


// This function is called by the Arduino when it receives serial data input. 
// If data is received while loop() is running, this function will execute
// after loop() finishes.
void serialEvent() {

	char *commandStr;
	int commandLen;
	bool query;

	// Allocate 65 bytes for commandStr, initing it to 0s
	commandStr = (char*)calloc(65, sizeof(char));

	// Read all the command upto a terminating newline
	commandLen = LInterface->readBytesUntil('\n', commandStr, 64);

	// All commands should be at least 4 bytes long so if we get less than this ignore it. 
	if (commandLen < 4) {

		free(commandStr);

		return; // Return to the normal loop
	}

	// Check if this command is a query
	// If so, set the query flag
	query = (commandLen > 4 && commandStr[commandLen - 1] == '?');

	// Get the first 4 chars of the string: this is the command
	char command[5] = { 0 }; // 5 since we need a null terminator
	strncpy(command, commandStr, 4);

	if (0 == strcmp(command, "*TST")) { // Test the interface
		if (query)
			LInterface->sendOut(F("Query received."));
		else
			LInterface->sendOut(F("Loud and clear!"));
	}
	else if (0 == strcmp(command, "*IDN")) { LInterface->sendOut(IDENTIFIER); } // Send the identity string
	else if (0 == strcmp(command, "*VER")) { LInterface->sendOut(REV_VERSION); } // Send the version number
	else if (0 == strcmp(command, "ERRO") && query) {
		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		if (targetController) {
			// Send last measured error signal as string
			LInterface->print(targetController->getID());
			LInterface->print(", ");
			LInterface->print(millis());
			LInterface->print(", ");
			LInterface->printFlt(targetController->getInterface()->recallError());
			LInterface->sendOut("");
		}
	}
	else if (0 == strcmp(command, "MEAS") && query) {
		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		if (targetController) {
			// Read error signal as string
			LInterface->print(targetController->getID());
			LInterface->print(", ");
			LInterface->print(millis());
			LInterface->print(", ");
			LInterface->printFlt(targetController->getInterface()->readError());
			LInterface->sendOut("");
		}
	}
	else if (0 == strcmp(command, "*RST")) { resetFunc(); } // Reset the Arduino
	else if (0 == strcmp(command, "STAT") && query) { // Return the current status
		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		if (targetController) {
			char * c = (char*)malloc(sizeof(char) * 129); // Assign buffer
			targetController->reportState(c); // Write into buffer
			LInterface->sendOut(c);
			free(c); // free buffer
		}
		else { LInterface->sendOut("Invalid controller"); }
	}
	else if (0 == strcmp(command, "CONT")) { // "CONTrol signal"
		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		if (targetController) {

			if (query) { // Is this a query or a setting?
				// Send current control signal
				LInterface->print(targetController->getID());
				LInterface->print(", ");
				LInterface->printFlt(targetController->getInterface()->recallCtrl());
				LInterface->sendOut("");
			}
			else {
				// Set the output to the given level and spoof a steady state at this level
				double level = LInterface->readFlt();
				targetController->setOutput(level);

				// Output the data we just set
				LInterface->print(targetController->getID());
				LInterface->print(", ");
				LInterface->printFlt(level);
				LInterface->sendOut("");
			}
		}
	}
	else if (0 == strcmp(command, "LIMI")) { // "LIMIt the max ctrl output"

		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		if (targetController) {
			// Read the requested new maximum / minimums
			double mini = LInterface->readFlt();
			double maxi = LInterface->readFlt();

			// Set the limits (the interface does sanity checks so don't do them here)
			targetController->getInterface()->setLimits(mini, maxi);

			// Output the data we just set
			LInterface->print(targetController->getID());
			LInterface->print(", ");
			LInterface->printFlt(mini);
			LInterface->print(", ");
			LInterface->printFlt(maxi);
			LInterface->sendOut("");
		}
	}
	else if (0 == strcmp(command, "SETP")) { // "Change or set the SETPoint"

		// Read the target controller from the serial input
		delay(500);
		Controller * targetController = getController(LInterface->readInt());

		if (targetController) { // If it's a valid object
			if (query) { // Is this a query or a setting?
				// Send current setpoint
				LInterface->print(targetController->getID());
				LInterface->print(", ");
				LInterface->printFlt(targetController->getAlgorithm()->getSetpoint());
				LInterface->sendOut("");
			}
			else {
				// Change the setpoint
				delay(500);
				double setpoint = LInterface->readFlt();
				targetController->setSetpoint(setpoint);
				targetController->getAlgorithm()->resetAlgo();

				// Output the data we just set
				LInterface->print(targetController->getID());
				LInterface->print(F(", New set point: "));
				LInterface->printFlt(setpoint);
				LInterface->sendOut("");
			}
		}
	}
	else if (0 == strcmp(command, "GAIN")) { // "Change the gain of the algorithm"

		// Read the target controller from the serial input
		delay(500);
		Controller * targetController = getController(LInterface->readInt());

		delay(500);
		double gain = LInterface->readFlt();
		targetController->getAlgorithm()->setGain(gain);
		targetController->getAlgorithm()->resetAlgo();

		// Output the data we just set
		LInterface->print(targetController->getID());
		LInterface->print(F(", New gain: "));
		LInterface->printFlt(gain);
		LInterface->sendOut("");
		
	}
	else if (0 == strcmp(command, "ALGO")) { // "Restart the algorithm"

		// Read the target controller from the serial input
		delay(500);
		Controller * targetController = getController(LInterface->readInt());

		targetController->getAlgorithm()->resetAlgo();

		// Output the data we just set
		LInterface->print(targetController->getID());
		LInterface->print(F(" Algorithm reset..."));
		LInterface->sendOut("");
		
	}
	// else if (0 == strcmp(command, "*OFF")) { // "Restart the algorithm"

	// 	// Read the target controller from the serial input
	// 	delay(500);
	// 	Controller * targetController = getController(LInterface->readInt());

	// 	ConstantOut * algoConst0 = new ConstantOut(-1.0);

	// 	targetController->replaceAlgorithm(algoConst0);

	// 	// Output the data we just set
	// 	LInterface->print(targetController->getID());
	// 	LInterface->print(" Turning off...");
	// 	LInterface->sendOut("");
		
	// }
	// else if (0 == strcmp(command, "**ON")) { // "Restart the algorithm"

	// 	// Read the target controller from the serial input
	// 	delay(500);
	// 	Controller * targetController = getController(LInterface->readInt());

	// 	double setpt = targetController->getAlgorithm()->getSetpoint();

	// 	PIDAlgorithm * algoPID = new  PIDAlgorithm(-1, 1000000000000.0, 0.0, 0.0, setpt, 10);

	// 	targetController->replaceAlgorithm(algoPID);
	// 	targetController->getAlgorithm()->resetAlgo();

	// 	// Output the data we just set
	// 	LInterface->print(targetController->getID());
	// 	LInterface->print(" Turning on...");
	// 	LInterface->sendOut("");
		
	// }
	else if (0 == strcmp(command, "TUNE")) {
		// Do a tuningLoop using the values read in subsequently
		// This command takes <lowerLevel> <upperLevel> <period in milliseconds> <number of periods to perform>
		// Each number should be seperated by at least one non-numeric char

		// If this command was a query, report the levels

		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		if (targetController) { // If it's a valid object

			double lowerLevel, upperLevel;
			long period, cycles;

			// Read in the next command which should just be the parameters. 
			lowerLevel = LInterface->readFlt();
			upperLevel = LInterface->readFlt();
			period = LInterface->readLong();
			cycles = LInterface->readLong();

			if (query) { // Report the settings if query mode

				LInterface->print(F("# Injecting looper to controller "));
				LInterface->print(targetController->getID());
				LInterface->sendOut(F(" with:"));

				LInterface->print(F("# LL: "));
				LInterface->printFlt(lowerLevel);
				LInterface->print(F(", UL: "));
				LInterface->printFlt(upperLevel);
				LInterface->print(F(", period: "));
				LInterface->print(period);
				LInterface->print(F(", cycles: "));
				LInterface->sendOut(cycles);
			}
			else { // Report the settings in a more machine friendly manner

				LInterface->print(F("#STEPSettings: "));
				LInterface->print(targetController->getID());
				LInterface->print(F(","));
				LInterface->printFlt(lowerLevel);
				LInterface->print(F(","));
				LInterface->printFlt(upperLevel);
				LInterface->print(F(","));
				LInterface->print(period);
				LInterface->print(F(","));
				LInterface->sendOut(cycles);
			}

			// params: (double delta,                           IOInterface * ControlInterface, LabInterface * LInterface, long periodMicroseconds, int cycles = 1)
			//    or : (double lowerLevel, double upperLevel,   IOInterface * ControlInterface, LabInterface * LInterface, long periodMicroseconds, int cycles = 1)

			// Set up the looper and
			// inject the looper into the controller
			StepOutputAndRecord * tuningLooper = new StepOutputAndRecord(lowerLevel, upperLevel, targetController->getInterface(), LInterface, period, cycles, query);
			targetController->addTemporaryLooper(tuningLooper);

		}
	}
	else if (0 == strcmp(command, "LOCK")) { // CONSTANT VOLTAGE LOCK

		// Do a lock using the values read in subsequently
		// This command takes <K> <Ti> <Td> <N> <initialOutput> <Target>
		// Each number should be separated by at least one non-numeric char

		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		double K, Ti, Td, N, target, initial;

		// Read in the next command which should just be the parameters. 
		K = LInterface->readFlt();
		Ti = LInterface->readFlt();
		Td = LInterface->readFlt();
		N = LInterface->readFlt();
		initial = LInterface->readFlt();
		target = LInterface->readFlt();

		if (targetController) { // If it's a valid object
			if (query) { // Report the settings if query mode

				LInterface->print(F("# VoltageLock - Controller: "));
				LInterface->sendOut(targetController->getID());
				LInterface->print(F("# K: "));
				LInterface->printFlt(K);
				LInterface->print(F(", Ti: "));
				LInterface->printFlt(Ti);
				LInterface->print(F(", Td: "));
				LInterface->printFlt(Td);
				LInterface->print(F(", N: "));
				LInterface->printFlt(N);
				LInterface->print(F(", initial: "));
				LInterface->printFlt(initial);
				LInterface->print(F(", target: "));
				LInterface->printFlt(target);
				LInterface->sendOut("");
			}
			else { // Report the settings in a more machine friendly manner

				LInterface->print(F("#PIDSettings: "));
				LInterface->print(targetController->getID());
				LInterface->print(F(","));
				LInterface->printFlt(K);
				LInterface->print(F(","));
				LInterface->printFlt(Ti);
				LInterface->print(F(","));
				LInterface->printFlt(Td);
				LInterface->print(F(","));
				LInterface->printFlt(N);
				LInterface->print(F(","));
				LInterface->printFlt(initial);
				LInterface->print(F(","));
				LInterface->printFlt(target);
				LInterface->sendOut("");
			}

			// params: (double K, double Ti, double Td, double initialOutput, double target = 0, double N = 10, bool disableProportional = false, int timingCycles = 100, bool debug = false);

			// Set up the algorithm
			// Uncomment for verbose output in query mode
			//		PIDAlgorithm * lockingLooper = new PIDAlgorithm(K, Ti, Td, DAC_Interface->recallCtrl(), target, 10, false, 100, query);
			PIDAlgorithm * lockingLooper = new PIDAlgorithm(K, Ti, Td, initial, target, N);

			// Set up the interface
			ArduinoAnalog * constVoltInterface = new ArduinoAnalog(targetController->getID() == 1 ? DAC_Interface1 : DAC_Interface2);

			// Inject the algorithm into the controller
			targetController->replaceAlgorithm(lockingLooper);

			// Inject the a copy of the constant current interface into the controller
			targetController->replaceInterface(constVoltInterface);

		}
	}
	else if (0 == strcmp(command, "CLOC")) { // CONSTANT CURRENT LOCK

		// Do a lock using the values read in subsequently
		// This command takes <K> <Ti> <Td> <N> <initialOutput> <Target>
		// Each number should be separated by at least one non-numeric char

		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		double K, Ti, Td, N, target, initial;

		// Read in the next command which should just be the parameters. 
		K = LInterface->readFlt();
		Ti = LInterface->readFlt();
		Td = LInterface->readFlt();
		N = LInterface->readFlt();
		initial = LInterface->readFlt();
		target = LInterface->readFlt();

		if (targetController) { // If it's a valid object
			if (query) { // Report the settings if query mode

				LInterface->print(F("# CurrentLock - Controller: "));
				LInterface->sendOut(targetController->getID());
				LInterface->print(F("# K: "));
				LInterface->printFlt(K);
				LInterface->print(F(", Ti: "));
				LInterface->printFlt(Ti);
				LInterface->print(F(", Td: "));
				LInterface->printFlt(Td);
				LInterface->print(F(", N: "));
				LInterface->printFlt(N);
				LInterface->print(F(", initial: "));
				LInterface->printFlt(initial);
				LInterface->print(F(", target: "));
				LInterface->printFlt(target);
				LInterface->sendOut("");
			}
			else { // Report the settings in a more machine friendly manner

				LInterface->print(F("#PIDSettings: "));
				LInterface->print(targetController->getID());
				LInterface->print(F(","));
				LInterface->printFlt(K);
				LInterface->print(F(","));
				LInterface->printFlt(Ti);
				LInterface->print(F(","));
				LInterface->printFlt(Td);
				LInterface->print(F(","));
				LInterface->printFlt(N);
				LInterface->print(F(","));
				LInterface->printFlt(initial);
				LInterface->print(F(","));
				LInterface->printFlt(target);
				LInterface->sendOut("");
			}

			// params: (double K, double Ti, double Td, double initialOutput, double target = 0, double N = 10, bool disableProportional = false, int timingCycles = 100, bool debug = false);

			// Set up the algorithm
			// Uncomment for verbose output in query mode
			//		PIDAlgorithm * lockingLooper = new PIDAlgorithm(K, Ti, Td, DAC_Interface->recallCtrl(), target, N, false, 100, query);
			PIDAlgorithm * lockingLooper = new PIDAlgorithm(K, Ti, Td, initial, target, N);

			// Set up the interface
			ArduinoAnalogCurrent * constCurrInterface = (targetController->getID() == 1 ? Current_Interface1 : Current_Interface2);

			// Inject the algorithm into the controller
			targetController->replaceAlgorithm(lockingLooper);

			// Inject the a copy of the constant current interface into the controller
			targetController->replaceInterface(new ArduinoAnalogCurrent(constCurrInterface));
		}
	}
	else if (0 == strcmp(command, "TUDA") && query) { // "TUneDAta" I know, it's rubbish, I'll fix it later. Reports the tuner's data if it's running

		// Read the target controller from the serial input		
		Controller * targetController = getController(LInterface->readInt());

		if (targetController) { // If it's a valid object

			char * state;
			state = (char*)malloc(129 * sizeof(char));
			targetController->reportState(state); // Load state into 'state'

			if (strcmp(state, "Looper:Tuning") == 0) { // If match:
				targetController->getTemporaryLooper()->reportData();
			}
			else {
				LInterface->sendOut(F("Done"));
			}

			// Free buffer
			free(state);
		}
	}
	else if (0 == strcmp(command, "CURR")) { // CURRent: output the given current. 

		// Read in the controller
		int channel = LInterface->readInt();

		// Read in the requested current in amps (positive or negative)
		double theCurrent = LInterface->readFlt();

		// If we failed, break
		Controller * targetController = getController(channel);
		if (!targetController) {
			free(commandStr);
			return; // Return to the normal loop
		}

		// Create an algorithm that will just output the requested current always
		ConstantOut * constAlgo = new ConstantOut(theCurrent);

		// Inject the controller with the new algorithm to output the current
		targetController->replaceAlgorithm(constAlgo);

		// Inject the controller with a copy of the current interface
		targetController->replaceInterface(new ArduinoAnalogCurrent((channel == 1 ? Current_Interface1 : Current_Interface2)));

		// Write a confirmation
		LInterface->print(F("#Constant Current: "));
		LInterface->print(targetController->getID());
		LInterface->print(",");
		LInterface->printFlt(theCurrent);
		LInterface->sendOut("");
	}
	else if (0 == strcmp(command, "VOLT")) { // VOLTage: output the given voltage. 

		// Read in the controller
		int channel = LInterface->readInt();

		// Read in the requested voltage in scaled units from -1 to 1
		double voltage = LInterface->readFlt();

		// If we failed, break
		Controller * targetController = getController(channel);
		if (!targetController) {
			free(commandStr);
			return; // Return to the normal loop
		}

		// Create an algorithm that will output this voltage
		ConstantOut * constOutputAlgo = new ConstantOut(voltage);

		// Inject the controller with the new algorithm
		targetController->replaceAlgorithm(constOutputAlgo);

		// Write a confirmation
		LInterface->print(F("#Constant Voltage: "));
		LInterface->print(targetController->getID());
		LInterface->print(",");
		LInterface->printFlt(voltage);
		LInterface->sendOut("");
	}
	else {
		// Do nothing if unknown command sent
	}

	// Free memory for commandStr
	free(commandStr);

}

// Read in the target controller as an int from the serial connection
// and return an actual controller object, or NULLPTR if invalid
Controller * getController(uint8_t interface) {

	// Store a pointer to the required controller
	Controller * targetController;
	if (interface == 1) {
		targetController = peltier1;
	}
	else if (interface == 2) {
		targetController = peltier2;
	}
	else if (interface == 3) {
		targetController = peltier3;
	}
	else { // No valid command given: set to null
		targetController = 0;
	}

	return targetController;
}
