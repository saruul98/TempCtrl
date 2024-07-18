#include "PIDAlgorithm.h"
#include "Arduino.h"

namespace NewTempCtrl {


	PIDAlgorithm::PIDAlgorithm(double K, double Ti, double Td, double initialOutput, double target, double N, bool disableProportional, int timingCycles, bool debug) :
		_K(K), _Ti(Ti), _Td(Td), _N(N), _target(target), _disableProportional(disableProportional), _timingCycles(timingCycles), _debug(debug)
	{
		// Reset the loop counter that will time the cycles to determine the precalculated constants
		_timingCount = 0;
		_timerStart = micros();

		// Calculate the constants assuming that the cycle time is 50ms: 
		//	this will be wrong, but get corrected on the first recalulation. 
		recalculateConsts(50);

		// _Output should start as whatever it currently is
		_initOut = initialOutput;
		setOutput(_initOut);


		// Mark the PID as not having been run yet
		_firstRun = true;
	}


	PIDAlgorithm::~PIDAlgorithm()
	{
	}

	// Set the output level manually
	void PIDAlgorithm::setOutput(double output) {

		_output = output;

	}

	// Change the PID's setpoint
	void PIDAlgorithm::setSetpoint(double setpoint) {

		// Spoof a steady state at previous value
		_old2 = _old;
		_deltaDold = 0;

		// Check the bounds are OK
		// if (setpoint > 1)
		// 	setpoint = 1;
		// else if (setpoint < -1)
		// 	setpoint = -1;

		// Update the setpoint
		_target = setpoint;
	}


	void PIDAlgorithm::setGain(double gain) {

		_K = gain;
		_Kp = _K;

	}

	void PIDAlgorithm::resetAlgo(){

		_timingCount = 0;
		_timerStart = micros();

		recalculateConsts(50);
		
		setOutput(_initOut);

		_firstRun = true;

	}

	// Do the incremental PID calculation
	// See p242 of http://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf
	double PIDAlgorithm::output(double input)
	{
		// *** Loop timing:  *** 

		// The following lines provide a guess for the cycle time by timing a number of loop cycles
		// ( _timingCycles = 100 by default) and then recalculating the constants accordingly

		// _timingCount starts at 0 and is incremented up to timingCycles
		// When _timingCount == _timingCycles, _timingCycles cycles have been executed so 
		// the calculation is performed & _timingCount is reset
		if (_timingCount >= _timingCycles) {

			// Record the cycle time
			_timerEnd = micros();

			// Recalculate the constants. 
			// micros() will roll over to zero every 70 mins or so, so catch this
			if (_timerEnd > _timerStart)
			{
				// Calculate single cycle time in microseconds
				long singleCycle = (_timerEnd - _timerStart) / _timingCycles;

				recalculateConsts(singleCycle);
			}

			// Reset the counter
			_timingCount = 0;
			_timerStart = micros();
		}
		else {  // Unless we were on _timingCycles, increment the counter
			_timingCount++;
		}

		// ----------------------------------
		// ALGO 0: INCREMENTAL PID ALGORITHM:
		// ----------------------------------
		// if (_firstRun) {
		// 	_old = input ;
		// 	_old2 = input ;
		// 	_deltaDold = 0.0 ;
		// 	_firstRun = false ;
		// }
		// deltaP = _Kp *(_old - input) ;
		// deltaI = _Ki * (_target - input) ;
		// deltaD = _Kd1 * _deltaDold - _Kd2 * (input - 2 * _old + _old2) ;
		// if (_disableProportional)
		// 	_output += deltaI + deltaD;
		// else
		// 	_output += deltaP + deltaI + deltaD;
		// _old2 = _old;
		// _old = input;
		// _deltaDold = deltaD;
		// // Windup prevention:
		// if (_output > 1)
		// 	_output = 1;
		// else if (_output < -1)
		// 	_output = -1;
		// ---------------------------------------------



		// -------------------------------------------
		// ALGO 1: MODIFIED INCREMENTAL PID ALGORITHM:
		// -------------------------------------------
		// if (_firstRun) {
		// 	_old = input ;
		// 	_old2 = input ;
		// 	_deltaDold = 0.0 ;
		// 	_output = _Kp * (_target - input) ;
		// 	_firstRun = false ;
		// }
		// error = _target - input ; 
		// error_old = _target - _old ;

		// deltaP = _Kp * (error - error_old) ;
		// deltaI = _Ki * error ;
		// deltaD = _Kd1 * _deltaDold - _Kd2 * (input - 2 * _old + _old2) ;		
		// deltaU = deltaP + deltaI + deltaD;

		// _output += deltaU ;

		// _old2 = _old;
		// _old = input;
		// _deltaDold = deltaD;

		// // Windup prevention:
		// if (_output > 1)
		// 	_output = 1;
		// else if (_output < -1)
		// 	_output = -1;
		// ---------------------------------------------



		// ---------------------------------------
		// ALGO 2: FAST INCREMENTAL PID ALGORITHM:
		// ---------------------------------------
		if (_firstRun) {
			_old = input ;
			_old2 = input ;
			_deltaDold = 0.0 ;
			P = _Kp * (_target - input) ; I = 0 ; D = 0 ;
			_firstRun = false ;
		}
		error = _target - input ;
		error_old = _target - _old ;

		deltaP = _Kp * (error - error_old) ;
		deltaI = _Ki * error ;
		deltaD = _Kd1 * _deltaDold - _Kd2 * (input - 2 * _old + _old2) ;

		_old2 = _old;
		_old = input;

		P += deltaP ;
		I += deltaI ;
		D += deltaD ;
		_output = P + I + D ;


		// Windup prevention:
		if (_output > 1 || _output < -1) {
			P = _Kp * (_target - input) ;
			I += - deltaI ;
			_output = P + I + D ;
		}
		// ---------------------------------------------




		// --------------------------------------------------------
		// ALGO 3: ABSOLUTE PID ALGORITHM WITH ANTI-WINDUP I-RESET:
		// --------------------------------------------------------
		// if (_firstRun) {
		// 	_old = input ;
		// 	_I_old = 0.0 ;
		// 	_firstRun = false ;
		// }

		// double b = 1.0 ; // Setpoint weighting

		// P = _Kp * (b * _target - input) ;
		// I = _I_old + _Ki * (_target - _old) ;

		// _output = P + I ;

		// _old = input; 
		// _I_old = I ;

		// // Windup prevention:
		// if (_output > 2 || _output < -2) {
		// 	_I_old = 0 ; // Reset integral part
		// }
		// ---------------------------------------------



		if (_debug) {
			Serial.print("Error: ");
			Serial.print((_target - input),8);
			Serial.print(", integral bit: ");
			Serial.print(_Ki * (_target - input),8);
			Serial.print(", deltaD: ");
			Serial.print(deltaD,8);
			Serial.print(", ctrl sig: ");
			Serial.print(_output,8);
			Serial.print(", input: ");
			Serial.println(input,8);
		}

		return _output;
	}

	int PIDAlgorithm::recalculateConsts(unsigned long microseconds)
	{
		double cycletime = (float)microseconds / 1000000.0;
		// All calculations in seconds

		_Kp = _K;
		_Ki = _K * cycletime / _Ti;
		_Kd1 = _Td / (_Td + _N * cycletime);
		_Kd2 = _Kd1 * _K * _N;

		//                Serial.print(_Kp);
		//                Serial.print(", ");
		//                Serial.print(_Ki*1000000);
		//                Serial.print(", ");
		//                Serial.print(_Kd1*1000000);
		//                Serial.print(", ");
		//                Serial.print(_Kd2*1000000);
		//                Serial.println(".");

	}

}
