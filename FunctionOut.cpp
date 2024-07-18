#include "FunctionOut.h"
#include "Arduino.h"

namespace NewTempCtrl {


	FunctionOut::FunctionOut(int type, double frequency, double amplitude) : _type(type), _freq(frequency), _amp(amplitude)
	{
		_timingCount = 0;
		_timerStart = millis();
		_firstRun = true;
	}


	FunctionOut::~FunctionOut() {}

	void FunctionOut::setOutput(double output) {}

	void FunctionOut::setSetpoint(double setpoint) {}

	void FunctionOut::setGain(double gain) {}

	void FunctionOut::resetAlgo(){

		_timingCount = 0;
		_timerStart = millis();
		_firstRun = true;

	}

	double FunctionOut::output(double input)
	{
	
	_timerEnd = millis();
	double Time = (_timerEnd - _timerStart) / 1000.0 ;

	if (_type == 1){
		_output = _amp * sin( 2 * pi * _freq * Time ) ;
	}
	else
		_output = -1.0 ;

	
	// _output = Time ;

	return _output;
	}


}
