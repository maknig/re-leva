#include "pid.h"
#include "stm32l4xx_hal.h"

PID::PID() {}

PID::PID(float kp, float ki, float kd) : _kp(kp), _ki(ki), _kd(kd) {}

void PID::setTargetValue(float value) { _targetValue = value; }
void PID::setKp(float kp) { _kp = kp; }

void PID::setKi(float ki) { _ki = ki; }

void PID::setKd(float kd) { _kd = kd; }

void PID::reset() {
    _previous_error = 0;
    _iValue = 0;
}

float PID::update(float value) {

    _timeNow = HAL_GetTick();
    _deltaT = (_timeNow - _timePrevStep) * 0.001;

    // calculate the error
    _error = _targetValue - value;

    // Calculate the P value
    _pValue = _kp * _error;

    // Calculate the I value
    _iValue = _iValue + (_ki * _error) * _deltaT;

    if (_iValue > 100)
        _iValue = 100;
    if (_iValue < 0)
        _iValue = 0;

    // Now we can calculate the D value
    _dValue = _kd * ((_error - _previous_error) / _deltaT);

    // Final total PID value is the sum of P + I + D
    _controlValue = _pValue + _iValue + _dValue;

    // We define PWM range between 0 and 100
    if (_controlValue < 0 || _error < 0) {
        _controlValue = _minValue;
    }
    if (_controlValue > _maxValue) {
        _controlValue = _maxValue;
    }

    _previous_error =
        _error; // Remember to store the previous error for next loop.
    _timePrevStep = _timeNow;
    return _controlValue;
}
