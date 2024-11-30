#include "pid.h"
#include "stm32l4xx_hal.h"

PID::PID() {}

PID::PID(float kp, float ki, float kd) : _kp(kp), _ki(ki), _kd(kd) {}

void PID::setTargetValue(float value) { _targetValue = value; }
void PID::setKp(float kp) { _kp = kp; }

void PID::setKi(float ki) { _ki = ki; }

void PID::setKd(float kd) { _kd = kd; }

void PID::reset() {
    _lastError = 0;
    _cumError = 0;
    _rateError = 0;
}

float PID::update(float value) {

    _timeNow = HAL_GetTick();
    _deltaT = (_timeNow - _timePrevStep) * 0.001;

    if (_deltaT > (1 / _updateRate)) {
        // calculate the error

        _error = _targetValue - value;
        _cumError += _error * _deltaT;
        _rateError =
            _rateError * 0.85 + 0.15 * ((_error - _lastError) / _deltaT);

        if (_cumError > 30)
            _cumError = 30;
        if (_cumError < -10)
            _cumError = -10;

        // Calculate the P value
        _pValue = _kp * _error;

        // Calculate the I value
        _iValue = _ki * _cumError;

        // Calculate the D value
        _dValue = _kd * _rateError;

        // if (_iValue > 100)
        //     _iValue = 100;
        // if (_iValue < 0)
        //     _iValue = 0;
        if (_pValue < 0)
            _pValue = 0;

        // Final total PID value is the sum of P + I + D
        _controlValue = _pValue + _iValue + _dValue;

        // We define PWM range between 0 and 100
        if (_controlValue < 0 || _error < 0) {
            _controlValue = _minValue;
        }
        if (_controlValue > _maxValue) {
            _controlValue = _maxValue;
        }

        _lastError = _error;
        _timePrevStep = _timeNow;
    }

    return _controlValue;
}
