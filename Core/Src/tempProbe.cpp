#include "tempProbe.h"
#include "kalman.h"

//TempProbe::TempProbe(uint32_t *adcValue) {
TempProbe::TempProbe() {
	_adcValue = nullptr;
	//_kf = KalmanFilter(2.0, 2.0, 0.05);
	_factor = 150.0 / 4096;
	_offset = -0.3;
}

void TempProbe::setADCRef(uint32_t *adcValue) {
	_adcValue = adcValue;

}

void TempProbe::update() {
	_temperature = _kf.updateEstimate((*_adcValue) * _factor + _offset);
	//_temperature = (1.0 - _alpha) * _temperature + _alpha * (*_adcValue) * _factor;
}

float TempProbe::getTemperature() {
	return _temperature;
}
