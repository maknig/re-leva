#include "boiler.h"
#include "kalman.h"
#include "pid.h"
#include "tempProbe.h"
#include "waterlevel.h"

#include "stm32l4xx_hal.h"

Boiler::Boiler(LED &led, TempProbe &tempProbe, WaterLevel &level, Pump &pump)
    : _led(led), _tempProbe(tempProbe), _waterLevel(level), _pump(pump) {}
// Boiler::Boiler(TempProbe &tempProbe){
//	_firePulses = 3;
//	_skipPulses = 7;
//	_state = Boiler::state::HEATING;
//	_tempProbe = tempProbe;
// }

void Boiler::update() {

    _tempProbe.update();
    _led.update();

    if (_state == Boiler::state::HEATING) {
        _waterLevel.update();
        _pump.update();
        // updateSwitch();

        float temp = _tempProbe.getTemperature();
        _firePulses = _pid.update(temp);
        _skipPulses = _pid.getMaxValue() - _firePulses;

        if (_pid.getTargetValue() - temp > _tempMargin)
            _led.doBlink();
        else
            _led.setState(LED::state::ON);

    } else {
        _led.setState(LED::state::OFF);
    }

}

bool Boiler::isWaterLevelLow() { return _waterLevel.isLow(); }

void Boiler::setTargetTemp(float temp) { _pid.setTargetValue(temp); }

void Boiler::firePulse() {
    if (_state == Boiler::state::HEATING && !_waterLevel.isLow()) {

        if (_fireCtrl < _firePulses) {
            HAL_GPIO_WritePin(_heaterPort, _heaterPin, GPIO_PIN_SET);
            _fireCtrl++;
        } else if (_skipCtrl < _skipPulses) {
            HAL_GPIO_WritePin(_heaterPort, _heaterPin, GPIO_PIN_RESET);
            _skipCtrl++;
        } else {
            _skipCtrl = 0;
            _fireCtrl = 0;
        }
    } else {

        HAL_GPIO_WritePin(_heaterPort, _heaterPin, GPIO_PIN_RESET);
    }
}

float Boiler::getTemperature() { return _tempProbe.getTemperature(); }

void Boiler::setTempProbe(TempProbe &tempProbe) { _tempProbe = tempProbe; }
void Boiler::setWaterLevel(WaterLevel &waterLevel) { _waterLevel = waterLevel; }

void Boiler::setHeater(GPIO_TypeDef *port, uint16_t pin) {
    _heaterPort = port;
    _heaterPin = pin;
}

void Boiler::toggleState() {
    if (_state == Boiler::state::HEATING) {
        _state = Boiler::state::OFF;
        _pump.stop();
    } else {
        _state = Boiler::state::HEATING;
    }
}
