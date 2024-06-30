#include "boiler.h"
#include "kalman.h"
#include "pid.h"
#include "tempProbe.h"

#include "stm32l4xx_hal.h"

Boiler::Boiler() {
	_firePulses = 3;
	_skipPulses = 7;
	_state = Boiler::state::OFF;
	_tempProbe = TempProbe();
	//_pid = PID_Heater(100, 3, 100.0);
}
// Boiler::Boiler(TempProbe &tempProbe){
//	_firePulses = 3;
//	_skipPulses = 7;
//	_state = Boiler::state::HEATING;
//	_tempProbe = tempProbe;
// }
void Boiler::updateSwitch() {
	if (HAL_GPIO_ReadPin(_switchPort, _switchPin) == 0) {

		if (_switchState == Boiler::switchState::RELEASED) {
			_switchState = Boiler::switchState::PRESSED;
			_timePressed = HAL_GetTick();
		} else {
		}
	} else {
		if (_switchState == Boiler::switchState::PRESSED) {
			uint32_t deltaT = HAL_GetTick() - _timePressed;
			if (deltaT > _eventTime1 && deltaT < _eventTime2) {
				//_handleEvent1();
				if (_state == Boiler::state::OFF)
					_state = Boiler::state::HEATING;
				else
					_state = Boiler::state::OFF;
				_switchState = Boiler::switchState::RELEASED;

			} else if ((HAL_GetTick() - _timePressed) > _eventTime2) {
				//_handleEvent2();
				//_led->doBlink();
				_state = Boiler::state::OFF;

				_switchState = Boiler::switchState::RELEASED;
			}

		}
	}
}

void Boiler::update() {
	_tempProbe.update();
	//updateSwitch();

	float temp = _tempProbe.getTemperature();
	_firePulses = _pid.update(temp);
	_skipPulses = _pid.getMaxValue() - _firePulses;

	if (_pid.getTargetValue() - temp > _tempMargin)
		_led->doBlink();
	else
		_led->setState(LED::state::ON);

	if (_state == Boiler::state::OFF)
		_led->setState(LED::state::OFF);

	_led->update();
}

bool Boiler::isWaterLevelLow() {
	return false;
}

void Boiler::setTargetTemp(float temp) {
	_pid.setTargetValue(temp);
}

void Boiler::firePulse() {
	if (_state == Boiler::state::HEATING) {
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
	}
}

float Boiler::getTemperature() {
	return _tempProbe.getTemperature();
}

void Boiler::setTempProbe(TempProbe &tempProbe) {
	_tempProbe = tempProbe;
}

void Boiler::setHeater(GPIO_TypeDef *port, uint16_t pin) {
	_heaterPort = port;
	_heaterPin = pin;
}

void Boiler::setSwitch(GPIO_TypeDef *port, uint16_t pin) {
	_switchPort = port;
	_switchPin = pin;
}

void Boiler::toggleState(){
	if (_state == Boiler::state::HEATING)
		_state = Boiler::state::OFF;
	else
		_state = Boiler::state::HEATING;
}

