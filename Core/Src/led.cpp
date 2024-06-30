/*
 * led.cpp
 *
 *  Created on: Jun 23, 2024
 *      Author: matthias
 */
#include "led.h"
#include "stm32l4xx_hal.h"

LED::LED(GPIO_TypeDef *port, uint16_t pin) :
		_port(port), _pin(pin) {
}

LED::LED() {
}

void LED::setState(LED::state state) {
	_state = state;
}

void LED::doBlink() {
	_state = LED::state::BLINKING;
}

void LED::setBlinkTime(uint32_t time) {
	_blinkTime = time;
}
void LED::toggle(){

	if (_state == LED::state::OFF)
		_state = LED::state::ON;
	else
		_state = LED::state::OFF;
}

void LED::update() {
	switch (_state) {

	case LED::state::OFF:
		HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
		break;

	case LED::state::ON:
		HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
		break;

	case LED::state::BLINKING:
		uint32_t deltaT = (HAL_GetTick() - _lastBlink);
		if (deltaT > _blinkTime) {
			HAL_GPIO_TogglePin(_port, _pin);
			_lastBlink = HAL_GetTick();
		}
		break;

	}

}

