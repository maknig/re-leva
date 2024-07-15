#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "switch.h"

Switch::Switch() {}

Switch::Switch(GPIO_TypeDef *port, uint16_t pin) : _port(port), _pin(pin) {}

void Switch::update() {
    if (HAL_GPIO_ReadPin(_port, _pin) == 0) {

        if (_state == Switch::state::OFF) {
            _state = Switch::state::PRESSED;
            _timePressed = HAL_GetTick();

        } else {
        }
    } else {
        if (_state == Switch::state::PRESSED) {
            uint32_t deltaT = HAL_GetTick() - _timePressed;
            if (deltaT > _eventTime1 && deltaT < _eventTime2) {
                _handleEvent1();
                //_led->toggle();
                _state = Switch::state::OFF;
            } else if ((HAL_GetTick() - _timePressed) > _eventTime2) {
                _handleEvent2();
                //_led->doBlink();
                _state = Switch::state::OFF;
            }
        }
    }
}
