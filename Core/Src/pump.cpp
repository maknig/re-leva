#include "pump.h"
#include "stm32l4xx_hal.h"

Pump::Pump(GPIO_TypeDef *port, uint16_t pin) : _port(port), _pin(pin) {}

void Pump::update() {
    if (_state == Pump::state::PUMP_FOR) {

        if ((HAL_GetTick() - _timePumpStart) > _timePumpFor)
            _state = Pump::state::OFF;
    }
}

void Pump::togglePump() {
    if (_state == Pump::state::OFF)
        _state = Pump::state::PUMP;
    else
        _state = Pump::state::OFF;
}

void Pump::pump(uint16_t time) {
    if (_state == Pump::state::OFF) {
        _timePumpStart = HAL_GetTick();
        _timePumpFor = time * 1000;
        _state = Pump::state::PUMP_FOR;
    }
}

void Pump::firePulse() {
    if (_state == Pump::state::PUMP || _state == Pump::state::PUMP_FOR) {
        if (_fireCtrl < _firePulses) {
            HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
            _fireCtrl++;
        } else if (_skipCtrl < _skipPulses) {
            HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
            _skipCtrl++;
        } else {
            _skipCtrl = 0;
            _fireCtrl = 0;
        }

    } else {

        HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
    }
}
