#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "waterlevel.h"

WaterLevel::WaterLevel() {}

WaterLevel::WaterLevel(GPIO_TypeDef *port, uint16_t pin, uint32_t *adcRef)
    : _portEnable(port), _pinEnable(pin), _adcValue(adcRef) {}

void WaterLevel::update() {

    if ((HAL_GetTick() - _timeLastMeasure) > (1 / _updateRate) * 1000) {

        HAL_GPIO_WritePin(_portEnable, _pinEnable, GPIO_PinState::GPIO_PIN_SET);

        if (_measureCount < _numMeasure) {
            _value += *_adcValue;
            ++_measureCount;
        } else {
            _value /= _numMeasure;

            if (_value > _threshold) {
                _state = WaterLevel::state::LOW;
                if (_handleLevelLow)
                    _handleLevelLow();
            } else {
                _state = WaterLevel::state::OK;
                if (_handleLevelOk)
                    _handleLevelOk();
            }

            HAL_GPIO_WritePin(_portEnable, _pinEnable,
                              GPIO_PinState::GPIO_PIN_RESET);
            _measureCount = 0;
            _timeLastMeasure = HAL_GetTick();
        }
    }
}

bool WaterLevel::isLow() {
    if (_state == WaterLevel::state::LOW)
        return true;
    else
        return false;
}
