#ifndef WATERLEVEL_H
#define WATERLEVEL_H

#include "stm32l4xx_hal.h"
#include <functional>

class WaterLevel {

  public:
    WaterLevel();
    WaterLevel(GPIO_TypeDef *port, uint16_t pin, uint32_t *adcRef);
    void setHandleLevelLow(std::function<void(void)> handle) {
        _handleLevelLow = handle;
    };
    void setHandleLevelOk(std::function<void(void)> handle) {
        _handleLevelOk = handle;
    };
    bool isLow();

    void update(float temp);

    enum state { LOW, OK };

  private:
    state _state = state::LOW;
    GPIO_TypeDef *_portEnable;
    uint16_t _pinEnable;

    uint32_t *_adcValue = nullptr;

    std::function<void(void)> _handleLevelLow;
    std::function<void(void)> _handleLevelOk;

    uint32_t _threshold = 2270;
    uint32_t _thresholdLowTemp = 2800;
    float _lowTemp = 50;
    uint8_t _numMeasure = 10;
    uint8_t _measureCount = 0;
    float _value = 0;
    float _updateRate = 1;
    uint32_t _timeLastMeasure;
};

#endif
