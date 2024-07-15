#ifndef INC_SWITCH_H_
#define INC_SWITCH_H_

#include "led.h"
#include "stm32l4xx_hal.h"
#include <functional>

class Switch {
  public:
    enum state { PRESSED, OFF };
    Switch();
    Switch(GPIO_TypeDef *port, uint16_t pin);
    void update();
    // void setEvent1Handle(void (*handle)(void)) { _handleEvent1 = handle; };
    void setEventHandle1(std::function<void(void)> handle) {
        _handleEvent1 = handle;
    };
    void setEventHandle2(std::function<void(void)> handle) {
        _handleEvent2 = handle;
    };
    void onRelease(std::function<void(void)> handle) {
        _handleOnRelease = handle;
    };
    // void setEvent2Handle(void (*handle)(void)) { _handleEvent2 = handle; };

  private:
    state _state = state::OFF;
    std::function<void(void)> _handleOnRelease;
    std::function<void(void)> _handleEvent1;
    std::function<void(void)> _handleEvent2;
    // void (*_handleEvent1)(void);
    // void (*_handleEvent2)(void);

    GPIO_TypeDef *_port;
    uint16_t _pin;
    uint32_t _timePressed;
    uint32_t _eventTime1 = 500;
    uint32_t _eventTime2 = 1500;
};

#endif
