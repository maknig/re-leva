#ifndef PUMP_H
#define PUMP_H

#include "pid.h"
#include "stm32l4xx_hal.h"

class Pump {

  public:
    Pump();
    Pump(GPIO_TypeDef *port, uint16_t pin);
    void firePulse();
    void update();
    void togglePump();
    void pump(uint16_t time);

    enum state { OFF, PUMP, PUMP_FOR };

  private:
    state _state = state::OFF;
    GPIO_TypeDef *_port;
    uint16_t _pin;

    PID _pid;
    uint16_t _timePumpStart = 0;
    uint16_t _timePumpFor = 0;
    uint8_t _totalPulses = 8;
    uint8_t _skipPulses = 2;
    uint8_t _firePulses = 6;
    uint8_t _fireCtrl = 0;
    uint8_t _skipCtrl = 0;
};

#endif
