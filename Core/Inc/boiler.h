#ifndef boiler_h
#define boiler_h

#include "pid.h"
#include "led.h"
#include "switch.h"
#include "stm32l4xx_hal.h"
#include "tempprobe.h"

class Boiler {
  public:
    Boiler();
    // Boiler(TempProbe &tempProbe);
    void firePulse();
    void update();
    void updateSwitch();
    void setHeater(GPIO_TypeDef *port, uint16_t pin);
    float getTemperature();
    void setTempProbe(TempProbe &tempProbe);
    void setTargetTemp(float temp);
    void setSwitch(GPIO_TypeDef *port, uint16_t pin);
    void toggleState();
    void setLed(LED *led){_led = led;};
    bool isWaterLevelLow();

    enum state { OFF, HEATING };
    enum switchState { PRESSED, RELEASED };

  private:
    state _state;
    switchState _switchState;

    uint32_t *_adcValueWaterLevel = nullptr;

    uint16_t _waterLevelEnablePin;
    LED  *_led;
    uint16_t _switchPin;
    GPIO_TypeDef *_switchPort;

    uint32_t _timePressed = 0;
        uint32_t _eventTime1 = 500;
        uint32_t _eventTime2 = 1500;

    uint16_t _heaterPin;
    GPIO_TypeDef *_heaterPort;
    GPIO_TypeDef *_waterLevelEnablePort;
    GPIO_TypeDef *_ledPort;
    TempProbe _tempProbe;
    float _tempMargin = 2.0;
    PID _pid;
    uint8_t _totalPulses = 100;
    uint8_t _skipPulses = 0;
    uint8_t _firePulses = 0;
    uint8_t _fireCtrl = 0;
    uint8_t _skipCtrl = 0;
};

#endif
