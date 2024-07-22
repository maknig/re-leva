#ifndef boiler_h
#define boiler_h

#include "led.h"
#include "pid.h"
#include "pump.h"
#include "stm32l4xx_hal.h"
#include "tempProbe.h"
#include "waterlevel.h"

class Boiler {
  public:
    Boiler(LED &led, TempProbe &tempProbe, WaterLevel &level, Pump &pump);
    // Boiler(TempProbe &tempProbe);
    void firePulse();
    void update();
    void updateSwitch();
    void setHeater(GPIO_TypeDef *port, uint16_t pin);
    float getTemperature();
    void setTempProbe(TempProbe &tempProbe);
    void setWaterLevel(WaterLevel &waterLevel);
    void setTargetTemp(float temp);
    void setSwitch(GPIO_TypeDef *port, uint16_t pin);
    void toggleState();
    void setLed(LED &led) { _led = led; };
    bool isWaterLevelLow();

    enum state { OFF, HEATING };

  private:
    state _state = state::OFF;


    LED &_led;

    uint16_t _heaterPin;
    GPIO_TypeDef *_heaterPort;
    TempProbe &_tempProbe;
    WaterLevel &_waterLevel;
    Pump &_pump;

    float _tempMargin = 1.0;
    PID _pid = PID(22, 0, 4);
    uint8_t _totalPulses = 100;
    uint8_t _skipPulses = 0;
    uint8_t _firePulses = 0;
    uint8_t _fireCtrl = 0;
    uint8_t _skipCtrl = 0;
};

#endif
