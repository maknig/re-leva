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
    void setMinMaxTemp(float minTemp, float maxTemp);
    float getTargetTemp();
    void setSwitch(GPIO_TypeDef *port, uint16_t pin);
    void setPidParam(float kp, float ki, float kd) {
        _pid.setKp(kp);
        _pid.setKi(ki);
        _pid.setKd(kd);
    };
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

    float _minTemp = 20;
    float _maxTemp = 130;

    float _tempMargin = 0.8;

    PID _pid = PID(7, 0.38, 85);

    uint8_t _totalPulses = 100;
    uint8_t _skipPulses = 0;
    uint8_t _firePulses = 0;
    uint8_t _fireCtrl = 0;
    uint8_t _skipCtrl = 0;
};

#endif
