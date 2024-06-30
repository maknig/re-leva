#ifndef TEMPPROBE_H
#define TEMPPROBE_H

#include "kalman.h"
#include "main.h"

class TempProbe {
  public:

    TempProbe();
    //TempProbe(uint32_t *adcValue);
    void setADCRef(uint32_t *adcValue);
    void update();
    float getTemperature();

  private:
    uint32_t *_adcValue = nullptr;
    float _factor = 1.0;
    float _offset = 0.5;
    float _temperature = 0;
    KalmanFilter _kf;
};

#endif
