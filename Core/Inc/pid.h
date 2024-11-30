#ifndef PID_h
#define PID_h

class PID {

  public:
    PID();
    PID(float kp, float ki, float kd);
    float update(float value);
    void setTargetValue(float value);
    float getTargetValue() { return _targetValue; };
    float getMaxValue() { return _maxValue; }
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void reset();

  private:
    float _error = 0;
    float _targetValue = 20;
    float _lastError = 0;
    float _cumError = 0;
    float _rateError = 0;
    float _lastRateError = 0;
    float _deltaT = 0;
    float _updateRate = 10; // 10Hz
    long _timeNow, _timePrevStep;
    double _controlValue = 0;
    float _minValue = 0;
    float _maxValue = 100;
    float _pValue = 0;
    float _iValue = 0;
    float _dValue = 0;
    float _kp = 5;
    float _ki = 0.5;
    float _kd = 1;
};

#endif
