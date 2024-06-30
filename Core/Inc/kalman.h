#ifndef KalmanFilter_h
#define KalmanFilter_h

class KalmanFilter{

public:
  KalmanFilter(float mea_e, float est_e, float q);
  KalmanFilter(){};
  float updateEstimate(float mea);
  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
  float getKalmanGain();
  float getEstimateError();

private:
  float _err_measure =2.0;
  float _err_estimate = 2.0;
  float _q = 0.03;
  float _current_estimate = 0;
  float _last_estimate = 0;
  float _kalman_gain = 0;

};

#endif
