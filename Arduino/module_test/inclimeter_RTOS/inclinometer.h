#pragma once

#include <Arduino.h>
#include <Adafruit_MPU6050.h>

class Inclinometer
{
public:
  Inclinometer() = default;
  ~Inclinometer() = default;

  void begin(const double freq);
  void updateTheta();

  inline const double& getFilteredTheta() const {
    Serial.println(theta_data_[0][0]);
    return theta_data_[0][0];
  }
  inline const double& getThetaFromAcc() const {
    sensors_event_t acc;
    acc_->getEvent(&acc);
    return -atan2(acc.acceleration.x - offset.acceleration.x, acc.acceleration.z - offset.acceleration.z) * 180./3.14;
  }
  inline const double& getThetaDotFromGyro() const {
    sensors_event_t gyro;
    gyro_->getEvent(&gyro);
    return (gyro.gyro.y - offset.gyro.y) * 180./3.14;
  }

private:
  inline void setupIMU();
  inline void calculateOffset();
  inline void initThetaVariance();
  inline void initThetaDotVariance();

  //=========================================================
  //Matric calculate function
  //=========================================================
  //addition
  inline void mat_add(const double *m1, const double *m2, double *sol, const int& row, const int& column);
  //subtraction
  inline void mat_sub(const double *m1, const double *m2, double *sol, const int& row, const int& column);
  //multiplication
  inline void mat_mul(const double *m1, const double *m2, double *sol, const int row1, const int column1, const int row2, const int column2);
  //transposition
  inline void mat_tran(const double *m1, double *sol, const int& row_original, const int& column_original);
  //scalar maltiplication
  inline void mat_mul_const(const double *m1, const double& c, double *sol, const int& row, const int& column);
  //inversion (by Gaussian elimination)
  inline void mat_inv(const double *m, double *sol, const int& column, const int& row);

private:
  Adafruit_MPU6050 mpu_;
  Adafruit_Sensor *acc_, *gyro_;
  sensors_event_t offset;

  //=========================================================
  //Accelerometer and gyro statistical data
  //=========================================================
  const int sample_num_ = 100;
  const int meas_interval_ = 10; // measure interval
  double theta_mean_;
  double theta_variance_;
  double theta_dot_mean_;
  double theta_dot_variance_;

  //=========================================================
  //Kalman filter (for angle estimation) variables
  //=========================================================
  int theta_update_freq_; //Hz
  double theta_update_interval_ = 1.0/(double)theta_update_freq_; //2.5msec
  //State vector
  //[[theta(degree)], [offset of theta_dot(degree/sec)]]
  double theta_data_predict_[2][1];
  double theta_data_[2][1];
  //Covariance matrix
  double P_theta_predict_[2][2];
  double P_theta_[2][2];
  //"A" of the state equation
  double A_theta_[2][2] = {{1, -theta_update_interval_}, {0, 1}};
  //"B" of the state equation
  double B_theta_[2][1] = {{theta_update_interval_}, {0}};
  //"C" of the state equation
  double C_theta_[1][2] = {{1, 0}};
};
