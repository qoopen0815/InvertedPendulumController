#include "inclinometer.h"

void Inclinometer::begin(const double freq)
{
  theta_update_freq_ = freq;
  theta_update_interval_ = 1.0/theta_update_freq_;

  A_theta_[0][1] = -theta_update_interval_;
  B_theta_[0][0] = theta_update_interval_;

  // Serial.println("Inclinometer Info");
  // Serial.print("- Frequency[Hz] : ");
  // Serial.println(theta_update_freq_);
  // Serial.print("- Interval[ms] : ");
  // Serial.println(theta_update_interval_*1000);

  setupIMU();
  calculateOffset();

  //Accelerometer & Gyro calibration (100 step)
  // Serial.println("Inclinometer : calculating Acc & Gyro variance ...");
  initThetaVariance();
  initThetaDotVariance();
  // Serial.println("Inclinometer : Done !");

  //カルマンフィルタの初期設定 初期姿勢は0°(直立)を想定
  theta_data_predict_[0][0] = 0;
  theta_data_predict_[1][0] = theta_dot_mean_;
  
  P_theta_predict_[0][0] = 1;
  P_theta_predict_[0][1] = 0;
  P_theta_predict_[1][0] = 0;
  P_theta_predict_[1][1] = theta_dot_variance_;
}

void Inclinometer::updateTheta(void)
{
  double y = getThetaFromAcc(); // [degree]
  double theta_dot_gyro = getThetaDotFromGyro(); // [degree/sec]

  //カルマン・ゲイン算出: G = P'C^T(W+CP'C^T)^-1
  double P_CT[2][1] = {};
  double tran_C_theta[2][1] = {};
  mat_tran(C_theta_[0], tran_C_theta[0], 1, 2);//C^T
  mat_mul(P_theta_predict_[0], tran_C_theta[0], P_CT[0], 2, 2, 2, 1);//P'C^T
  double G_temp1[1][1] = {};
  mat_mul(C_theta_[0], P_CT[0], G_temp1[0], 1,2, 2,1);//CP'C^T
  double G_temp2 = 1.0f / (G_temp1[0][0] + theta_variance_);//(W+CP'C^T)^-1
  double G1[2][1] = {};
  mat_mul_const(P_CT[0], G_temp2, G1[0], 2, 1);//P'C^T(W+CP'C^T)^-1

  //傾斜角推定値算出: theta = theta'+G(y-Ctheta')
  double C_theta_theta[1][1] = {};
  mat_mul(C_theta_[0], theta_data_predict_[0], C_theta_theta[0], 1, 2, 2, 1);//Ctheta'
  double delta_y = y - C_theta_theta[0][0];//y-Ctheta'
  double delta_theta[2][1] = {};
  mat_mul_const(G1[0], delta_y, delta_theta[0], 2, 1);
  mat_add(theta_data_predict_[0], delta_theta[0], theta_data_[0], 2, 1);
          
  //共分散行列算出: P=(I-GC)P'
  double GC[2][2] = {};
  double I2[2][2] = {{1,0},{0,1}};
  mat_mul(G1[0], C_theta_[0], GC[0], 2, 1, 1, 2);//GC
  double I2_GC[2][2] = {};
  mat_sub(I2[0], GC[0], I2_GC[0], 2, 2);//I-GC
  mat_mul(I2_GC[0], P_theta_predict_[0], P_theta_[0], 2, 2, 2, 2);//(I-GC)P'

  //次時刻の傾斜角の予測値算出: theta'=Atheta+Bu
  double A_theta_theta[2][1] = {};
  double B_theta_dot[2][1] = {};
  mat_mul(A_theta_[0], theta_data_[0], A_theta_theta[0], 2, 2, 2, 1);//Atheta
  mat_mul_const(B_theta_[0], theta_dot_gyro, B_theta_dot[0], 2, 1);//Bu
  mat_add(A_theta_theta[0], B_theta_dot[0], theta_data_predict_[0], 2, 1);//Atheta+Bu 
  
  //次時刻の共分散行列算出: P'=APA^T + BUB^T
  double AP[2][2] = {};   
  double APAT[2][2] = {};
  double tran_A_theta[2][2] = {};
  mat_tran(A_theta_[0], tran_A_theta[0], 2, 2);//A^T 
  mat_mul(A_theta_[0], P_theta_[0], AP[0], 2, 2, 2, 2);//AP
  mat_mul(AP[0], tran_A_theta[0], APAT[0], 2, 2, 2, 2);//APA^T
  double BBT[2][2];
  double tran_B_theta[1][2] = {};
  mat_tran(B_theta_[0], tran_B_theta[0], 2, 1);//B^T
  mat_mul(B_theta_[0], tran_B_theta[0], BBT[0], 2, 1, 1, 2);//BB^T
  double BUBT[2][2] = {};
  mat_mul_const(BBT[0], theta_dot_variance_, BUBT[0], 2, 2);//BUB^T
  mat_add(APAT[0], BUBT[0], P_theta_predict_[0], 2, 2);//APA^T+BUB^T
}

void Inclinometer::setupIMU()
{
  // Try to IMU initialize!
  if ( !mpu_.begin() ) while(1) delay(10);

  // Accelerometer range set to: +-8G
  mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
  // Gyro range set to: +-250 deg/s
  mpu_.setGyroRange(MPU6050_RANGE_250_DEG);
  // Filter bandwidth set to: 21 Hz
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  acc_ = mpu_.getAccelerometerSensor();
  gyro_ = mpu_.getGyroSensor();
}

void Inclinometer::calculateOffset()
{
  delay(1000);
  offset.acceleration.x = 0.0;
  offset.acceleration.y = 0.0;
  offset.acceleration.z = 0.0;
  offset.gyro.x = 0.0;
  offset.gyro.y = 0.0;
  offset.gyro.z = 0.0;

  for(int i=0; i<10; i++) {
    sensors_event_t acc, gyro;
    acc_->getEvent(&acc);
    gyro_->getEvent(&gyro);
    delay(meas_interval_);
    offset.acceleration.x += acc.acceleration.x;
    offset.acceleration.y += acc.acceleration.y;
    offset.acceleration.z += acc.acceleration.z;
    offset.gyro.x += gyro.gyro.x;
    offset.gyro.z += gyro.gyro.y;
    offset.gyro.z += gyro.gyro.z;
  }

  offset.acceleration.x /= 10.;
  offset.acceleration.y /= 10.;
  offset.acceleration.z /= 10.;
  offset.acceleration.z -= 9.8;
  offset.gyro.x /= 10.;
  offset.gyro.y /= 10.;
  offset.gyro.z /= 10.;
}

inline void Inclinometer::initThetaVariance()
{
  double theta_array[sample_num_];
  for(int i=0; i<sample_num_; i++) {
    theta_array[i] = getThetaFromAcc();
    delay(meas_interval_);
  }

  //平均値
  theta_mean_ = 0;
  for(int i=0; i<sample_num_; i++) {
    theta_mean_ += theta_array[i];
  }
  theta_mean_ /= sample_num_;
    
  //分散
  double temp;
  theta_variance_ = 0;
  for(int i=0; i<sample_num_; i++) {
    temp = theta_array[i] - theta_mean_;
    theta_variance_ += temp*temp;
  }
  theta_variance_ /= sample_num_;
}

inline void Inclinometer::initThetaDotVariance()
{
  double theta_dot_array[sample_num_];
  for(int i=0;i<sample_num_;i++) {
    theta_dot_array[i] =  getThetaDotFromGyro();
    delay(meas_interval_);
  }

  //平均値
  theta_dot_mean_ = 0;
  for(int i=0;i<sample_num_;i++) {
    theta_dot_mean_ += theta_dot_array[i];    
  }
  theta_dot_mean_ /= sample_num_;
 
  //分散
  double temp;
  theta_dot_variance_ = 0;
  for(int i=0; i<sample_num_; i++) {
    temp = theta_dot_array[i] - theta_dot_mean_;
    theta_dot_variance_ += temp*temp;
  }
  theta_dot_variance_ /= sample_num_;
}

void Inclinometer::mat_add(double *m1, double *m2, double *sol, int row, int column)
{
  for(int i=0; i<row; i++){
    for(int j=0; j<column; j++){
      sol[i*column + j] = m1[i*column + j] + m2[i*column + j];
    }    
  }
  return;
}

inline void Inclinometer::mat_sub(double *m1, double *m2, double *sol, int row, int column)
{
  for(int i=0; i<row; i++){
    for(int j=0; j<column; j++){
      sol[i*column + j] = m1[i*column + j] - m2[i*column + j];
    }    
  }
  return;
}

inline void Inclinometer::mat_mul(double *m1, double *m2, double *sol, int row1, int column1, int row2, int column2)
{
  for(int i=0; i<row1; i++){
    for(int j=0; j<column2; j++){
      sol[i*column2 + j] = 0;
      for(int k=0; k<column1; k++){
        sol[i*column2 + j] += m1[i*column1 + k]*m2[k*column2 + j];
      }
    }
  }
  return;
}

inline void Inclinometer::mat_tran(double *m1, double *sol, int row_original, int column_original)
{
  for(int i=0; i<row_original; i++){
    for(int j=0; j<column_original; j++){
      sol[j*row_original + i] = m1[i*column_original + j];
    }
  }
  return;
}

inline void Inclinometer::mat_mul_const(double *m1,double c, double *sol, int row, int column)
{
  for(int i=0; i<row; i++){
    for(int j=0; j<column; j++){
      sol[i*column + j] = c * m1[i*column + j];
    }
  }
  return;
}

inline void Inclinometer::mat_inv(double *m, double *sol, int column, int row)
{
  //allocate memory for a temporary matrix
  double* temp = (double *)malloc( column*2*row*sizeof(double) );

  //make the augmented matrix
  for(int i=0; i<column; i++){
    //copy original matrix
    for(int j=0; j<row; j++){
      temp[i*(2*row) + j] = m[i*row + j];  
    }
    
    //make identity matrix
    for(int j=row; j<row*2; j++){
      if(j-row == i){
        temp[i*(2*row) + j] = 1;
      }else{
        temp[i*(2*row) + j] = 0;    
      }
    }
  }

  //Sweep (down)
  for(int i=0; i<column; i++){
    //pivot selection
    double pivot = temp[i*(2*row) + i];
    int pivot_index = i;
    double pivot_temp;
    for(int j=i; j<column;j++){
      if( temp[j*(2*row)+i] > pivot ){
        pivot = temp[j*(2*row) + i];
        pivot_index = j;
      }
    }  
    if(pivot_index != i){
      for(int j=0; j<2*row; j++){
        pivot_temp = temp[ pivot_index * (2*row) + j ];
        temp[pivot_index * (2*row) + j] = temp[i*(2*row) + j];
        temp[i*(2*row) + j] = pivot_temp;    
      }
    }

    //division
    for(int j=0; j<2*row; j++){
      temp[i*(2*row) + j] /= pivot;    
    }
      
    //sweep
    for(int j=i+1; j<column; j++){
      double temp2 = temp[j*(2*row) + i];
      
      //sweep each row
      for(int k=0; k<row*2; k++){
        temp[j*(2*row) + k] -= temp2 * temp[ i*(2*row) + k ];    
      }
    }
  }
      
  //Sweep (up)
  for(int i=0; i<column-1; i++){
    for(int j=i+1; j<column; j++){
      double pivot = temp[ (column-1-j)*(2*row) + (row-1-i)];   
      for(int k=0; k<2*row; k++){
        temp[(column-1-j)*(2*row)+k] -= pivot * temp[(column-1-i)*(2*row)+k];    
      }
    }    
  }     
  
  //copy result
  for(int i=0; i<column; i++){
    for(int j=0; j<row; j++){
      sol[i*row + j] = temp[i*(2*row) + (j+row)];    
    }
  }
  free(temp);
  return;
}
