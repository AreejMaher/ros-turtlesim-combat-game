#include <Wire.h>

const int MPU6050_Address = 0x68;
const int PowerMang = 0x6B;
const int MPU6050_GYRO_Z_High = 0x47;
const int MPU6050_GYRO_Z_Low = 0x48;

const float GyroToDegree = 131.0;

// Kalman filter variables
float process_variance = 0.006 * 0.006;
float error_in_measurement = 0.01;
float error_in_estimate = 1.0;
float estimate = 0.0;

long Previous_Time = 0;
float Yaw = 0.0;
float GyroscopeZ_Offset = 0.0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

 
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(PowerMang);
  Wire.write(0); 
  Wire.endTransmission(true);

  // Calibrate the gyroscope to find the offset
  GyroscopeZ_Offset = calibrateGyro();
  Serial.print("Gyroscope Z Offset: ");
  Serial.println(GyroscopeZ_Offset);
  
  Previous_Time = millis();
}

void loop() {
  int16_t Gyroscope_Z = ReadGyoZ(); //Read
  float Gyroscope_Z_corrected = (Gyroscope_Z / GyroToDegree) - GyroscopeZ_Offset;  // subtracting the offset(from calibration) for each degree in Z

  long Current_Time = millis();
  float Elapsed_Time = (Current_Time - Previous_Time) / 1000.0; 
  Previous_Time = Current_Time;

  // Apply Kalman filter to the gyroscope data
  float filtered_yaw = kalmanFilterUpdate(Gyroscope_Z_corrected * Elapsed_Time); //( degree/sec )*sec = degree (angular displacement)
  Yaw += filtered_yaw;

  Serial.print("Yaw: ");
  Serial.println(Yaw);

  delay(200); 
}

int16_t ReadGyoZ() { //starting communication with MPU and reading z (yaw)
  Wire.beginTransmission(MPU6050_Address);
  Wire.write(MPU6050_GYRO_Z_High);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_Address, 2, true);

  int16_t Gyroscope_Z = Wire.read() << 8 | Wire.read();
  return Gyroscope_Z;
}

float kalmanFilterUpdate(float measurement) { // logic used in task 10.1
  float prev_estimate = estimate;
  float prev_error_in_estimate = error_in_estimate + process_variance;

  float kalman_gain = prev_error_in_estimate / (prev_error_in_estimate + error_in_measurement);
  estimate = prev_estimate + kalman_gain * (measurement - prev_estimate);
  error_in_estimate = (1 - kalman_gain) * prev_error_in_estimate;

  return estimate;
}

float calibrateGyro() { //to get offset (read 200 times then convert it to degree per second then while intecrating apply kalman filter to each then )/100
 
  float Gyroscope_Z_sum = 0;
  int i=0;
  Serial.println("Calibrating Gyroscope..."); 
  while (i<200) {
    int16_t Gyroscope_Z = ReadGyoZ();
    float Gyroscope_Z_corrected = Gyroscope_Z / GyroToDegree;
    Gyroscope_Z_sum += kalmanFilterUpdate(Gyroscope_Z_corrected);
    delay(3);
    i++ ;
  }

  return Gyroscope_Z_sum / 200;
}
