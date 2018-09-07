#include "SparkFunMPU9250-DMP.h"
#include "I2Cdev.h"
#include "Kalman.h"
/*======================Global variable======================*/
int PID_debug_mode = 1;
int RF_debug_mode = 0;
// MPU9250
MPU9250_DMP imu_9250;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll, pitch;
float gyroXrate, gyroYrate;
float rad_to_reg = 180 / 3.141592654;

// Kalman Filter
Kalman kalmanX;
Kalman kalmanY;
double gyroXangle, gyroYangle; // Gyroscope angle
double compAngleX, compAngleY; // Complementary filter angle
double kalAngleX, kalAngleY; // Angle after Kalman filter
double corrected_x, corrected_y; // Corrected with offset

// Motor
int dir1_L_PIN = 2;
int dir2_L_PIN = 3;
int speed_L_PIN = 9;
int dir1_R_PIN = 4;
int dir2_R_PIN = 5;
int speed_R_PIN = 10;

// PID
float kp = 8;
float ki = 0.1;
float kd = 1;
float reference_angle = 0.0;
float kp_error = 0.0;
float ki_error = 0.0;
float kd_error = 0.0;
float kp_pass_error = 0.0;
float kp_result = 0;
float ki_result = 0;
float kd_result = 0;
float final_result = 0;
float control_signal = 0;
float MIN_SPEED = 20;

// Receiver
enum re_command {forward = 1, backward = 2, left = 3, right = 4, stay = 0};
int re_1 = 11;
int re_2 = 8;
int re_3 = 7;
int re_4 = 6;

// Timer
float now_time;
float pas_time;
float dif_time;

/*======================support function======================*/
void UpdateIMUData(void)
{
  accelX = imu_9250.calcAccel(imu_9250.ax);
  accelY = imu_9250.calcAccel(imu_9250.ay);
  accelZ = imu_9250.calcAccel(imu_9250.az);
  gyroX = imu_9250.calcGyro(imu_9250.gx);
  gyroY = imu_9250.calcGyro(imu_9250.gy);
  gyroZ = imu_9250.calcGyro(imu_9250.gz);

  // Convert to deg/s
  roll = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * rad_to_reg;
  pitch = atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * rad_to_reg;
  gyroXrate = gyroX / 131.0;
  gyroYrate = gyroY / 131.0;
}

void printIMUData(float control)
{    
  Serial.println("Angle: " + String(roll) + "                     kalAngleY: " + String(kalAngleY)+ "                        Control signal: " + String(control)+ "                       kp_error: " + String(kp_error));
}

re_command check_receiver()
{
  int re_1_state = digitalRead(re_1);
  int re_2_state = digitalRead(re_2);
  int re_3_state = digitalRead(re_3);
  int re_4_state = digitalRead(re_4);
  if (RF_debug_mode) {
    Serial.println("PIN 1: " + String(re_1_state));
    Serial.println("PIN 2: " + String(re_2_state));
    Serial.println("PIN 3: " + String(re_3_state));
    Serial.println("PIN 4: " + String(re_4_state));
  }
  re_command command = stay;
  if (re_4_state == 1) {  // forward
    command = forward;
  }
  else if (re_3_state == 1) {  // backward
    command = backward;
  }
  else if (re_2_state == 1) {  // left
    command = left;
  }
  else if (re_1_state == 1) {  // right
    command = right;
  }

  return command;
}

float pid_control() { // ONLY PD RIGHT NOW
  kp_error = kalAngleY - reference_angle;
  ki_error += kp_error * dif_time;
  kd_error = (kp_error - kp_pass_error) / dif_time;
  kp_result = kp_error * kp;
  ki_result = ki_error * ki;
  kd_result = kd_error * kd;
  kp_pass_error = kp_error;
  final_result = kp_result + kd_result + ki_result;
  return final_result;
}

void kalman() {
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dif_time); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dif_time);
  gyroXangle += gyroXrate * dif_time; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dif_time;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dif_time) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dif_time) + 0.07 * pitch;
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}
/*======================setup======================*/
void setup() {
  Serial.begin(9600);

  // MPU-9250
  if (imu_9250.begin() != INV_SUCCESS)
  {
    imu_9250.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);// Enable all sensors:
    imu_9250.setGyroFSR(2000); // Set gyro to 2000 dps
    imu_9250.setAccelFSR(2); // Set accel to +/-2g
    imu_9250.setLPF(5); // Set LPF corner frequency to 5Hz
    imu_9250.setSampleRate(10); // Set sample rate to 10Hz
  }

  // L298N
  pinMode(dir1_L_PIN, OUTPUT);
  pinMode(dir2_L_PIN, OUTPUT);
  pinMode(speed_L_PIN, OUTPUT);
  pinMode(dir1_R_PIN, OUTPUT);
  pinMode(dir2_R_PIN, OUTPUT);
  pinMode(speed_R_PIN, OUTPUT);

  // Receiver
  pinMode(re_1, INPUT);
  pinMode(re_2, INPUT);
  pinMode(re_3, INPUT);
  pinMode(re_4, INPUT);

  // Timer
  pas_time = millis();
}

/*======================main loop======================*/
void loop() {
  // calculate time
  now_time = millis();
  dif_time = (now_time - pas_time)/1000; // in seconds. We work in ms so we haveto divide the value by 1000
  pas_time = now_time;
  // Update IMU data
  if ( imu_9250.dataReady() )
  {
    imu_9250.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    UpdateIMUData();
    kalman();
  }

  /*
    // Check the receive message
    re_command command = check_receiver();
    if (command == forward) {
    Serial.println("Received message: forward " + String(command));
    }
    else if (command == backward) {
    Serial.println("Received message: backward " + String(command));
    }
    else if (command == left) {
    Serial.println("Received message: left " + String(command));
    }
    else if (command == right) {
    Serial.println("Received message: right " + String(command));
    }
    else {
    //Serial.println("Received message: stay " + String(command));
    }*/

  // PID and motor
  float control_signal = pid_control();
  control_signal = constrain(control_signal, -90, 90);
  if (control_signal > 0 && control_signal < MIN_SPEED) {control_signal = MIN_SPEED;}
  else if (control_signal < 0  && control_signal > -MIN_SPEED) {control_signal = -MIN_SPEED;}
  
  if (control_signal < 0) {
    analogWrite(speed_L_PIN, abs(control_signal));
    analogWrite(speed_R_PIN, abs(control_signal));
    digitalWrite(dir1_L_PIN, HIGH);
    digitalWrite(dir2_L_PIN, LOW);
    digitalWrite(dir1_R_PIN, HIGH);
    digitalWrite(dir2_R_PIN, LOW);

  }
  else {
    analogWrite(speed_L_PIN, control_signal);
    analogWrite(speed_R_PIN, control_signal);
    digitalWrite(dir1_L_PIN, LOW);
    digitalWrite(dir2_L_PIN, HIGH);
    digitalWrite(dir1_R_PIN, LOW);
    digitalWrite(dir2_R_PIN, HIGH);
  }
  printIMUData(control_signal);
}

