#include "SparkFunMPU9250-DMP.h"

/*======================Global variable======================*/
int PID_debug_mode = 1;
int RF_debug_mode = 0;
// MPU9250
MPU9250_DMP imu;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float acc_angle_x, acc_angle_y;
float gyr_angle_x, gyr_angle_y;
float rad_to_reg = 180 / 3.141592654;

// Motor
int dir1_L_PIN = 2;
int dir2_L_PIN = 3;
int speed_L_PIN = 9;
int dir1_R_PIN = 4;
int dir2_R_PIN = 5;
int speed_R_PIN = 10;

// PID
float kp = 60.0;
float ki = 1.00;
float kd = 38;
float reference_angle = 0.0;
float kp_error = 0.0;
float ki_error = 0.0;
float kd_error = 0.0;
float kp_pass_error = 0.0;
float kp_result = 0;
float ki_result = 0;
float kd_result = 0;
float final_result = 0;
// Receiver
enum re_command {forward = 1, backward = 2, left = 3, right = 4, stay = 0};
int re_1 = 11;
int re_2 = 8;
int re_3 = 7;
int re_4 = 6;

// Timer
unsigned long now_time;
unsigned long pas_time;
unsigned long dif_time;

/*======================support function======================*/

void UpdateIMUData(void)
{
  accelX = imu.calcAccel(imu.ax);
  accelY = imu.calcAccel(imu.ay);
  accelZ = imu.calcAccel(imu.az);
  gyroX = imu.calcGyro(imu.gx);
  gyroY = imu.calcGyro(imu.gy);
  gyroZ = imu.calcGyro(imu.gz);
  acc_angle_x = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * rad_to_reg;
  acc_angle_y = atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * rad_to_reg;
}

void printIMUData(void)
{
  Serial.println("Angle:                                                                          " + String(acc_angle_y));
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
  kp_error = acc_angle_y - reference_angle;
  ki_error += kp_error * dif_time;
  kd_error = (kp_error - kp_pass_error)/dif_time;
  kp_result = kp_error * kp;
  ki_result = ki_error * ki;
  kd_result = kd_error * kd;
  kp_pass_error = kp_error;
  final_result = kp_result + kd_result;
  return final_result;
}



/*======================setup======================*/
void setup() {
  Serial.begin(9600);

  // MPU-9250
  if (imu.begin() != INV_SUCCESS)
  {
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);// Enable all sensors:
    imu.setGyroFSR(2000); // Set gyro to 2000 dps
    imu.setAccelFSR(2); // Set accel to +/-2g
    imu.setLPF(5); // Set LPF corner frequency to 5Hz
    imu.setSampleRate(10); // Set sample rate to 10Hz
    imu.setCompassSampleRate(10); // Set mag rate to 10Hz
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
  dif_time = (now_time - pas_time); // in seconds. We work in ms so we haveto divide the value by 1000
  pas_time = now_time;
  ;
  // Update IMU data
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    UpdateIMUData();
    //printIMUData();
  }
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
  }

  float control_signal = pid_control();
  control_signal = constrain(control_signal, -150, 150);
  if (control_signal < 0) {
    analogWrite(speed_L_PIN, abs(control_signal));
    analogWrite(speed_R_PIN, abs(control_signal));
    digitalWrite(dir1_L_PIN, LOW);
    digitalWrite(dir2_L_PIN, HIGH);
    digitalWrite(dir1_R_PIN, LOW);
    digitalWrite(dir2_R_PIN, HIGH);
  }
  else {
    analogWrite(speed_L_PIN, control_signal);
    analogWrite(speed_R_PIN, control_signal);
    digitalWrite(dir1_L_PIN, HIGH);
    digitalWrite(dir2_L_PIN, LOW);
    digitalWrite(dir1_R_PIN, HIGH);
    digitalWrite(dir2_R_PIN, LOW);
  }
  if(PID_debug_mode) {
    Serial.println("control signal : " + String(control_signal));
  }
}

