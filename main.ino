#include "SparkFunMPU9250-DMP.h"

/*======================Global variable======================*/

// MPU9250
MPU9250_DMP imu;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float acc_angle_x, acc_angle_y;
float gyr_angle_x, gyr_angle_y;
float rad_to_reg = 180/3.141592654;

// Motor
int dir1_L_PIN = 2;
int dir2_L_PIN = 3;
int speed_L_PIN = 9;
int dir1_R_PIN = 4;
int dir2_R_PIN = 5;
int speed_L_PIN = 10;

// PID
float kp = 4.0;
float ki = 0.005;
float kd = 2.0;
float reference_angle = 0.0;
float kp_error = 0.0;
float ki_error = 0.0;
float kd_error = 0.0;
float kp_pass_error = 0.0;

// Receiver
enum re_command {forward=1, backward=2, left=3, right=4, stay=0};
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
  acc_angle_x = atan(accelY/sqrt(pow(accelX,2)+pow(accelZ,2)))*rad_to_reg;
  acc_angle_y = atan(-1*accelX/sqrt(pow(accelY,2)+pow(accelZ,2)))*rad_to_reg;
}
  
void printIMUData(void)
{  
  Serial.println("Angle: " + String(acc_angle_y));
}

re_command check_receiver()
{
  int re_1_state = digitalRead(re_1);
  int re_2_state = digitalRead(re_2);
  int re_3_state = digitalRead(re_3);
  int re_4_state = digitalRead(re_4);
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

float pid_control(){
  kp_error = acc_angle_x - reference_angle;
  ki_error += kp_error * dif_time;
  kd_error = kp_error - kp_pass_error;
  float kp_result = kp_error * kp;
  float ki_result = ki_error * ki;
  float kd_result = kd_error * kd;
  float final_control = kp_result + ki_result + kd_result;
  
  return final_control;
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
  pinMode(dir1PinA, OUTPUT);
  pinMode(dir2PinA, OUTPUT);
  pinMode(speedPinA, OUTPUT);
  pinMode(dir1PinB, OUTPUT);
  pinMode(dir2PinB, OUTPUT);
  pinMode(speedPinB, OUTPUT);

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
  dif_time = now_time - pas_time;
  pas_time = now_time;
  ;
  // Update IMU data
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    UpdateIMUData();
    printIMUData();
  }
  while(1){
    debug_receive_message();
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
    Serial.println("Received message: stay " + String(command));
  }

  float control_signal = pid_control();
/* keyboard test
  if (Serial.available() > 0) {
    int input_command = Serial.read();
    switch (input_command) {
      case '1': // Motor 1 Forward
        analogWrite(speedPinA, 255);//Sets speed variable via PWM
        digitalWrite(dir1PinA, LOW);
        digitalWrite(dir2PinA, HIGH);
        break;

      case '2': // Motor 1 Stop (Freespin)
        analogWrite(speedPinA, 0);
        digitalWrite(dir1PinA, LOW);
        digitalWrite(dir2PinA, HIGH);
        Serial.println("Motor 1 Stop");
        Serial.println("   ");
        break;

      case '3': // Motor 1 Reverse
        analogWrite(speedPinA, 255);
        digitalWrite(dir1PinA, HIGH);
        digitalWrite(dir2PinA, LOW);
        Serial.println("Motor 1 Reverse");
        Serial.println("   ");
        break;

      //______________Motor 2______________

      case '4': // Motor 2 Forward
        analogWrite(speedPinB, 255);
        digitalWrite(dir1PinB, LOW);
        digitalWrite(dir2PinB, HIGH);
        Serial.println("Motor 2 Forward");
        Serial.println("   ");
        break;

      case '5': // Motor 1 Stop (Freespin)
        analogWrite(speedPinB, 0);
        digitalWrite(dir1PinB, LOW);
        digitalWrite(dir2PinB, HIGH);
        Serial.println("Motor 2 Stop");
        Serial.println("   ");
        break;

      case '6': // Motor 2 Reverse
        analogWrite(speedPinB, 255);
        digitalWrite(dir1PinB, HIGH);
        digitalWrite(dir2PinB, LOW);
        Serial.println("Motor 2 Reverse");
        Serial.println("   ");
        break;
*/
    }
  }
}

