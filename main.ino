#include "SparkFunMPU9250-DMP.h"



/*======================Global variable======================*/
MPU9250_DMP imu;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// Motor 1
int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 9; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 4;
int dir2PinB = 5;
int speedPinB = 10; // Needs to be a PWM pin to be able to control motor speed





/*======================support function======================*/
void UpdateIMUData(void)
{  
  accelX = imu.calcAccel(imu.ax);
  accelY = imu.calcAccel(imu.ay);
  accelZ = imu.calcAccel(imu.az);
  gyroX = imu.calcGyro(imu.gx);
  gyroY = imu.calcGyro(imu.gy);
  gyroZ = imu.calcGyro(imu.gz);
}
void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  accelX = imu.calcAccel(imu.ax);
  accelY = imu.calcAccel(imu.ay);
  accelZ = imu.calcAccel(imu.az);
  gyroX = imu.calcGyro(imu.gx);
  gyroY = imu.calcGyro(imu.gy);
  gyroZ = imu.calcGyro(imu.gz);
    
  Serial.println("Accel: " + String(accelX) + ", " +
              String(accelY) + ", " + String(accelZ) + " g");
  Serial.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
  delay(1000);
}


/*======================setup======================*/
void setup() {
  Serial.begin(9600);
  
  // MPU-9250
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);// Enable all sensors:
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(5); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz
  
  // L298N
  pinMode(dir1PinA, OUTPUT);
  pinMode(dir2PinA, OUTPUT);
  pinMode(speedPinA, OUTPUT);
  pinMode(dir1PinB, OUTPUT);
  pinMode(dir2PinB, OUTPUT);
  pinMode(speedPinB, OUTPUT);
}

/*======================main loop======================*/
void loop() {

  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    UpdateIMUData();
  }

  // Initialize the Serial interface:

  if (Serial.available() > 0) {
    int input_command = Serial.read();

    switch (input_command) {
      //______________Motor 1______________

      case '1': // Motor 1 Forward
        analogWrite(speedPinA, 255);//Sets speed variable via PWM
        digitalWrite(dir1PinA, LOW);
        digitalWrite(dir2PinA, HIGH);
        Serial.println("Motor 1  Forward"); // Prints out “Motor 1 Forward” on the serial monitor
        Serial.println("   "); // Creates a blank line printed on the serial monitor

        analogWrite(speedPinB, 255);
        digitalWrite(dir1PinB, LOW);
        digitalWrite(dir2PinB, HIGH);
        Serial.println("Motor 2 Forward");
        Serial.println("   ");
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
    }
  }
}

