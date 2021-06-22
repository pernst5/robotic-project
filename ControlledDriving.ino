/*
 * Created by: Preston Ernst
 * Date: 5/26/2021
 * 
 */

//////////////////  Variables for MPU   ///////////////////////
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float yaw;
float elapsedTimeMPU, currentTimeMPU, previousTimeMPU;
float GyroErrorZ;
int c = 0; // A counter variable for gyroError

////////////////////    For PID Control for driving straight    ////////////////////////
double P = 2; // proportional constant
double I = 0; // integral constant
double D = 0; // derivative constant

////////////////////    For PID Control for turning    ////////////////////////
double Kp = 0.1; // proportional constant
double Ki = 0; // integral constant
double Kd = 5; // derivative constant

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

///////  For Motors  //////////
int enA = 5; int in1 = 4; int in2 = 3; // LEFT motor pins
int enB = 6; int in3 = 8; int in4 = 7; // RIGHT motor pins
int speed = 120;
int time;

///// Other assorted Pins /////
int IRObstaclePin = 2;
int buttonPin = 12;
int motionPin = 11;


void setup() {
  // Configure the Left motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Configure the Right motor
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(buttonPin, INPUT);     // Button Pin will be an input
  pinMode(IRObstaclePin, INPUT); // Obstacle detection Pin will be an input
  pinMode(motionPin,INPUT);      // Obstacle detection Pin will be an input

  Serial.begin(19200);               // Set up serial communication
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // end the transmission
}

void loop() {

float driveTime = 1; // Adjust the drive time in seconds
float dtime = 700;   // Use dtime as the delay time

   ButtonWait();
   delay(2000);

/// Square /////

  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(90,speed);   delay(dtime);
  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(90,speed);   delay(dtime);
  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(90,speed);   delay(dtime);
  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(-90,speed); delay(dtime);  TurnWithGyro(-90,speed);  delay(dtime*3);

  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(-90,speed);   delay(dtime);
  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(-90,speed);   delay(dtime);
  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(-90,speed);   delay(dtime);
  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(-90,speed);   delay(dtime*30);


////// Straight Driving Demonstration  /////

  DriveForward(speed,driveTime*2);  Stop();    delay(dtime*2);
  TurnWithGyro(90,speed);   delay(dtime);
  DriveForward(speed,0.3);  Stop();    delay(dtime);
  TurnWithGyro(90,speed);   delay(dtime);
  DriveForward(speed,driveTime*2);  Stop();    delay(dtime*2);
  
  TurnWithGyro(-90,speed);   delay(dtime);
  DriveForward(speed,0.3);  Stop();    delay(dtime);
  TurnWithGyro(-90,speed);   delay(dtime);

  DriveForward(speed,driveTime*2);  Stop();    delay(dtime*2);
  TurnWithGyro(90,speed);   delay(dtime);
  DriveForward(speed,0.3);  Stop();    delay(dtime);
  TurnWithGyro(90,speed);   delay(dtime);
  DriveForward(speed,driveTime*2);  Stop();    delay(dtime*2);
  
  TurnWithGyro(-90,speed);   delay(dtime);
  DriveForward(speed,0.3);  Stop();    delay(dtime);
  TurnWithGyro(-90,speed);   delay(dtime);


/////////////////// Angle Turning Demo /////////////////////////////

  TurnWithGyro(90,speed);   delay(dtime);
  TurnWithGyro(-90,speed);   delay(dtime);
  TurnWithGyro(-90,speed);   delay(dtime);
  TurnWithGyro(90,speed);   delay(dtime);
  TurnWithGyro(175,speed);   delay(dtime);
  TurnWithGyro(-175,speed);   delay(dtime);
  TurnWithGyro(330,speed);   delay(dtime);

}

void DriveForwardUntilObstacle(int speed,float driveTime){
  // This function will drive forward until either driveTime is reached or an obstacle is detected
  int obstacle = digitalRead(IRObstaclePin); // will read 1 if there is no obstacle
  while ((obstacle == 1) && (millis() < 1000*driveTime)) {  // will loop until obstacle is found (obstacle = 0)
    DriveForward(speed,driveTime); // Call the function to drive forward
    obstacle = digitalRead(IRObstaclePin); // Check to see if there is an obsticle
  }
  if (obstacle == 0) {
  Serial.println("Obstacle detected"); // Print out if an obstacle is found
  }
}

void DriveForward(int speed,float driveTime){
    // This function will drive forward using the MPU and a PID controler to follow a straight line
    Serial.println("Start of DriveForward Function");
    yaw = 0;      // reset the heading everytime the drive foreward function is called 
    int goalAngle = 0;         // The angle the robot should keep to drive straight
    int speedL = speed;  int speedR = speed; // Want each motor to have independent speed values
    // Set the left motor to go Forward
    digitalWrite(in1,0); digitalWrite(in2,1);
    // Set the right motor to go Forward
    digitalWrite(in4,0); digitalWrite(in3,1);
    int millis1 = millis();
    float runTime = 0; // Start runTime is 0s

    while ( runTime < (driveTime * 1000) ) {
        double yaw = ReadGyro();
        output = computePID(yaw,goalAngle,P,I,D);
  
        if (output > 0){
          // Turn left
          speedL = (speed);
          speedR = (abs(output) * 5 + speed); // Have the speed of the right wheel to increase to turn the robot left
          Serial.print(speedL); Serial.print(" / "); Serial.println(speedR);  // Print out the speed of each motor for debugging 
        }
        else if (output < 0){
          // Turn Right
          speedL = (abs(output) * 5 + speed); // Have the speed of the left wheel to increase to turn the robot left
          speedR = (speed);
          Serial.print(speedL); Serial.print(" / "); Serial.println(speedR);  // Print out the speed of each motor for debugging 
        } else { Stop();}
       analogWrite(enA,speedL); // Set left motor to its adjusted speed 
       analogWrite(enB,speedR); // Set right motor to its adjusted speed 
       runTime = (millis() - millis1);
    }
}

void TurnWithGyro(int goalAngle,int speed) {
  yaw = 0; // reset the heading everytime the turn function is called
  output =  computePID(yaw,goalAngle,Kp,Ki,Kd); // based on the current yaw and the goal angle compute the current output
  
  while (abs(output) > .002){   // This is used to stop the robot when it is very close to the desired angle
    double yaw = ReadGyro(); // Read the current yaw value
    output =  computePID(yaw,goalAngle,Kp,Ki,Kd); // based on the current yaw and the goal angle compute the current output

    if (output > 0){ 
      int speedturn = (abs(output) + speed);
      TurnLeft(speedturn);
      Serial.println("TurnLeft");
      Serial.println(speedturn);
    }
    if (output < 0){
      int speedturn = (abs(output) + speed);
      TurnRight(speedturn);
      Serial.println("TurnRight");
      Serial.println(speedturn);
    }
  }
  Stop(); // Stop the robot after it has reached the correct angle
}

void TurnRight(int speed) {
  // Turn on Motor L Forward
  digitalWrite(in1, 0); digitalWrite(in2, 1);
  analogWrite(enA, speed); // Set left motor to input speed

  // Turn on Motor R Backward
  digitalWrite(in4, 1); digitalWrite(in3, 0);
  analogWrite(enB, speed); // Set left motor to input speed
  Serial.println("Forward --- Backward");
}

void TurnLeft(int speed) {
  // Turn on Motor L Backward
  digitalWrite(in1, 1); digitalWrite(in2, 0);
  analogWrite(enA, speed); // Set left motor to input speed

  // Turn on Motor R Forward
  digitalWrite(in4, 0); digitalWrite(in3, 1);
  analogWrite(enB, speed); // Set left motor to input speed
  Serial.println("Backward --- Forward");
}
void DriveBackward(int speed) {
  // Turn on Motor L Backward
  digitalWrite(in1, 1); digitalWrite(in2, 0);
  analogWrite(enA, speed); // Set left motor to input speed

  // Turn on Motor R Backward
  digitalWrite(in4, 1); digitalWrite(in3, 0);
  analogWrite(enB, speed); // Set left motor to input speed
  Serial.println("Backward --- Forward");
}

void Stop() {
  // Stop both motors by setting the speed to 0
  analogWrite(enA, 0); 
  analogWrite(enB, 0); 
  Serial.println("Stop");
}

void ButtonWait() {
  Serial.println("waiting");
  bool buttonPress = 0;
  while (buttonPress == 0){
        buttonPress = digitalRead(buttonPin); // check button to see if it is being pressed
  }
  Serial.println("Starting");
  delay(1000);
}

void motionDetect() {
 Serial.println("waiting");
 int motion = digitalRead(motionPin); 
 while (motion != HIGH) {            // if motion detected
    motion = digitalRead(motionPin); 
 } 
  Serial.println("Starting");
  delay(1000);
}

double ReadGyro(){
  // === Read gyroscope data === //
  previousTimeMPU = currentTimeMPU;     // Previous time is stored before the actual time read
  currentTimeMPU = millis();            // Current time actual time read
  elapsedTimeMPU = (currentTimeMPU - previousTimeMPU) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x47); // Gyro data first register address for the GyroZ data (0x47)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);    // Read 2 registers total
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  // Correct the outputs with the calculated error values
  GyroZ = GyroZ + 0.85; // GyroErrorZ ~ (-0.85)
  yaw =  yaw + GyroZ * elapsedTimeMPU;
  // Serial.println(yaw); // Use to check calibration
  return yaw;
}

void FindGyroError(){
  // This function can be called to calculate the gyro error term that can be used above
  while (c < 1000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);  // Serial.println(GyroErrorZ);
    c++;
  }
  GyroErrorZ = GyroErrorZ / 1000.0;
  // Serial.println(GyroErrorZ);
}

double computePID(double input,int goalAngle,double P,double I,double D) {
  
  currentTime = millis();   // find current time
  elapsedTime = double(currentTime - previousTime);  //compute time elapsed from previous computation

  error = goalAngle - input; // calculate the proportional error
  cumError += error * elapsedTime; // calculate the integral of the error
  rateError = (error - lastError) / elapsedTime; // calculate the derivative of the error

  double out = P * error + I * cumError + D * rateError;  //compute the PID output based on the error and the PID constatnts

  lastError = error;                             //update current error
  previousTime = currentTime;                    //update current time
  return out;              //have function return the PID output
}
