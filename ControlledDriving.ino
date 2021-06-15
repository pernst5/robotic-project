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
int c = 0;

////////////////////    For PID Control for driving straight    ////////////////////////
double Kp = 2; // proportional constant
double Ki = 0; // integral constant
double Kd = 0; // derivative constant

////////////////////    For PID Control for turning    ////////////////////////
double Kp_t = 0.1; // proportional constant
double Ki_t = 0; // integral constant
double Kd_t = 5; // derivative constant

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

///////  For Motors  //////////
int enA = 5; int in1 = 4; int in2 = 3; // Motor A (LEFT) pins
int enB = 6; int in3 = 8; int in4 = 7; // Motor B (RIGHT) pins
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

  pinMode(buttonPin, INPUT);     //Button Pin will be an input
  pinMode(IRObstaclePin, INPUT); // Obstacle detection Pin will be an input
  pinMode(motionPin,INPUT);

  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // end the transmission
}

void loop() {
  // motionDetect();
  //  delay(1000);
  //  FindGyroError();
   ButtonWait();
   delay(2000);
/*
  while (Serial.available() == 0) {
    // Wait for User to Input Data
  }
  char driveCommand = Serial.read();

switch(driveCommand) {

   case 'w':
      Serial.println("Drive Forward");
      DriveForward(speed);
      break;
   case 's':
      Serial.println("Drive backwards");
      DriveBackward(speed);
      break;
   case 'a':
      Serial.println("Turn Left");
      TurnLeft(speed);
      break;
   case 'd':
      Serial.println("Turn Right");
      TurnRight(speed);
      break;
   case 'x':
      Serial.println("Stop");
      Stop();
      break;
      
   default : 
   Serial.println("Try Again");
}

*/
  //DriveForwardUntilObstacle(speed,1);
  //delay(5000);
float driveTime = 1; // In seconds
int dtime = 200;
  /*
  for (int k=0;k<5;k++) {
  }
TurnWithGyro(90,speed);   delay(500);
TurnWithGyro(-90,speed);   delay(500);
TurnWithGyro(90,speed);   delay(500);
TurnWithGyro(-90,speed);   delay(500);
  */

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

////////////////////////////////////////////////////////////
/*
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
  */
/*
  TurnWithGyro(45,speed);   delay(dtime);
  DriveForward(speed,driveTime*sqrt(2));  Stop();    delay(dtime);
  TurnWithGyro(45,speed);   delay(dtime);   TurnWithGyro(90,speed); delay(dtime);
  DriveForward(speed,driveTime);  Stop();    delay(dtime); 
  TurnWithGyro(90,speed);   delay(dtime);
  DriveForward(speed,driveTime);  Stop();    delay(dtime);
  TurnWithGyro(90,speed);   delay(dtime);
  */
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
    float runTime = 0;

    while ( runTime < (driveTime * 1000) ) {                             ///// Change here

      double yaw = ReadGyro();
      output = computePID(yaw,goalAngle);

      if (output > 0){
        // Turn left
        speedL = (speed);
        speedR = (abs(output) * 5 + speed); // Have the speed of the right wheel to increase to turn the robot left
         //    Serial.print(speedL); Serial.print(" / "); Serial.println(speedR);  // Print out the speed of each motor for debugging 
      }
      else if (output < 0){
        // Turn Right
        speedL = (abs(output) * 5 + speed); // Have the speed of the left wheel to increase to turn the robot left
        speedR = (speed);
          //   Serial.print(speedL); Serial.print(" / "); Serial.println(speedR);  // Print out the speed of each motor for debugging 
      } else { Stop();}
     analogWrite(enA,speedL); // Set left motor to its adjusted speed 
     analogWrite(enB,speedR); // Set right motor to its adjusted speed 
     //Serial.println(i);
     runTime = (millis() - millis1);
     Serial.println(runTime);
    }
}

void TurnWithGyro(int goalAngle,int speed) {
  yaw = 0; // reset the heading everytime the turn function is called
  output =  computePIDturn(yaw,goalAngle); // based on the current yaw and the goal angle compute the current output
  
  while (abs(output) > .002){
    // delay(200);
    double yaw = ReadGyro(); // Read the current yaw value
    //Serial.println(yaw);
    output =  computePIDturn(yaw,goalAngle) - 0.002*goalAngle; // based on the current yaw and the goal angle compute the current output

    if (output > 0){ 
      int speedturn = (abs(output) + speed);
      TurnLeft(speedturn);
      // Serial.println("TurnLeft");
      //Serial.println(speedturn);
    }
    if (output < 0){
      int speedturn = (abs(output) + speed);
      TurnRight(speedturn);
      // Serial.println("TurnRight");
      //Serial.println(speedturn);
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
  //Serial.println("Forward --- Backward");
}

void TurnLeft(int speed) {
  // Turn on Motor L Backward
  digitalWrite(in1, 1); digitalWrite(in2, 0);
  analogWrite(enA, speed); // Set left motor to input speed

  // Turn on Motor R Forward
  digitalWrite(in4, 0); digitalWrite(in3, 1);
  analogWrite(enB, speed); // Set left motor to input speed
  //Serial.println("Backward --- Forward");
}
void DriveBackward(int speed) {
  // Turn on Motor L Backward
  digitalWrite(in1, 1); digitalWrite(in2, 0);
  analogWrite(enA, speed); // Set left motor to input speed

  // Turn on Motor R Backward
  digitalWrite(in4, 1); digitalWrite(in3, 0);
  analogWrite(enB, speed); // Set left motor to input speed
  //Serial.println("Backward --- Forward");
}

void Stop() {
  // Stop both motors by setting the speed to 0
  analogWrite(enA, 0); 
  analogWrite(enB, 0); 
  //Serial.println("Stop");
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
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);    // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroZ = GyroZ + 0.85; // GyroErrorZ ~ (-0.88)
  yaw =  yaw + GyroZ * elapsedTimeMPU;
  // Serial.println(yaw); // Use to check calibration
  return yaw;
}

void FindGyroError(){
  // === Read gyroscope data === //
  while (c < 1000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);  // Serial.println(GyroErrorZ);
    c++;
  }

  GyroErrorZ = GyroErrorZ / 1000.0;
  // Serial.println(GyroErrorZ);
}

double computePID(double input,int goalAngle) {
  
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation

  error = goalAngle - input;                     // determine error
  cumError += error * elapsedTime;               // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = Kp * error + Ki * cumError + Kd * rateError;  //PID output

  lastError = error;                             //remember current error
  previousTime = currentTime;                    //remember current time

  return out;              //have function return the PID output
}
double computePIDturn(double input,int goalAngle) {
  
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation

  error = goalAngle - input;                     // determine error
  cumError += error * elapsedTime;               // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = Kp_t * error + Ki_t * cumError + Kd_t * rateError;  //PID output

  lastError = error;                             //remember current error
  previousTime = currentTime;                    //remember current time

  return out;              //have function return the PID output
}
