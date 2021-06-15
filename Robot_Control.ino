    int enA = 5; int in1 = 4; int in2 = 3; // Motor A (LEFT) pins
    int enB = 6; int in3 = 8; int in4 = 7; // Motor B (RIGHT) pins
    int speed = 100;

void setup() {
  Serial.begin(19200);
  
    // Configure motor L (Left)
    pinMode(enA, OUTPUT); 
    pinMode(in1, OUTPUT); 
    pinMode(in2, OUTPUT);  
    // Configure motor R (Right)
    pinMode(enB, OUTPUT); 
    pinMode(in3, OUTPUT); 
    pinMode(in4, OUTPUT); 
}

void loop() {
  for (int i=1;i<100;i++) {
  DriveForward(speed); }
  for (int i=1;i<100;i++) {
  DriveBackward(speed); }
  for (int i=1;i<100;i++) {
  TurnLeft(speed); }
  for (int i=1;i<100;i++) {
  TurnRight(speed); }
}

void DriveForward(int speed){
  Serial.println("Forward ----- Forward");
    // Turn on Motor L Forward
    digitalWrite(in1,0); digitalWrite(in2,1);
    analogWrite(enA,speed); // Set left motor to input speed 
    
    // Turn on Motor R Forward
    digitalWrite(in4,0); digitalWrite(in3,1);
    analogWrite(enB,speed); // Set left motor to input speed 
}
void DriveBackward(int speed){
  Serial.println("Backward ----- Backward");
    // Turn on Motor L Backward
    digitalWrite(in1,1); digitalWrite(in2,0);
    analogWrite(enA,speed); // Set left motor to input speed 
    
    // Turn on Motor R Backward
    digitalWrite(in4,1); digitalWrite(in3,0);
    analogWrite(enB,speed); // Set left motor to input speed 
}
void TurnLeft(int speed){
  Serial.println("Backward ----- Forward");
    // Turn on Motor L Backward
    digitalWrite(in1,1); digitalWrite(in2,0);
    analogWrite(enA,speed); // Set left motor to input speed 
    
    // Turn on Motor R Forward
    digitalWrite(in4,0); digitalWrite(in3,1);
    analogWrite(enB,speed); // Set left motor to input speed 
}
void TurnRight(int speed){
  Serial.println("Forward ----- Backward");
    // Turn on Motor L Forward
    digitalWrite(in1,0); digitalWrite(in2,1);
    analogWrite(enA,speed); // Set left motor to input speed 
    
    // Turn on Motor R Backward
    digitalWrite(in4,1); digitalWrite(in3,0);
    analogWrite(enB,speed); // Set left motor to input speed 
}
