#include <Wire.h>
#include <PID_v1.h>  // Include the PID library
#include <Adafruit_VL53L0X.h>  // Include the Adafruit VL53L0X Lidar library
#include <MPU6050_light.h>  // Include the MPU6050_light library

// Motor Pins
#define IN1 12
#define IN2 13
#define ENA 14
#define IN3 25
#define IN4 26
#define ENB 27

// Encoder Pins
#define ENC_A1 32
#define ENC_B1 33
#define ENC_A2 34
#define ENC_B2 35

// IR Sensor Pins
#define IR_LEFT_PIN 4
#define IR_RIGHT_PIN 5


// Lidar setup
Adafruit_VL53L0X lidar;  // Create a Lidar object
int lidarDistance;

// MPU6050 setup (using MPU6050_light)
MPU6050 mpu(Wire);  // Create the MPU6050_light object
float mpuAngleZ = 0.0;  // Current Z-axis angle from MPU6050_light
int cellTime = 700;

// Encoder variables
volatile int encoderCountA = 0;
volatile int encoderCountB = 0;
//float wheelCircumference = 20.0; // Adjust to the wheel circumference in cm
float currentDistance = 0.0;

static unsigned long lastPrint = 0;
static unsigned long lastPrint2 = 0;


// PID setup for turning
double setPoint = 0;   // Set target angle (0 for forward)
double input, output;
double kp = 100;
double kd = 18;
double ki = 2;

const float wheelDiameter       = 4.5;
const float wheelCircumference  = 3.1416 * wheelDiameter;
const int   encoderCountsPerRevolution = 360;

long driveStartEncoder1 = 0;
long driveStartEncoder2 = 0;


PID pid(&input, &output, &setPoint, kp, ki, kd, DIRECT); // Using the PID library



// **PID for Direction Correction**
double Input, Output; // Input from the MPU, output for motor speed adjustment

double KP = 2, KI = 0, KD = 0.65; // PID constants
double Error = 0;
int LoopDelayTime = 40;
double Proportional = 0;
double Integral = 0;
double Derivative = 0;
double PreviousError = 0;
double setPoint1 = 0;   // Set target angle (0 for forward)

PID pid2(&Input, &Output, &setPoint1, KP, KI, KD, DIRECT); // Using the PID library


// void moveForward(int delayTime) {
//     // Reset encoder count to track this movement
    
//     // Move both motors forward at equal speed
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//   analogWrite(ENA, 140 - Output);
//   analogWrite(ENB, 140 + Output);
//   setPoint1 = 0;
//    float time1 = millis() + delayTime;

//     while(true){
      
//       if(millis() > time1){
//         break;
//       }
//   Input = getAngle();  // Replace with actual sensor reading code
  
//   pid2.Compute();

//   // Apply motor speeds
//   analogWrite(ENA, output);
//   analogWrite(ENB, output);
  

//   // Delay for the loop (PID tuning)
//   delay(LoopDelayTime);  // Convert seconds to milliseconds


//   }
    
//     Serial.println("Moving forward...");
    
//     // Wait until the robot has moved 20 cm (180 encoder pulses)
    
// }

//Motor control functions
void moveForward(int delayTime) {
    // Reset encoder count to track this movement
    
    // Move both motors forward at equal speed
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  analogWrite(ENA, 110);
  analogWrite(ENB, 110);
  setPoint1 = getAngle();
   float time1 = millis() + delayTime;


    while(true){
      
      if(millis() > time1){
        turnToConverge(setPoint1);
        break;
      }
  Input = getAngle();  // Replace with actual sensor reading code
  
  // Calculate PID components
  Error = setPoint1 - Input;  // Setpoint is 0 (want to move straight)
  
  Proportional = Error;
  Integral = Integral + (Error * LoopDelayTime);
  Derivative = (Error - PreviousError) / LoopDelayTime;

  // Calculate PID output
  double output = (KP * Proportional) + (KI * Integral) + (KD * Derivative);

  // Control motors with the calculated output
  int motorLeftSpeed = 120 - output;  // Adjust left motor speed
  int motorRightSpeed = 120 + output; // Adjust right motor speed
  
  // Ensure motor speed is within range (0-255)
  motorLeftSpeed = constrain(motorLeftSpeed, 0, 125);
  motorRightSpeed = constrain(motorRightSpeed, 0, 125);

  // Apply motor speeds
  analogWrite(ENA, motorLeftSpeed);
  analogWrite(ENB, motorRightSpeed);
  
  // Save the current error for the next loop iteration
  PreviousError = Error;
    Serial.println("Moving forward...");

  // Delay for the loop (PID tuning)
  delay(LoopDelayTime);  // Convert seconds to milliseconds


  }
    
    Serial.println("Moving forward...");
    
    // Wait until the robot has moved 20 cm (180 encoder pulses)
    
}

void moveStraight(int targetHeading, int speed) {
    setPoint1 = targetHeading;  // Desired heading (usually 0 for straight movement)
    input = getAngle();         // Get the current angle from MPU6050
    double error = setPoint1 - input; // Calculate initial error

    Serial.println("Moving Forward...");
    Serial.print("Target Heading: ");
    Serial.println(setPoint1);

    while (true) {  // Continuous movement
        input = getAngle(); // Update current angle
        error = setPoint1 - input; // Compute the current error
        
        // If the error is too large, use turnToConverge() to correct heading
        if (abs(error) > 10) {  // Threshold for correction
            Serial.println("Significant heading error detected! Adjusting...");
            turnToConverge(setPoint1); // Call turn function to fix alignment
        }

        // Compute PID correction
        pid2.Compute();

        // Constrain the output to prevent excessive speed changes
        Output = constrain(Output, -50, 50); // Small adjustments

        // Apply motor speeds with PID correction
        int leftMotorSpeed = speed - Output; // Reduce left speed if needed
        int rightMotorSpeed = speed + Output; // Increase right speed if needed

        // Ensure motor speeds stay within 0-255 range
        leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
        rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

        // Move forward with adjusted speeds
        analogWrite(ENA, leftMotorSpeed);
        analogWrite(ENB, rightMotorSpeed);
        
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        Serial.print("Current Angle: ");
        Serial.print(input);
        Serial.print(" | Adjusted Speeds - Left: ");
        Serial.print(leftMotorSpeed);
        Serial.print(" | Right: ");
        Serial.println(rightMotorSpeed);

        delay(50); // Small delay for stability
    }
}



void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    Serial.println("Motors stopped.");
}
void turnToConverge(int targetAngle) {
    setPoint = targetAngle; // Set the target angle
    input = getAngle();     // Get current angle from MPU6050_light
    double error = setPoint - input; // Calculate initial error

    Serial.println("Starting Turn...");
    Serial.print("Target Angle: ");
    Serial.println(setPoint);
    
    // Turn until we reach the target angle
    while (abs(error) > 1) { // Stop when error is small enough
        input = getAngle(); // Update current angle
        error = setPoint - input; // Compute new error
        
        pid.Compute(); // Compute the PID output

        // Constrain the output to prevent too high/low speed
        output = constrain(output, 50, 255); // Ensure minimum speed to avoid stalling

        // Determine direction based on error
        if (error > 0) { // Need to turn left
            analogWrite(ENA, output);
            analogWrite(ENB, output);

            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);

            Serial.print("Current Angle: ");
            Serial.print(input);
            Serial.print(" degrees | Turning Left | Speed: ");
            Serial.println(output);
        } 
        else { // Need to turn right
            analogWrite(ENA, output);
            analogWrite(ENB, output);

            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);

            Serial.print("Current Angle: ");
            Serial.print(input);
            Serial.print(" degrees | Turning Right | Speed: ");
            Serial.println(output);
        }

        delay(20); // Small delay for stability
    }

    // Stop the motors once the target angle is reached
    stopMotors();
    Serial.println("Turn Completed.");
}



void turnLeft(int speed) {
    // Set the target angle to 90 degrees
    setPoint = 90;
    input = getAngle();  // Get current angle from MPU6050_light
    
    // Start the PID computation
    pid.Compute();
            Serial.print("____________________________________________________");

    // Turn until we reach the target angle (90 degrees)
    while (1){
    while (getAngle() < (input + setPoint)) {
        pid.Compute(); // Compute the PID output
        
           // Apply the PID output to control motor speed for turning
        analogWrite(ENA, output);
        analogWrite(ENB, output);
                Serial.print(output);


        // Rotate left
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        
        Serial.print("Current Angle: ");
        Serial.print(getAngle()); // Get current angle in degrees
        Serial.println(" degrees    Turning left");
    }
        while (getAngle() > (input + setPoint)) {
        pid.Compute(); // Compute the PID output
        
           // Apply the PID output to control motor speed for turning
        analogWrite(ENA, output);
        analogWrite(ENB, output);
                Serial.print(output);


        // Rotate left
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        
        Serial.print("Current Angle: ");
        Serial.print(getAngle()); // Get current angle in degrees
        Serial.println(" degrees    Turning Right");
    }
    if(getAngle() - (input + setPoint) < 1)
    break;
    }

    // Stop the motors once the target angle is reached
    stopMotors();
    Serial.println("Turned left 90 degrees.");
}

void turnRight(int speed) {
    // Set the target angle to -90 degrees (right turn)
    setPoint = -90;
    input = getAngle();  // Get current angle from MPU6050_light
    
    // Start the PID computation
    pid.Compute();
    while(1){
    // Turn until we reach the target angle (-90 degrees)
    while (getAngle() > (input + setPoint) ) {
        pid.Compute(); // Compute the PID output
        
         // Apply the PID output to control motor speed for turning
        analogWrite(ENA, output);
        analogWrite(ENB, output);
        
        // Rotate right
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        
        Serial.print("Current Angle: ");
        Serial.print(getAngle()); // Get current angle in degrees
        Serial.println(" degrees    Turning right");
    }

     // Turn until we reach the target angle (-90 degrees)
    while (getAngle() < (input + setPoint) ) {
        pid.Compute(); // Compute the PID output
        
         // Apply the PID output to control motor speed for turning
        analogWrite(ENA, output);
        analogWrite(ENB, output);
        
        // Rotate left
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        
        Serial.print("Current Angle: ");
        Serial.print(getAngle()); // Get current angle in degrees
        Serial.println(" degrees    Turning left");
    }
    if(getAngle() - (input + setPoint) < 1)
    break;
}

    // Stop the motors once the target angle is reached
    stopMotors();
    Serial.println("Turned right 90 degrees.");
}

void setup() {
    // Initialize motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Initialize encoder pins
    pinMode(ENC_A1, INPUT);
    pinMode(ENC_B1, INPUT);
    pinMode(ENC_A2, INPUT);
    pinMode(ENC_B2, INPUT);
    
    // Initialize IR sensors
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);

    // Initialize Serial communication
    Serial.begin(115200);  // Ensure the baud rate matches Serial Monitor
    Wire.begin();  // Initialize I2C communication
    
    // Initialize Lidar and MPU
    if (!lidar.begin()) {
        Serial.println("Failed to initialize VL53L0X sensor.");
        while (1);
    }

    // Initialize MPU6050 using MPU6050_light
    mpu.begin();  // Initialize the MPU6050
    mpu.calcOffsets();  // Calibrate the sensor (gyro and accelerometer)

    // Attach encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderISR_A1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderISR_A2, CHANGE);

    // Initialize PID controller
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(80, 120);  // Control the output range to match motor speed
    pid.SetTunings(kp, ki, kd);

       // Initialize PID controller
    pid2.SetMode(AUTOMATIC);
    pid2.SetOutputLimits(80, 120);  // Control the output range to match motor speed
    pid2.SetTunings(KP, KI, KD);



 

}

void loop() {
    if (millis() - lastPrint2 > 500) {
        lastPrint2 = millis();
        // Get Lidar distance
        lidarDistance = lidar.readRange();  // Read the distance in millimeters
        Serial.print("Lidar Distance: ");
        Serial.print(lidarDistance / 10);  // Convert to centimeters
        Serial.println(" cm");
        Serial.print(output);

        // Get MPU angle (Z-axis rotation) from MPU6050_light
        Serial.print("Current Z-Angle: ");
        Serial.print(getAngle());
        Serial.println(" degrees");

        // Calculate distance traveled based on encoder count
        currentDistance = (encoderCountA + encoderCountB) * wheelCircumference / 2.0;
        Serial.print("Current Distance: ");
        Serial.print(currentDistance);
        Serial.println(" cm");
    }



// right hand roll 
 if(digitalRead(IR_RIGHT_PIN) == LOW && digitalRead(IR_LEFT_PIN) == LOW && lidar.readRange() < 130){ // turn 
        turnRight(130);
      delay(1500);
        turnRight(130);
        stopMotors(); 
         delay(1000);
        // moveForward(800);

    }
   else if ((digitalRead(IR_LEFT_PIN) == LOW && digitalRead(IR_RIGHT_PIN) == LOW)) { // move forword if there is wall right and left
        moveForward(cellTime);
        //moveStraight(getAngle(),120);
        stopMotors();
        delay(1000); 

        
    } else if (digitalRead(IR_LEFT_PIN) == LOW && digitalRead(IR_RIGHT_PIN) != LOW) { // move right if there is no wall on right
      //   moveForward(500);
      //  stopMotors(); 
      //   delay(500);
        turnRight(130);
        moveForward(cellTime);
             //   moveStraight(getAngle(),120);


    } else if (digitalRead(IR_RIGHT_PIN) == LOW && digitalRead(IR_LEFT_PIN) != LOW && lidar.readRange() < 120) { // move left if there is wall on right and forword
      //   moveForward(500);
      //  stopMotors(); 
      //   delay(500);
        turnLeft(130);
         moveForward(cellTime);
             //   moveStraight(getAngle(),120);


    } 
    else if (digitalRead(IR_RIGHT_PIN) != LOW && digitalRead(IR_LEFT_PIN) != LOW && lidar.readRange() < 120){
      turnRight(130);
        moveForward(cellTime);
       //        moveStraight(getAngle(),120);

    }
    
    else { 
     moveForward(cellTime);
     //        moveStraight(getAngle(),120);

    stopMotors(); 
    delay(4000);

          }

 }

// Encoder interrupt functions
void encoderISR_A1() {
    if (digitalRead(ENC_B1) == HIGH) {
        encoderCountA++;
    } else {
        encoderCountA--;
    }
}

void encoderISR_A2() {
    if (digitalRead(ENC_B2) == HIGH) {
        encoderCountB++;
    } else {
        encoderCountB--;
    }
}

// void encoderISR1() {
//   int stateA = digitalRead(ENC_A1);
//   int stateB = digitalRead(ENC_B1);

//   // If A == B => increment, else => decrement
//   // Each A edge triggers an increment or decrement
//   if (stateA == stateB) {
//     encoderCountA--;
//   } else {
//     encoderCountB++;
//   }
// }

// // ------------------------------
// // 2× Quadrature Decode: Motor B
// // Using only A interrupts, read B inside the ISR
// // ------------------------------
// void encoderISR2() {
//   int stateA = digitalRead(ENC_A2);
//   int stateB = digitalRead(ENC_B2);

//   if (stateA == stateB) {
//     encoderCountA++;
//   } else {
//     encoderCountB--;
//   }
// }

float getAngle() {
    mpu.update();
    return mpu.getAngleZ();  // Return the current Z-axis angle from MPU6050
}

