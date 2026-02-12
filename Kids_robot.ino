#include <Servo.h>

// Pin definitions
const int servoPin = 9;       // Servo for sweeping ultrasonic
const int trigPin = 10;       // Ultrasonic trigger
const int echoPin = 11;       // Ultrasonic echo

// L298N motor pins
const int leftMotorEN = 5;    // Enable left motor (PWM for speed)
const int leftMotorIN1 = 4;   // Left motor forward
const int leftMotorIN2 = 3;   // Left motor backward
const int rightMotorEN = 6;   // Enable right motor (PWM)
const int rightMotorIN3 = 8;  // Right motor forward
const int rightMotorIN4 = 7;  // Right motor backward

// Constants
const int obstacleDistance = 20;  // Avoid if closer than 20cm
const int sweepSteps = 5;         // Number of sweep positions (e.g., 0°, 45°, 90°, 135°, 180°)
const int motorSpeed = 200;       // PWM speed (0-255); adjust for your motors
const int burstTimeMin = 500;     // Min time for a movement burst (ms)
const int burstTimeMax = 2000;    // Max time for a movement burst (ms)
const int stopTimeMin = 500;      // Min stop time (ms)
const int stopTimeMax = 1500;     // Max stop time (ms)

Servo servo;
int positions[sweepSteps] = {0, 45, 90, 135, 180};  // Sweep angles

void setup() {
  // Initialize serial for debugging (optional)
  Serial.begin(9600);
  
  // Servo setup
  servo.attach(servoPin);
  servo.write(90);  // Start at center
  
  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Motor pins
  pinMode(leftMotorEN, OUTPUT);
  pinMode(leftMotorIN1, OUTPUT);
  pinMode(leftMotorIN2, OUTPUT);
  pinMode(rightMotorEN, OUTPUT);
  pinMode(rightMotorIN3, OUTPUT);
  pinMode(rightMotorIN4, OUTPUT);
  
  // Stop motors initially
  stopMotors();
  
  randomSeed(analogRead(0));  // Seed random for unpredictable behavior
}

void loop() {
  // Abrupt scurry: Move forward in a burst, then stop randomly
  forward(random(burstTimeMin, burstTimeMax));
  stopMotors();
  delay(random(stopTimeMin, stopTimeMax));
  
  // Occasionally add a random turn for cockroach erraticness (20% chance)
  if (random(100) < 20) {
    int turnDirection = random(2);  // 0 left, 1 right
    if (turnDirection == 0) {
      turnLeft(random(300, 800));  // Short turn burst
    } else {
      turnRight(random(300, 800));
    }
  }
  
  // Check for obstacles and avoid if needed
  if (detectObstacle()) {
    avoidObstacle();
  }
}

// Function to move forward for a duration
void forward(int duration) {
  digitalWrite(leftMotorIN1, HIGH);
  digitalWrite(leftMotorIN2, LOW);
  analogWrite(leftMotorEN, motorSpeed);
  
  digitalWrite(rightMotorIN3, HIGH);
  digitalWrite(rightMotorIN4, LOW);
  analogWrite(rightMotorEN, motorSpeed);
  
  delay(duration);
  stopMotors();
}

// Function to turn left for a duration
void turnLeft(int duration) {
  digitalWrite(leftMotorIN1, LOW);
  digitalWrite(leftMotorIN2, HIGH);
  analogWrite(leftMotorEN, motorSpeed);
  
  digitalWrite(rightMotorIN3, HIGH);
  digitalWrite(rightMotorIN4, LOW);
  analogWrite(rightMotorEN, motorSpeed);
  
  delay(duration);
  stopMotors();
}

// Function to turn right for a duration
void turnRight(int duration) {
  digitalWrite(leftMotorIN1, HIGH);
  digitalWrite(leftMotorIN2, LOW);
  analogWrite(leftMotorEN, motorSpeed);
  
  digitalWrite(rightMotorIN3, LOW);
  digitalWrite(rightMotorIN4, HIGH);
  analogWrite(rightMotorEN, motorSpeed);
  
  delay(duration);
  stopMotors();
}

// Function to stop both motors
void stopMotors() {
  analogWrite(leftMotorEN, 0);
  analogWrite(rightMotorEN, 0);
}

// Function to get distance from ultrasonic
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;  // Speed of sound formula
  return distance;
}

// Function to detect if there's an obstacle in front (at 90°)
bool detectObstacle() {
  servo.write(90);  // Face forward
  delay(200);       // Wait for servo to move
  long dist = getDistance();
  Serial.print("Front distance: ");
  Serial.println(dist);
  return (dist < obstacleDistance && dist > 0);  // Ignore 0 (no echo)
}

// Function to avoid obstacle: Sweep, find clearest path, turn abruptly
void avoidObstacle() {
  stopMotors();
  int clearestDirection = 90;  // Default forward
  long maxDist = 0;
  
  for (int i = 0; i < sweepSteps; i++) {
    servo.write(positions[i]);
    delay(200);  // Servo settle time
    long dist = getDistance();
    Serial.print("Distance at ");
    Serial.print(positions[i]);
    Serial.print(": ");
    Serial.println(dist);
    
    if (dist > maxDist) {
      maxDist = dist;
      clearestDirection = positions[i];
    }
  }
  
  // Turn abruptly toward clearest direction
  if (clearestDirection < 90) {
    turnLeft(500);  // Turn left abruptly
  } else if (clearestDirection > 90) {
    turnRight(500);  // Turn right abruptly
  }
  // Then resume forward burst
  forward(random(burstTimeMin, burstTimeMax));
}
