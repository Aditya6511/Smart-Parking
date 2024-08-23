#include <Servo.h>
#include <LiquidCrystal.h>

// Define HC-SR04 ultrasonic sensor pins
const int doorSensorTrigPin = 2;   // Ultrasonic sensor for door control - Trig pin
const int doorSensorEchoPin = 3;   // Ultrasonic sensor for door control - Echo pin
const int slot1SensorTrigPin = 4;  // Ultrasonic sensor for parking slot 1 - Trig pin
const int slot1SensorEchoPin = 5;  // Ultrasonic sensor for parking slot 1 - Echo pin
const int slot2SensorTrigPin = 6;  // Ultrasonic sensor for parking slot 2 - Trig pin
const int slot2SensorEchoPin = 7;  // Ultrasonic sensor for parking slot 2 - Echo pin

// Define servo motor pin
const int servoPin = A1;

// Define distances for door and parking slots
const int doorOpenDistance = 5;     // Adjust this distance based on your setup
const int doorCloseDistance = 5;    // Adjust this distance based on your setup
const int slotOccupiedDistance = 5; // Adjust this distance based on your setup

// Define gate open and close delay
const int gateOpenCloseDelay = 5000; // 5 seconds

Servo doorServo;

// Define LCD parameters
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); // RS, E, D4, D5, D6, D7

bool slot1OccupiedPrev = false;
bool slot2OccupiedPrev = false;
unsigned long lastGateTime = 0; // Variable to store the last time the gate was opened or closed

// Function to get distance from HC-SR04 ultrasonic sensor
int getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;
    return distance;
}

// Function to open the door
void openDoor(bool slot1Occupied, bool slot2Occupied) {
    Serial.println("Opening Door");
    // Check if either slot is not occupied before opening the door
    if (!(slot1Occupied && slot2Occupied)) {
        doorServo.write(90); // Adjust the angle based on your setup
    }
}

// Function to close the door
void closeDoor() {
    Serial.println("Closing Door");
    doorServo.write(0); // Adjust the angle based on your setup
}

// Function to update LCD with parking status
void updateLCD(bool slot1Occupied, bool slot2Occupied) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Slot 1: ");
    lcd.print(slot1Occupied ? "Occupied" : "Empty");
    lcd.setCursor(0, 1);
    lcd.print("Slot 2: ");
    lcd.print(slot2Occupied ? "Occupied" : "Empty");
}

// Function to check if a car leaves a slot
bool carLeavesSlot(bool previousOccupied, int currentDistance, int occupiedDistance) {
    return previousOccupied && !(currentDistance < occupiedDistance);
}

// Update the slot status and open the gate if necessary
void updateSlotStatus(bool &slotOccupiedPrev, int currentDistance, int occupiedDistance) {
    bool slotOccupied = currentDistance < occupiedDistance;
    if (carLeavesSlot(slotOccupiedPrev, currentDistance, occupiedDistance)) {
        unsigned long currentTime = millis();
        if (currentTime - lastGateTime >= gateOpenCloseDelay) {
            // Open the gate
            openDoor(false, false);
            delay(gateOpenCloseDelay);
            // Close the gate
            closeDoor();
            lastGateTime = currentTime;
        }
    }
    // Update previous occupied status
    slotOccupiedPrev = slotOccupied;
}

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    
    // Attach servo to its pin
    doorServo.attach(servoPin);
    
    // Set sensor pins as inputs and outputs
    pinMode(doorSensorTrigPin, OUTPUT);
    pinMode(doorSensorEchoPin, INPUT);
    pinMode(slot1SensorTrigPin, OUTPUT);
    pinMode(slot1SensorEchoPin, INPUT);
    pinMode(slot2SensorTrigPin, OUTPUT);
    pinMode(slot2SensorEchoPin, INPUT);
    
    // Initialize LCD
    lcd.begin(16, 2);
}

void loop() {
    // Read distances from ultrasonic sensors
    int doorDistance = getDistance(doorSensorTrigPin, doorSensorEchoPin);
    int slot1Distance = getDistance(slot1SensorTrigPin, slot1SensorEchoPin);
    int slot2Distance = getDistance(slot2SensorTrigPin, slot2SensorEchoPin);

    // Control the door based on the door distance
    if (doorDistance < doorOpenDistance) {
        unsigned long currentTime = millis();
        if (currentTime - lastGateTime >= gateOpenCloseDelay) {
            // Open the gate
            openDoor(slot1OccupiedPrev, slot2OccupiedPrev);
            delay(gateOpenCloseDelay);
            // Close the gate
            closeDoor();
            lastGateTime = currentTime;
        }
    }

    // Update LCD with parking status
    updateLCD(slot1OccupiedPrev, slot2OccupiedPrev);

    // Update slot 1 status and open gate if necessary
    updateSlotStatus(slot1OccupiedPrev, slot1Distance, slotOccupiedDistance);

    // Update slot 2 status and open gate if necessary
    updateSlotStatus(slot2OccupiedPrev, slot2Distance, slotOccupiedDistance);

    // Add a delay to avoid rapid sensor readings
    delay(100);
}
