#include <avr/sleep.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>

#define DEBUG_MODE 0

// Define the states of the system
enum State {
  IDLE,
  LOCKING,
  UNLOCKING,
};

State currentState = IDLE;

// Pin assignments
const int hallSensorPin = 2;
const int touchSensorPin = 3;

const int fingerprintSensorRXPin = 7;
const int fingerprintSensorTXPin = 8;
const int fingerprintSensorPowerOnPin = 5;
const int buzzerPin = A2;

const int servoPowerOnPin = 6;
const int servoCtrlPin = 4;
const int servoPotPin = A0;

const int toggleSwitchPin = 9;

const int voltageDividerReadPin = A1;

// Other variables for tracking state
volatile unsigned long lastWakeTime = 0;
volatile unsigned long lastFingerWakeTime = 0;
unsigned long lastUnlockTime = 0;
volatile bool fingerPrintSensorInitialized = false;
bool doorOpen = false;
bool autolock = true;
bool fingerprintIsOn = false;
int recordedLockReading = -1;

// Set up servo
Servo servo;

// Servo parameters
const int lockedPosition = 10;
const int unlockedPosition = 170;
const int straightPosition = 90;
unsigned long servoWaitTime = 1000;

// Fingerprint sensor
SoftwareSerial mySerial(fingerprintSensorTXPin, fingerprintSensorRXPin);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

void setup() {
    if (DEBUG_MODE) {
        Serial.begin(9600);
    }

    // Set up pins
    pinMode(hallSensorPin, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);

    // Setup fingerprint sensor
    finger.begin(57600);

    // Interrupts for waking from sleep on LOW
    attachInterrupt(digitalPinToInterrupt(touchSensorPin), fingerPressed, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensorPin), doorMoved, CHANGE);
}

void loop() {
    // if (DEBUG_MODE) {
    //     // Print all of the state infomation
    //     Serial.print("State: ");
    //     Serial.println(currentState);
    //     Serial.print("Door Open: ");
    //     Serial.println(doorOpen);
    //     Serial.print("Hall effect: ");
    //     Serial.println(digitalRead(hallSensorPin));
    //     Serial.print("Autolock: ");
    //     Serial.println(autolock);
    //     Serial.print("Fingerprint Sensor On: ");
    //     Serial.println(fingerprintIsOn);
    //     delay(500);
    // }

    switch (currentState) {
        case IDLE:
            handleIDLE();
            break;
        case LOCKING:
            handleLOCKING();
            break;
        case UNLOCKING:
            handleUNLOCKING();
            break;
    }
}

void activatePins() {
    pinMode(toggleSwitchPin, INPUT_PULLUP);
}

void deactivatePins() {
    pinMode(toggleSwitchPin, INPUT);
}

void fingerPressed() {
    lastWakeTime = millis();
    lastFingerWakeTime = millis();
    // Turn on the fingerprint sensor
    powerFingerprintSensor(true);
}

void doorMoved() {
    lastWakeTime = millis();
}

void handleIDLE() {
    static unsigned long lastVoltagePrintTime = 0;
    const unsigned long voltagePrintInterval = 1000; // 30 seconds

    // If the door just closed, lock it
    bool previousDoorOpen = doorOpen;
    doorOpen = isDoorOpen();
    if (previousDoorOpen && !doorOpen) {
        currentState = LOCKING;
        return;
    }

    if (fingerprintIsOn) {
        if (DEBUG_MODE) Serial.print("Scanner on! ");
        // If the fingerprint sensor has been on for over 10 seconds, turn it off
        if (millis() - lastFingerWakeTime > 10000) {
            powerFingerprintSensor(false);
        } else {
            if (!fingerPrintSensorInitialized) {
                fingerPrintSensorInitialized = finger.verifyPassword();
                if (DEBUG_MODE) {
                    Serial.print("FP Sensor init'd: ");
                    Serial.println(fingerPrintSensorInitialized);
                }
                if (fingerPrintSensorInitialized) tone(buzzerPin, 1000, 50);
            } else {
                // Check if a registered fingerprint has been detected
                if (DEBUG_MODE) Serial.print("Scanning: ");
                int result = getFingerprintIDez();
                if (DEBUG_MODE) Serial.println(result);

                if (result != -1) {
                    currentState = UNLOCKING;
                    return;
                }
            }
        }
    }

    // Check voltage
    if (millis() - lastVoltagePrintTime >= voltagePrintInterval) {
        float voltage = readVoltage();
        if (DEBUG_MODE) {
            Serial.print("Voltage: ");
            Serial.print(voltage);
            Serial.println(" V");
        }
        lastVoltagePrintTime = millis();
    }

    // Go to sleep if 30 seconds have passed since the last wake up
    if (millis() - lastWakeTime > 10000) {
        // Turn off the fingerprint sensor
        powerFingerprintSensor(false);
        deactivatePins();
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_bod_disable();
        sleep_mode();
        // Program continues here after waking up
        activatePins();
    }
}

void handleLOCKING() {
    powerServo(true);
    if (!autolock || isDoorOpen()) {
        currentState = IDLE;
        return;
    }

    // Lock the door
    servo.write(lockedPosition);
    bool locked = waitForLock();

    // Return to straight position
    servo.write(straightPosition);

    // Play a tone based on the lockSwitchPin status
    if (locked) {
        lockedTone(locked);
    } else {
        // Try one last time
        delay(500);  // allow return to straight position
        // Lock the door
        servo.write(lockedPosition);
        locked = waitForLock();
        // Return to straight position
        servo.write(straightPosition);
        lockedTone(locked);
    }
    delay(500);  // allow return to straight position

    powerServo(false);
    currentState = IDLE;
}

bool waitForLock() {
    unsigned long startTime = millis();
    while (millis() - startTime < servoWaitTime) {
        if (recordedLockReading != -1) {
            int currReading = analogRead(servoPotPin);
            int positionDiff = abs(recordedLockReading - currReading);
            if (positionDiff < 10) return true;
        }
    }

    if (recordedLockReading == -1) {
        // recordedLockReading = analogRead(servoPotPin);
        return true;
    }

    return false;
}

void lockedTone(bool success) {
    if (millis() - lastUnlockTime < 10000) return;
    if (success) {
        // Play two short beeps
        for (int i=0; i<2; i++) {
            tone(buzzerPin, 1000, 100);  // 1000 Hz for 300 ms
            delay(200);  // Wait for the tone to finish and a short pause
        }
    } else {
        // Play four long beeps
        for (int i=0; i<4; i++) {
            tone(buzzerPin, 1000, 400);  // 1000 Hz for 300 ms
            delay(500);  // Wait for the tone to finish and a short pause
        }
    }
}

void handleUNLOCKING() {
    // Turn off the fingerprint sensor
    powerFingerprintSensor(false);
    // Start unlocking the door
    powerServo(true);
    servo.write(unlockedPosition);
    // Play chime
    tone(buzzerPin, 523, 100);  // C5
    delay(150);
    tone(buzzerPin, 659, 100);  // E5
    delay(150);
    tone(buzzerPin, 784, 300);  // G5
    delay(350);
    // Return the servo to normal position
    servo.write(straightPosition);
    delay(500);
    powerServo(false);

    currentState = IDLE;
    lastUnlockTime = millis();
}

bool isDoorOpen() {
    if (digitalRead(hallSensorPin) == HIGH) {
        delay(50);
        return digitalRead(hallSensorPin) == HIGH;
    }
    return false;
}

void powerFingerprintSensor(bool power) {
    if (power) {
        pinMode(fingerprintSensorPowerOnPin, OUTPUT);
        digitalWrite(fingerprintSensorPowerOnPin, HIGH);
    }
    else {
        pinMode(fingerprintSensorPowerOnPin, INPUT);
    }
    fingerprintIsOn = power;
    fingerPrintSensorInitialized = false;
}

void powerServo(bool power) {
    if (power) {
        pinMode(servoPowerOnPin, OUTPUT);
        digitalWrite(servoPowerOnPin, LOW);
        servo.attach(servoCtrlPin);
    } else {
        pinMode(servoPowerOnPin, INPUT_PULLUP);
        servo.detach();
    }
}

int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) {
      if (DEBUG_MODE) Serial.print("No image ");
      return -1;
  }
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) {
      if (DEBUG_MODE) Serial.print("image2Tz failed ");
      return -1;
  }
  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) {
      if (DEBUG_MODE) Serial.print("No match ");
      return -1;
  }
  return finger.fingerID;
}

float readVoltage() {
    analogReference(DEFAULT);
    const float referenceVoltage = 5.0;  // Assuming a 5V Arduino board
    const int adcResolution = 1023;  // 10-bit ADC
    int rawValue = analogRead(voltageDividerReadPin);
    float voltage = (rawValue / (float)adcResolution) * referenceVoltage * 2.0;

    return voltage;
}
