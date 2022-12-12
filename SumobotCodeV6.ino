                // PINOUTS:
    // Ultrasonic Sensor Pinout
#define ECHO_PIN 2
#define TRIGGER_PIN 3
    // IR Proximity Sensor Pinout
#define LEFT_IR_PROXIMITY_PIN 11
#define RIGHT_IR_PROXIMITY_PIN 12
    // IR Tracking Sensor Pinout
#define LEFT_IR_TRACKING_PIN A2
#define RIGHT_IR_TRACKING_PIN A3
#define BACK_IR_TRACKING_PIN A4 
    // DC Motors Pinout
#define IN1 6   // FORWARD (M1)
#define IN2 7   // BACKWARD (M1)
#define ENA 5   // SPEED (M1)
#define IN3 8   // FORWARD (M2)
#define IN4 9   // BACKWARD (M2)
#define ENB 10   // SPEED (M2)


                // SENSOR GLOBAL VARIABLES: (TIME FUNCTIONALITIES)
    // ULTRASONIC SENSOR
unsigned long previousTimeUltrasonicTrigger = millis();
int ultrasonicSensorDelay = 60;
double ultrasonicSensorDistance = 40.0;
double previousDistance;
bool ultrasonicSensorState = 0;

    // variables under interrupt function:
volatile unsigned long pulseInTimeBegin;
volatile unsigned long pulseInTimeEnd;
volatile bool newDistanceAvailable = false;
double objectDistance;


    // variables under IR Tracking Sensors:
unsigned long previousTimeTrackTriggers = millis();
bool triggerIRTracker = 0;
int IRTrackDelay = 2000;

    // variables under Searching Mode
int zigzagPath = 0;
bool sensorDetectState = 0;
unsigned long previousTimeStartSearching = millis();
unsigned long previousTimeStopSearching = millis();
unsigned long previousTimeSearchingDelay = millis();
const int startSearchingTimeDelay = 750;
const int searchingDelay = 750;


void setup() {

    Serial.begin(9600);
        // Sensor Setup
    pinMode(LEFT_IR_PROXIMITY_PIN, INPUT);
    pinMode(RIGHT_IR_PROXIMITY_PIN, INPUT);
    pinMode(LEFT_IR_TRACKING_PIN, INPUT); 
    pinMode(RIGHT_IR_TRACKING_PIN, INPUT);
    pinMode(BACK_IR_TRACKING_PIN, INPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
        // Motor Setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
        // Turn off motors - Initial State
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}


void loop() {

    unsigned long currentTimeUltrasonicTrigger = millis();

    // triggers the triggerPin of ultrasonic sensor every 60ms
    if (currentTimeUltrasonicTrigger - previousTimeUltrasonicTrigger > ultrasonicSensorDelay) {
        previousTimeUltrasonicTrigger += ultrasonicSensorDelay;
        ultrasonicSensorTrigger();
    }

    // drives the motor forward if an object is near ultrasonicSensorDistance or below
    else if (newDistanceAvailable) {
        objectDistance = getUltrasonicDistance();
        if (objectDistance <= ultrasonicSensorDistance && objectDistance >= 2.0) {
            ultrasonicSensorState = 1;
        }

        if (ultrasonicSensorState) {

                        // reads the state of IR Proximity Sensors
            if (digitalRead(LEFT_IR_PROXIMITY_PIN) == LOW && digitalRead(RIGHT_IR_PROXIMITY_PIN) == HIGH) {     // Turns Left
                Serial.println("Turn Left");
                analogWrite(ENA, 255);
                analogWrite(ENB, 255);
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, HIGH);
                digitalWrite(IN3, HIGH);
                digitalWrite(IN4, LOW);
                ultrasonicSensorState = 0;
                return;
            }
            else if (digitalRead(LEFT_IR_PROXIMITY_PIN) == HIGH && digitalRead(RIGHT_IR_PROXIMITY_PIN) == LOW) {    // Turns Right
                Serial.println("Turn Right");
                analogWrite(ENA, 255);
                analogWrite(ENB, 255);
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, LOW);
                digitalWrite(IN4, HIGH);
                ultrasonicSensorState = 0;
                return;
            }
            else if (digitalRead(LEFT_IR_PROXIMITY_PIN) == LOW && digitalRead(RIGHT_IR_PROXIMITY_PIN) == LOW) {    // Forward
                Serial.println("Forward");
                analogWrite(ENA, 255);
                analogWrite(ENB, 255);
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, HIGH);
                digitalWrite(IN4, LOW);
                ultrasonicSensorState = 0;
                return;
            }
            else if (digitalRead(LEFT_IR_PROXIMITY_PIN) == HIGH && digitalRead(RIGHT_IR_PROXIMITY_PIN) == HIGH) {      // Default
                Serial.println("Default");
                analogWrite(ENA, 255);
                analogWrite(ENB, 255);
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, HIGH);
                digitalWrite(IN4, LOW);
                ultrasonicSensorState = 0;
                return;
            }
        }
            
    }

    // reads the state of line tracking sensors
    if (digitalRead(LEFT_IR_TRACKING_PIN) == LOW || digitalRead(RIGHT_IR_TRACKING_PIN) == LOW) {
        Serial.println("TRACK_TRIGGERED");
        detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
        triggerIRTracker = 1;
        while (triggerIRTracker) {
        unsigned long currentTimeTrackTriggers = millis();

        if (digitalRead(LEFT_IR_TRACKING_PIN) == LOW) {
            analogWrite(ENA, 200);
            analogWrite(ENB, 255);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            previousTimeTrackTriggers = currentTimeTrackTriggers;
            Serial.println("BACKWARD_LEFT");
            
        }
            
        else if (digitalRead(RIGHT_IR_TRACKING_PIN) == LOW) {
            analogWrite(ENA, 255);
            analogWrite(ENB, 200);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            previousTimeTrackTriggers = currentTimeTrackTriggers;
            Serial.println("BACKWARD_RIGHT");
            
        }

        else if (digitalRead(RIGHT_IR_TRACKING_PIN) == LOW && digitalRead(RIGHT_IR_TRACKING_PIN) == LOW) {
            analogWrite(ENA, 255);
            analogWrite(ENB, 255);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            previousTimeTrackTriggers = currentTimeTrackTriggers;
            Serial.println("BACKWARD");
        }

        else if (currentTimeTrackTriggers - previousTimeTrackTriggers > IRTrackDelay) {
            Serial.println("BACKWARD_DELAYSTOP");
            previousTimeTrackTriggers = currentTimeTrackTriggers;
            triggerIRTracker = 0;
            attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
            return;
        }

        if (digitalRead(BACK_IR_TRACKING_PIN) == LOW) {
            Serial.println("BACKWARD_SENSORSTOP");
            previousTimeTrackTriggers = currentTimeTrackTriggers;
            triggerIRTracker = 0;
            attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
            return;
        }
    }
    }

    
       
    // default state
    else if (objectDistance > ultrasonicSensorDistance) {
        unsigned long currentTimeStartSearching = millis();
        unsigned long currentTimeStopSearching = 0;
        unsigned long currentTimeSearchingDelay = 0;
        

        if (digitalRead(LEFT_IR_PROXIMITY_PIN) == LOW || digitalRead(RIGHT_IR_PROXIMITY_PIN) == LOW) {
            sensorDetectState = 1;
            ultrasonicSensorState = 1;
        }

        else if (digitalRead(LEFT_IR_PROXIMITY_PIN) == HIGH && digitalRead(RIGHT_IR_PROXIMITY_PIN) == HIGH) {
            sensorDetectState = 0;
        }
        

        if (sensorDetectState == 0) {
            if (zigzagPath == 0) {
                
                // Serial.print("Left: ");
                // Serial.println(currentTimeStartSearching - previousTimeStartSearching);
                if (currentTimeStartSearching - previousTimeStartSearching < startSearchingTimeDelay) {
                    analogWrite(ENA, 0);
                    analogWrite(ENB, 255);
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, HIGH);
                    digitalWrite(IN4, LOW);
                    previousTimeSearchingDelay = currentTimeStartSearching;
                    Serial.println("Search Left");
                }
                
                else if (currentTimeStartSearching - previousTimeStartSearching > startSearchingTimeDelay) {

                    currentTimeSearchingDelay = millis();
                    if (currentTimeSearchingDelay - previousTimeSearchingDelay < searchingDelay) {
                        analogWrite(ENA, 255);
                        analogWrite(ENB, 255);
                        digitalWrite(IN1, HIGH);
                        digitalWrite(IN2, LOW);
                        digitalWrite(IN3, LOW);
                        digitalWrite(IN4, HIGH);
                        Serial.println("Reverse Left");                        
                    }
                    
                    else if (currentTimeSearchingDelay - previousTimeSearchingDelay > searchingDelay && currentTimeSearchingDelay - previousTimeSearchingDelay < (2 * searchingDelay)) {
                        digitalWrite(IN1, LOW);
                        digitalWrite(IN2, LOW);
                        digitalWrite(IN3, LOW);
                        digitalWrite(IN4, LOW);
                        Serial.println("Wait");
                    }

                    else if (currentTimeSearchingDelay - previousTimeSearchingDelay > (2 * searchingDelay)) {
                        previousTimeSearchingDelay = currentTimeSearchingDelay;
                        previousTimeStartSearching = currentTimeStartSearching;
                        zigzagPath = 1;
                        return;
                    }
                }
            }

            else if (zigzagPath == 1) {
            
                // Serial.print("Right: ");
                // Serial.println(currentTimeStartSearching - previousTimeStartSearching);
                if (currentTimeStartSearching - previousTimeStartSearching < startSearchingTimeDelay) {
                    Serial.println("Search Right");
                    analogWrite(ENA, 255);
                    analogWrite(ENB, 0);
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, LOW);
                    previousTimeSearchingDelay = currentTimeStartSearching;
                }

                else if (currentTimeStartSearching - previousTimeStartSearching > startSearchingTimeDelay) {

                    currentTimeSearchingDelay = millis();
                    if (currentTimeSearchingDelay - previousTimeSearchingDelay < searchingDelay) {
                        analogWrite(ENA, 255);
                        analogWrite(ENB, 255);
                        digitalWrite(IN1, LOW);
                        digitalWrite(IN2, HIGH);
                        digitalWrite(IN3, HIGH);
                        digitalWrite(IN4, LOW);
                        Serial.println("Reverse Right");
                    }

                    else if (currentTimeSearchingDelay - previousTimeSearchingDelay > searchingDelay && currentTimeSearchingDelay - previousTimeSearchingDelay < (2 * searchingDelay)) {
                        digitalWrite(IN1, LOW);
                        digitalWrite(IN2, LOW);
                        digitalWrite(IN3, LOW);
                        digitalWrite(IN4, LOW);
                        Serial.println("Wait");
                    }

                    else if (currentTimeSearchingDelay - previousTimeSearchingDelay > (2 * searchingDelay)) {
                        previousTimeSearchingDelay = currentTimeSearchingDelay;
                        previousTimeStartSearching = currentTimeStartSearching;
                        zigzagPath = 0;
                        return;
                    }
                }
            }
        }
    }
}


void echoPinInterrupt() { // measures the duration of the signal
  if (digitalRead(ECHO_PIN) == HIGH) { //  start counting the time
    pulseInTimeBegin = micros();
  }
  else {
    pulseInTimeEnd = micros();  // end counting the time
    newDistanceAvailable = true; // activates the getUltrasonicDistance() function
  }
}


void ultrasonicSensorTrigger() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
}


double getUltrasonicDistance() {  
  double durationMicros = pulseInTimeEnd - pulseInTimeBegin;
  double distance = durationMicros / 2 * 0.0343;
  if (distance > 400) {
    return previousDistance;
  }
  distance = previousDistance * 0.6 + distance * 0.4;
  previousDistance = distance;
  return distance;
}
