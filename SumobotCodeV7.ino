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
#define BACK_IR_TRACKER1_PIN A4
#define BACK_IR_TRACKER2_PIN A5
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
double ultrasonicSensorDistance = 60.0;
double previousDistance;
bool ultrasonicSensorState = 0;

    // variables under interrupt function:
volatile unsigned long pulseInTimeBegin;
volatile unsigned long pulseInTimeEnd;
volatile bool newDistanceAvailable = false;
double objectDistance;


    // variables under IR Tracking Sensors:
unsigned long previousTimeIRPhase = 0;
bool triggerIRTracker = 0;
bool leftTrackTrigger = 0;
bool rightTrackTrigger = 0;
bool bothTrackTrigger = 0;
int IRPhaseDelay = 400;

    // variables under Searching Mode
int zigzagPath = 0;
int zigzagPhaseCounter = 0;
bool zigzagPhase = 0;
bool sensorDetectState = 0;
unsigned long previousTimeStartSearching = millis();
unsigned long previousTimeStopSearching = millis();
unsigned long previousTimeSearchingDelay = millis();
const int startSearchingTimeDelay = 1000;
const int searchingDelay = 750;


void setup() {

    Serial.begin(115200);
        // Sensor Setup
    pinMode(LEFT_IR_PROXIMITY_PIN, INPUT_PULLUP);
    pinMode(RIGHT_IR_PROXIMITY_PIN, INPUT_PULLUP);
    pinMode(LEFT_IR_TRACKING_PIN, INPUT_PULLUP); 
    pinMode(RIGHT_IR_TRACKING_PIN, INPUT_PULLUP);
    pinMode(BACK_IR_TRACKER1_PIN, INPUT_PULLUP);
    pinMode(BACK_IR_TRACKER2_PIN, INPUT_PULLUP);
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

    // delay for 5 seconds
    Serial.println("Delay for 5 seconds");
    delay(5000);

    // turn twice
    Serial.println("Turn Twice");
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(1500);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}


void loop() {

    unsigned long currentTimeUltrasonicTrigger = millis();

    // triggers the triggerPin of ultrasonic sensor every 60ms
    if (currentTimeUltrasonicTrigger - previousTimeUltrasonicTrigger > ultrasonicSensorDelay) {
        previousTimeUltrasonicTrigger += ultrasonicSensorDelay;
        ultrasonicSensorTrigger();
    }

    // drives the motor forward if an object is near ultrasonicSensorDistance or below
    if (newDistanceAvailable) {
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
        previousTimeIRPhase = millis();
        zigzagPath = 0;
        zigzagPhaseCounter = 2;
        triggerIRTracker = 1;
            analogWrite(ENA, 150);
            analogWrite(ENB, 150);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        while (triggerIRTracker) {
            unsigned long currentTimeTrackTriggers = millis();
            unsigned long currentTimeIRPhase = 0;

            if (digitalRead(LEFT_IR_TRACKING_PIN) == LOW && digitalRead(RIGHT_IR_TRACKING_PIN) == HIGH) {
                leftTrackTrigger = 1;
                rightTrackTrigger = 0;
                bothTrackTrigger = 0;
            }
            else if (leftTrackTrigger) {
                currentTimeIRPhase = millis();
                if (currentTimeIRPhase - previousTimeIRPhase < IRPhaseDelay) {
                    Serial.println("BACKWARD");
                    analogWrite(ENA, 130);
                    analogWrite(ENB, 130);
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, HIGH);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, HIGH);
                }
                else if (currentTimeIRPhase - previousTimeIRPhase > IRPhaseDelay && currentTimeIRPhase - previousTimeIRPhase < (IRPhaseDelay * 3 / 2)) {
                    Serial.println("BACKWARD_LEFT");
                    analogWrite(ENA, 255);
                    analogWrite(ENB, 255);
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, HIGH); 
                }
                else if (currentTimeIRPhase - previousTimeIRPhase > (IRPhaseDelay * 3 / 2)) {
                    Serial.println("BACKWARD_LEFTSTOP");
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, LOW);
                    previousTimeIRPhase = currentTimeIRPhase;
                    leftTrackTrigger = 0;
                    triggerIRTracker = 0;
                    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
                    return;
                }
                if (digitalRead(BACK_IR_TRACKER1_PIN) == LOW || digitalRead(BACK_IR_TRACKER2_PIN) == LOW) {
                    Serial.println("BACKWARD_SENSORSTOP");
                    analogWrite(ENA, 255);
                    analogWrite(ENB, 255);
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, HIGH);
                    digitalWrite(IN4, LOW);
                    delay(100);
                    previousTimeIRPhase = currentTimeIRPhase;
                    triggerIRTracker = 0;
                    leftTrackTrigger = 0;
                    rightTrackTrigger = 0;
                    bothTrackTrigger = 0;
                    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
                    return;
                }
            }
                
            else if (digitalRead(LEFT_IR_TRACKING_PIN) == HIGH && digitalRead(RIGHT_IR_TRACKING_PIN) == LOW) {
                leftTrackTrigger = 0;
                rightTrackTrigger = 1;
                bothTrackTrigger = 0;
            }
            else if (rightTrackTrigger) {
                currentTimeIRPhase = millis();
                if (currentTimeIRPhase - previousTimeIRPhase < IRPhaseDelay) {
                    Serial.println("BACKWARD");
                    analogWrite(ENA, 130);
                    analogWrite(ENB, 130);
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, HIGH);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, HIGH);
                }
                else if (currentTimeIRPhase - previousTimeIRPhase > IRPhaseDelay && currentTimeIRPhase - previousTimeIRPhase < (IRPhaseDelay * 3 / 2)) {
                    Serial.println("BACKWARD_RIGHT");
                    analogWrite(ENA, 255);
                    analogWrite(ENB, 255);
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, HIGH);
                    digitalWrite(IN3, HIGH);
                    digitalWrite(IN4, LOW);
                }
                else if (currentTimeIRPhase - previousTimeIRPhase > (IRPhaseDelay * 3 / 2)) {
                    Serial.println("BACKWARD_RIGHTSTOP");
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, LOW);
                    previousTimeIRPhase = currentTimeIRPhase;
                    rightTrackTrigger = 0;
                    triggerIRTracker = 0;
                    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
                    return;
                }
                if (digitalRead(BACK_IR_TRACKER1_PIN) == LOW || digitalRead(BACK_IR_TRACKER2_PIN) == LOW) {
                    Serial.println("BACKWARD_SENSORSTOP");
                    analogWrite(ENA, 255);
                    analogWrite(ENB, 255);
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, HIGH);
                    digitalWrite(IN4, LOW);
                    delay(100);
                    previousTimeIRPhase = currentTimeIRPhase;
                    triggerIRTracker = 0;
                    leftTrackTrigger = 0;
                    rightTrackTrigger = 0;
                    bothTrackTrigger = 0;
                    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
                    return;
                }
            }

            else if (digitalRead(RIGHT_IR_TRACKING_PIN) == LOW && digitalRead(RIGHT_IR_TRACKING_PIN) == LOW) {
                leftTrackTrigger = 0;
                rightTrackTrigger = 0;
                bothTrackTrigger = 1;
            }
            else if (bothTrackTrigger) {
                currentTimeIRPhase = millis();
                if (currentTimeIRPhase - previousTimeIRPhase < IRPhaseDelay) {
                    Serial.println("BACKWARD");
                    analogWrite(ENA, 130);
                    analogWrite(ENB, 130);
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, HIGH);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, HIGH);
                }
                
                else if (currentTimeIRPhase - previousTimeIRPhase > IRPhaseDelay && currentTimeIRPhase - previousTimeIRPhase < (IRPhaseDelay * 2)) {
                    Serial.println("BACKWARD_ROTATE");
                    analogWrite(ENA, 255);
                    analogWrite(ENB, 255);
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, HIGH);
                }

                else if (currentTimeIRPhase - previousTimeIRPhase > (IRPhaseDelay * 2)) {
                    Serial.println("BACKWARD_ROTATESTOP");
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, LOW);
                    previousTimeIRPhase = currentTimeIRPhase;
                    triggerIRTracker = 0;
                    bothTrackTrigger = 0;
                    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
                    return;
                }
                if (digitalRead(BACK_IR_TRACKER1_PIN) == LOW || digitalRead(BACK_IR_TRACKER2_PIN) == LOW) {
                    Serial.println("BACKWARD_SENSORSTOP");
                    analogWrite(ENA, 255);
                    analogWrite(ENB, 255);
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, LOW);
                    digitalWrite(IN3, HIGH);
                    digitalWrite(IN4, LOW);
                    delay(100);
                    previousTimeIRPhase = currentTimeIRPhase;
                    triggerIRTracker = 0;
                    leftTrackTrigger = 0;
                    rightTrackTrigger = 0;
                    bothTrackTrigger = 0;
                    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinInterrupt, CHANGE);
                    return;
                }
            }
        }
    }

    
       
    // default state
    else if (objectDistance > ultrasonicSensorDistance) {
        unsigned long currentTimeStartSearching = millis();
        unsigned long currentTimeSearchingDelay;
        

        if (digitalRead(LEFT_IR_PROXIMITY_PIN) == LOW || digitalRead(RIGHT_IR_PROXIMITY_PIN) == LOW) {
            sensorDetectState = 1;
            ultrasonicSensorState = 1;
        }

        else if (digitalRead(LEFT_IR_PROXIMITY_PIN) == HIGH && digitalRead(RIGHT_IR_PROXIMITY_PIN) == HIGH) {
            sensorDetectState = 0;
        }
        
        if (zigzagPhaseCounter >= 2) {
            zigzagPhase = !zigzagPhase;
            zigzagPhaseCounter = 0;
            zigzagPath = 0;
            return;
        }

        else if (zigzagPhase == 0) {
            if (sensorDetectState == 0) {
                if (zigzagPath == 0) {
                    // Serial.print("Left: ");
                    // Serial.println(currentTimeStartSearching - previousTimeStartSearching);
                    if (currentTimeStartSearching - previousTimeStartSearching < startSearchingTimeDelay) {
                        analogWrite(ENA, 90);
                        analogWrite(ENB, 130);
                        digitalWrite(IN1, LOW);
                        digitalWrite(IN2, HIGH);
                        digitalWrite(IN3, HIGH);
                        digitalWrite(IN4, LOW);
                        previousTimeSearchingDelay = currentTimeStartSearching;
                        Serial.println("Search Left (Phase 1)");
                    }
                    
                    else if (currentTimeStartSearching - previousTimeStartSearching > startSearchingTimeDelay) {
                        currentTimeSearchingDelay = millis();
                        if (currentTimeSearchingDelay - previousTimeSearchingDelay < searchingDelay) {
                            analogWrite(ENA, 130);
                            analogWrite(ENB, 90);
                            digitalWrite(IN1, HIGH);
                            digitalWrite(IN2, LOW);
                            digitalWrite(IN3, LOW);
                            digitalWrite(IN4, HIGH);
                            Serial.println("Reverse Left (Phase 1)");                        
                        } 
                        else if (currentTimeSearchingDelay - previousTimeSearchingDelay > searchingDelay && currentTimeSearchingDelay - previousTimeSearchingDelay < (searchingDelay * 3 / 2)) {
                            digitalWrite(IN1, LOW);
                            digitalWrite(IN2, LOW);
                            digitalWrite(IN3, LOW);
                            digitalWrite(IN4, LOW);
                            Serial.println("Wait");
                        }
                        else if (currentTimeSearchingDelay - previousTimeSearchingDelay > (searchingDelay * 3 / 2)) {
                            previousTimeSearchingDelay = currentTimeSearchingDelay;
                            previousTimeStartSearching = currentTimeStartSearching;
                            zigzagPhaseCounter++;
                            zigzagPath = 1;
                            return;
                        }
                    }
                }

                else if (zigzagPath == 1) {
                    // Serial.print("Right: ");
                    // Serial.println(currentTimeStartSearching - previousTimeStartSearching);
                    if (currentTimeStartSearching - previousTimeStartSearching < startSearchingTimeDelay) {
                        Serial.println("Search Right (Phase 1)");
                        analogWrite(ENA, 130);
                        analogWrite(ENB, 90);
                        digitalWrite(IN1, HIGH);
                        digitalWrite(IN2, LOW);
                        digitalWrite(IN3, LOW);
                        digitalWrite(IN4, HIGH);
                        previousTimeSearchingDelay = currentTimeStartSearching;
                    }

                    else if (currentTimeStartSearching - previousTimeStartSearching > startSearchingTimeDelay) {
                        currentTimeSearchingDelay = millis();
                        if (currentTimeSearchingDelay - previousTimeSearchingDelay < searchingDelay) {
                            analogWrite(ENA, 90);
                            analogWrite(ENB, 130);
                            digitalWrite(IN1, LOW);
                            digitalWrite(IN2, HIGH);
                            digitalWrite(IN3, HIGH);
                            digitalWrite(IN4, LOW);
                            Serial.println("Reverse Right (Phase 1)");
                        }
                        else if (currentTimeSearchingDelay - previousTimeSearchingDelay > searchingDelay && currentTimeSearchingDelay - previousTimeSearchingDelay < (searchingDelay * 3 / 2)) {
                            digitalWrite(IN1, LOW);
                            digitalWrite(IN2, LOW);
                            digitalWrite(IN3, LOW);
                            digitalWrite(IN4, LOW);
                            Serial.println("Wait");
                        }
                        else if (currentTimeSearchingDelay - previousTimeSearchingDelay > (searchingDelay * 3 / 2)) {
                            previousTimeSearchingDelay = currentTimeSearchingDelay;
                            previousTimeStartSearching = currentTimeStartSearching;
                            zigzagPhaseCounter++;
                            return;
                        }
                    }
                }
            }
        }

        else if (zigzagPhase == 1) {
            if (sensorDetectState == 0) {
                if (zigzagPath == 0) {
                    // Serial.print("Right: ");
                    // Serial.println(currentTimeStartSearching - previousTimeStartSearching);
                    if (currentTimeStartSearching - previousTimeStartSearching < startSearchingTimeDelay) {
                        Serial.println("Search Right (Phase 2)");
                        analogWrite(ENA, 130);
                        analogWrite(ENB, 90);
                        digitalWrite(IN1, HIGH);
                        digitalWrite(IN2, LOW);
                        digitalWrite(IN3, LOW);
                        digitalWrite(IN4, HIGH);
                        previousTimeSearchingDelay = currentTimeStartSearching;
                    }

                    else if (currentTimeStartSearching - previousTimeStartSearching > startSearchingTimeDelay) {
                        currentTimeSearchingDelay = millis();
                        if (currentTimeSearchingDelay - previousTimeSearchingDelay < searchingDelay) {
                            analogWrite(ENA, 90);
                            analogWrite(ENB, 130);
                            digitalWrite(IN1, LOW);
                            digitalWrite(IN2, HIGH);
                            digitalWrite(IN3, HIGH);
                            digitalWrite(IN4, LOW);
                            Serial.println("Reverse Right (Phase 2)");
                        }
                        else if (currentTimeSearchingDelay - previousTimeSearchingDelay > searchingDelay && currentTimeSearchingDelay - previousTimeSearchingDelay < (searchingDelay * 3 / 2)) {
                            digitalWrite(IN1, LOW);
                            digitalWrite(IN2, LOW);
                            digitalWrite(IN3, LOW);
                            digitalWrite(IN4, LOW);
                            Serial.println("Wait");
                        }
                        else if (currentTimeSearchingDelay - previousTimeSearchingDelay > (searchingDelay * 3 / 2)) {
                            previousTimeSearchingDelay = currentTimeSearchingDelay;
                            previousTimeStartSearching = currentTimeStartSearching;
                            zigzagPhaseCounter++;
                            zigzagPath = 1;
                            return;
                        }
                    }
                }

                else if (zigzagPath == 1) {
                    // Serial.print("Left: ");
                    // Serial.println(currentTimeStartSearching - previousTimeStartSearching);
                    if (currentTimeStartSearching - previousTimeStartSearching < startSearchingTimeDelay) {
                        analogWrite(ENA, 90);
                        analogWrite(ENB, 130);
                        digitalWrite(IN1, LOW);
                        digitalWrite(IN2, HIGH);
                        digitalWrite(IN3, HIGH);
                        digitalWrite(IN4, LOW);
                        previousTimeSearchingDelay = currentTimeStartSearching;
                        Serial.println("Search Left (Phase 2)");
                    }
                    
                    else if (currentTimeStartSearching - previousTimeStartSearching > startSearchingTimeDelay) {
                        currentTimeSearchingDelay = millis();
                        if (currentTimeSearchingDelay - previousTimeSearchingDelay < searchingDelay) {
                            analogWrite(ENA, 130);
                            analogWrite(ENB, 90);
                            digitalWrite(IN1, HIGH);
                            digitalWrite(IN2, LOW);
                            digitalWrite(IN3, LOW);
                            digitalWrite(IN4, HIGH);
                            Serial.println("Reverse Left (Phase 2)");                        
                        } 
                        else if (currentTimeSearchingDelay - previousTimeSearchingDelay > searchingDelay && currentTimeSearchingDelay - previousTimeSearchingDelay < (searchingDelay * 3 / 2)) {
                            digitalWrite(IN1, LOW);
                            digitalWrite(IN2, LOW);
                            digitalWrite(IN3, LOW);
                            digitalWrite(IN4, LOW);
                            Serial.println("Wait");
                        }
                        else if (currentTimeSearchingDelay - previousTimeSearchingDelay > (searchingDelay * 3 / 2)) {
                            previousTimeSearchingDelay = currentTimeSearchingDelay;
                            previousTimeStartSearching = currentTimeStartSearching;
                            zigzagPhaseCounter++;
                            return;
                        }
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