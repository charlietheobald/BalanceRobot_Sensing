#include <Arduino.h>
#include <Servo.h>

#include <WiFi.h>

// UDP socket

class ultrasound {

public:

int distance;
int angle;
int distanceAvg;

ultrasound(int trig, int echo, int servo) : trigPin(trig), echoPin(echo), servoPin(servo){
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void attachServo(){
    ultraServo.attach(servoPin);
}

int calculateDistance(){ 
  
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 7300); // Reads the echoPin, returns the sound wave travel time in microseconds. Currently set to around 125cm
  if(duration == 0){return 150;} // Improves timing by removing any measurements that don't receive a reflection in time
  distance = duration*0.034/2.00;
  return distance;
}

bool objectDetected(int distance){
    bool found;

    if(distance < 10){
        found = true;
    }
    else{
        found = false;
    }
    return found;
}

void servoSweep(int startAngle, int endAngle, int dly, int measPerAngle) {
    // 1. Declare historical variables HERE so both forward and backward 
    // loops share the continuous memory as the sensor moves back and forth.
    static float r1 = 150.0f;
    static float r2 = 150.0f;
    static float r3 = 150.0f;

    // --- FORWARD SWEEP ---
    for (int angle = startAngle * measPerAngle; angle <= endAngle * measPerAngle; angle++) {
        ultraServo.write(angle / measPerAngle);
        delay(dly / measPerAngle);

        // Capture raw data
        float calcDistance = calculateDistance();  
        
        // Push into shared history pipeline
        r3 = r2;
        r2 = r1;
        r1 = calcDistance;
    
        // Calculate the true float average
        distanceAvg = (r1 + r2 + r3) / measPerAngle; 

        // Send to Processing
        Serial.print(angle / measPerAngle); 
        Serial.print(","); 
        Serial.print(distanceAvg); 
        Serial.println();
    }

    // --- BACKWARD SWEEP ---
    for (int angle = endAngle * measPerAngle; angle >= startAngle * measPerAngle; angle--) {
        ultraServo.write(angle / measPerAngle);
        delay(dly / measPerAngle);

        // Capture raw data
        float calcDistance = calculateDistance();  
        
        // Push into the EXACT SAME history pipeline
        r3 = r2;
        r2 = r1;
        r1 = calcDistance;
    
        // Corrected math block (Fixed the circular definition & layout typo)
        int distanceAvg = (r1 + r2 + r3) / 3.0f; 

        // Send to Processing
        Serial.print(angle / measPerAngle); 
        Serial.print(","); 
        Serial.print(distanceAvg); 
        Serial.println();
    }
}



private:
int trigPin;
int echoPin;
int servoPin;
Servo ultraServo;
long duration;
int reflectionTimeout;
};

