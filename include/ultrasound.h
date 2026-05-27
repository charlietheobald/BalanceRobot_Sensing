#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <Arduino.h>
#include <ESP32Servo.h>

extern int radarMap[181];
extern int currentRadarAngle;

class ultrasound {

public:
    ultrasound(int trig, int echo, int servo) : trigPin(trig), echoPin(echo), servoPin(servo){
        pinMode(trigPin, OUTPUT); 
        pinMode(echoPin, INPUT); 
    }

    int distance;
    int angle;

    void attachServo(){
        ultraServo.setPeriodHertz(50);
        ultraServo.attach(servoPin, 500, 2400);
    }

    int calculateDistance(){  
        digitalWrite(trigPin, LOW); 
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH); 
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH, 7300); 
        if(duration == 0){return 150;} 
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
        static float r1 = 150.0f;
        static float r2 = 150.0f;
        static float r3 = 150.0f;

        for (int angle = startAngle * measPerAngle; angle <= endAngle * measPerAngle; angle++) {
            int calcAngle = angle / measPerAngle;
            currentRadarAngle = calcAngle;
            
            ultraServo.write(calcAngle);
            delay(dly / measPerAngle);

            float calcDistance = calculateDistance();  
            
            r3 = r2;
            r2 = r1;
            r1 = calcDistance;
        
            int distanceAvg = (r1 + r2 + r3) / measPerAngle; 

            if(calcAngle >= 0 && calcAngle <= 180) {
                radarMap[calcAngle] = distanceAvg;
            }

            Serial.print(calcAngle); 
            Serial.print(","); 
            Serial.print(distanceAvg); 
            Serial.print("."); 
        }

        for (int angle = endAngle * measPerAngle; angle >= startAngle * measPerAngle; angle--) {
            int calcAngle = angle / measPerAngle;
            currentRadarAngle = calcAngle;

            ultraServo.write(calcAngle);
            delay(dly / measPerAngle);

            float calcDistance = calculateDistance();  
            
            r3 = r2;
            r2 = r1;
            r1 = calcDistance;
        
            int distanceAvg = (r1 + r2 + r3) / 3.0f; 

            if(calcAngle >= 0 && calcAngle <= 180) {
                radarMap[calcAngle] = distanceAvg;
            }

            Serial.print(calcAngle); 
            Serial.print(","); 
            Serial.print(distanceAvg);
            Serial.print("."); 
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

#endif
