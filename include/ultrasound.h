#include <Arduino.h>
#include <Servo.h>

class ultrasound {

public:
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
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds

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

void servoSweep(int startAngle, int endAngle, int dly){
    for(int angle = startAngle; angle<=endAngle; angle++){  
    ultraServo.write(angle);
    delay(dly);
    
    distance = calculateDistance();// Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    Serial.print(angle); // Sends the current degree into the Serial Port
    Serial.print(","); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
    Serial.print(distance); // Sends the distance value into the Serial Port
    Serial.print(".");
  }

  for(int angle = endAngle; angle >= startAngle; angle--){  
    ultraServo.write(angle);
    delay(dly);

    distance = calculateDistance();// Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    Serial.print(angle); // Sends the current degree into the Serial Port
    Serial.print(","); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
    Serial.print(distance); // Sends the distance value into the Serial Port
    Serial.print(".");
  }
}



private:
int trigPin;
int echoPin;
int servoPin;
Servo ultraServo;
long duration;
int distance;
};

