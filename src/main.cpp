#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <ultrasound.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 26; 
const int STEPPER2_STEP_PIN = 27;
const int STEPPER_EN_PIN    = 15;

const int ULTRA_TRIG = 2;
const int ULTRA_ECHO = 4;
const int SERVO_PIN = 33;

const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

const int TOGGLE_PIN        = 32;

const char* ssid     = "robet";
const char* password = "1234567890";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

ultrasound US (ULTRA_TRIG, ULTRA_ECHO, SERVO_PIN);

volatile float g_angleDeg  = 0.0f;
volatile float g_errorDeg  = 0.0f;
volatile float g_motorSpd  = 0.0f;
int distance = 0;
float batteryVoltage = 0.0f;

const float VREF = 4.096f;

int radarMap[181];
int currentRadarAngle = 30;

volatile float Kp = 10.0f;     
volatile float Ki = 0.0f;
volatile float Kd = 0.05f;     

volatile float balanceAngleDeg = 82.80f;

const float ALPHA = 0.99f;

const float MAX_MOTOR_SPEED_RAD = 20.0f;
const float INTEGRAL_LIMIT_DEG  = 10.0f;  
const float FALL_ANGLE_DEG      = 45.0f;  
const float MOTOR_ACCEL_RAD     = 500.0f; 

const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float kx = 20.0;
float gyroYBias = 0.0f;
long duration;

ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
}

uint16_t readADC(uint8_t channel) {
  uint8_t tx0 = 0x06 | (channel >> 2);
  uint8_t tx1 = (channel & 0x03) << 6;
  digitalWrite(ADC_CS_PIN, LOW); 
  SPI.transfer(tx0);                    
  uint8_t rx0 = SPI.transfer(tx1);      
  uint8_t rx1 = SPI.transfer(0x00);     
  digitalWrite(ADC_CS_PIN, HIGH); 
  uint16_t result = ((rx0 & 0x0F) << 8) | rx1;
  return result;
}

bool TimerHandler(void * timerNo)
{
  static bool toggle = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
  return true;
}

void setup()
{
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);

  for(int i=0; i<=180; i++) {
    radarMap[i] = 150;
  }

  US.attachServo();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("Gyro calibration -- hold still for 3 s...");
  const int CAL_SAMPLES = 300;
  float gyroSum = 0.0f;
  for (int i = 0; i < CAL_SAMPLES; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      gyroSum += g.gyro.y;
      delay(10);
  }
  gyroYBias = gyroSum / CAL_SAMPLES;

  Serial.print("Connecting to Wi-Fi: ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected!");
  Serial.println(WiFi.localIP()); 

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.begin();
  Serial.println("Asynchronous WebSocket Telemetry Streamer started.");

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }

  step1.setAccelerationRad(MOTOR_ACCEL_RAD);
  step2.setAccelerationRad(MOTOR_ACCEL_RAD);
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);
}

void loop()
{
  static unsigned long loopTimer = 0;   
  static unsigned long servoTimer = 0;
  static unsigned long txTimer = 0;
  static unsigned long batteryTimer = 0;     
  static float angle    = 0.0f;
  static bool  firstRun = true;
  static float integral  = 0.0f;
  static float prevError = 0.0f;
  static float motorSpeed = 0.0f;
  static float accelAngleDeg = 0.0f;
  static float gyroRateDeg   = 0.0f;

  static int speed = 1;
  static int speedL = speed; static int speedR = speed;
  static int measPerAngle = 1;

  static int sweepDir = 1;

  ws.cleanupClients();

  if (millis() - servoTimer >= 100) {
    servoTimer = millis();

    currentRadarAngle += (sweepDir * 3);
    if (currentRadarAngle >= 150) {
      currentRadarAngle = 150;
      sweepDir = -1;
    } else if (currentRadarAngle <= 30) {
      currentRadarAngle = 30;
      sweepDir = 1;
    }

    US.servoSweep(currentRadarAngle, currentRadarAngle, 5, measPerAngle);
    distance = US.calculateDistance();
    
    if (currentRadarAngle >= 0 && currentRadarAngle <= 180) {
      radarMap[currentRadarAngle] = distance;
    }
  }

  if (millis() - txTimer >= 35) {
    txTimer = millis();
    
    if (ws.count() > 0) {
      String json = "{\"distance\":" + String(distance) + 
                    ",\"voltage\":" + String(batteryVoltage) + 
                    ",\"sweepAngle\":" + String(currentRadarAngle) + "}";
      ws.textAll(json);
    }
  }

  if (millis() - batteryTimer >= 1000) {
    batteryTimer = millis();
    batteryVoltage = (readADC(0) * VREF) / 4095.0f;
  }

  if (US.objectDetected(distance)){
    speedL = speed; speedR = -speed;
  }
  else{
    speedL = speed; speedR = speed;
  }

  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    const float dt = LOOP_INTERVAL / 1000.0f;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accelAngleDeg = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
    gyroRateDeg   = -(g.gyro.y - gyroYBias)    * RAD_TO_DEG;

    if (firstRun) { angle = accelAngleDeg; firstRun = false; }

    angle = ALPHA * (angle + gyroRateDeg * dt) + (1.0f - ALPHA) * accelAngleDeg;

    float sp = balanceAngleDeg;
    float kp = Kp, ki = Ki, kd = Kd;

    float error = angle - sp;

    if (fabsf(error) > FALL_ANGLE_DEG) {
        step1.setTargetSpeedRad(0.0f);
        step2.setTargetSpeedRad(0.0f);
        integral  = 0.0f;
        prevError = error;
        g_angleDeg = angle; g_errorDeg = error; g_motorSpd = 0.0f;
        return;
    }

    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_LIMIT_DEG, INTEGRAL_LIMIT_DEG);

    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = kp * error + ki * integral + kd * derivative;
    output = constrain(output, -MAX_MOTOR_SPEED_RAD, MAX_MOTOR_SPEED_RAD);

    static float smoothed_output = 0.0f;
    smoothed_output = (0.80f * smoothed_output) + (0.20f * output);

    if (abs(smoothed_output) < 0.5f) {
        smoothed_output = 0.0f;
    }

    step1.setTargetSpeedRad(-smoothed_output);
    step2.setTargetSpeedRad(smoothed_output);

    motorSpeed = output;
    g_angleDeg = angle;
    g_errorDeg = error;
    g_motorSpd = motorSpeed;
  }
}