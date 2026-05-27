// Increase number of measurements per ultrasound cycle
// Calculate an average per integer angle
#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <ultrasound.h>

#include <WiFi.h>
#include <WebServer.h>


// PIN DEFINITIONS

// Stepper motors pin definition
const int STEPPER1_DIR_PIN  = 16; // changeme
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 26; 
const int STEPPER2_STEP_PIN = 27;
const int STEPPER_EN_PIN    = 15;

// Ultrasound sensor and servo pin definition
const int ULTRA_TRIG = 2;
const int ULTRA_ECHO = 4;
const int SERVO_PIN = 33;

//ADC pins definition
const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;


// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

const char* ssid     = "robet";
const char* password = "1234567890";

WebServer server(80);

ultrasound US (ULTRA_TRIG, ULTRA_ECHO, SERVO_PIN);

// --- WEB SERVER HANDLERS ---

// 1. Root Handler: Serves the beautiful dashboard page framework to your laptop
void handleRoot() {
  String html = R"rawhtml(
  <!DOCTYPE html>
  <html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Robot Telemetry</title>
    <style>
      body { font-family: 'Segoe UI', Arial, sans-serif; background: #121212; color: #e0e0e0; text-align: center; margin:0; padding:20px; }
      h1 { color: #00e676; margin-bottom: 30px; }
      .container { display: flex; flex-wrap: wrap; justify-content: center; gap: 20px; max-width: 900px; margin: 0 auto; }
      .card { background: #1e1e1e; border-radius: 12px; padding: 25px; width: 220px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); border: 1px solid #333; }
      .card h2 { font-size: 14px; text-transform: uppercase; letter-spacing: 1px; color: #888; margin: 0 0 15px 0; }
      .value { font-size: 36px; font-weight: bold; color: #fff; }
      .unit { font-size: 16px; color: #888; margin-left: 4px; }
      #card-angle { border-top: 4px solid #2196f3; }
      #card-distance { border-top: 4px solid #ff9800; }
      #card-voltage { border-top: 4px solid #00e676; }
    </style>
    <script>
      // Automatically request refreshed JSON payload every 200ms
      setInterval(function() {
        fetch('/data')
          .then(response => response.json())
          .then(data => {
            document.getElementById('angle').innerText = data.angle.toFixed(2);
            document.getElementById('distance').innerText = data.distance;
            document.getElementById('voltage').innerText = data.voltage.toFixed(2);
          })
          .catch(err => console.error(err));
      }, 200);
    </script>
  </head>
  <body>
    <h1>ESP32 Robot Dashboard</h1>
    <div class="container">
      <div class="card" id="card-angle"><h2>Robot Angle</h2><span class="value" id="angle">0.0</span><span class="unit">&deg;</span></div>
      <div class="card" id="card-distance"><h2>Object Distance</h2><span class="value" id="distance">0</span><span class="unit">cm</span></div>
      <div class="card" id="card-voltage"><h2>Battery Voltage</h2><span class="value" id="voltage">0.00</span><span class="unit">V</span></div>
    </div>
  </body>
  </html>
  )rawhtml";
  server.send(200, "text/html", html);
}

// 2. Data Endpoint Handler: Sends lightweight background JSON streams
void handleData() {
  String json = "{";
  json += "\"angle\":" + String(US.angle) + ",";
  json += "\"distance\":" + String(US.distance) + ",";
  //json += "\"voltage\":" + String(voltage); // ADD LATER
  json += "}";
  server.send(200, "application/json", json);
}

// PID GAINS
/*error in degrees -> output in rad/s
Tuning order:
1. Ki=0, Kd=0. Raise Kp until oscillation, back off 20%.
2. Raise Kd to damp oscillation (try Kp/10 as starting point).
3. Add tiny Ki (0.5-2.0) only if there is persistent drift.
*/
volatile float Kp = 10.0f;     // ~150 * (pi/180) scaled for deg error
volatile float Ki = 0.0f;
volatile float Kd = 0.05f;     // ~8 * (pi/180)


// BALANCE STPOINT
//  Your upright range was 142-158 mrad = 8.14-9.05 deg
//  Midpoint = 8.6 deg. Trim by +/-0.1 deg if it drifts.
volatile float balanceAngleDeg = 82.80f;


// COMPLEMENTARY FILTER
const float ALPHA = 0.99f;


// LIMITS
const float MAX_MOTOR_SPEED_RAD = 20.0f;
const float INTEGRAL_LIMIT_DEG  = 10.0f;  // deg
const float FALL_ANGLE_DEG      = 45.0f;  // deg from setpoint -> cut motors
const float MOTOR_ACCEL_RAD     = 500.0f; // rad/s^2 -- keep high for fast response

const float VREF = 4.096f;


// idk
const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float kx = 20.0;
float gyroYBias = 0.0f;

// Shared telemetry (written by loop, read by serial print)
volatile float g_angleDeg  = 0.0f;
volatile float g_errorDeg  = 0.0f;
volatile float g_motorSpd  = 0.0f;

long duration;
int distance;



//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );


//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
  return true;
}

uint16_t readADC(uint8_t channel) {
  uint8_t tx0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t tx1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 

  SPI.transfer(tx0);                    // Send Command Byte 0
  uint8_t rx0 = SPI.transfer(tx1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rx1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 

  uint16_t result = ((rx0 & 0x0F) << 8) | rx1; // Combine high and low byte into 12-bit result
  return result;
}

void setup()
{
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);


  // INIT SERVO
  US.attachServo();


  // INIT MPU6050 GYRO
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);


  // GYRO BIAS CALIBRATION - 3s ON STARTUP
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
  Serial.printf("Gyro bias: %.5f rad/s\n", gyroYBias);
  Serial.printf("Balance setpoint: %.2f deg\n", (float)balanceAngleDeg);


  // Connect to Wi-Fi Network
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected!");
  Serial.print("Local IP Address: http://");
  Serial.println(WiFi.localIP()); // <-- THIS IS THE URL TO VISIT ON YOUR LAPTOP

  // Set up Web Server Endpoints
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("HTTP Server started.");


  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");


  //SET AND UNLOCK MOTORS
  step1.setAccelerationRad(MOTOR_ACCEL_RAD);
  step2.setAccelerationRad(MOTOR_ACCEL_RAD);
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);


  //INIT ADC / SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

  //Serial.println("Ready. Balancing starting now.");
  //Serial.println("angle(deg) error(deg) motor(rad/s)");

}

void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  static float angle    = 0.0f;
  static bool  firstRun = true;
  static float integral  = 0.0f;
  static float prevError = 0.0f;
  static float motorSpeed = 0.0f;
  static float accelAngleDeg = 0.0f;
  static float gyroRateDeg   = 0.0f;




  static float tiltx = 0.0;             //current tilt angle
  static bool objectdetected = false;
  static int speed = 1;
  static int speedL = speed; static int speedR = speed;
  static int measPerAngle = 3;

  /*
  static int batteryVoltageADC = analogRead(BATTERY_VOLTAGE); // this should be an integer between 0 and 4095
  static float batteryPinVoltage = 3.30f / 4095 * batteryVoltageADC;
  static float actualVoltageDrop = batteryPinVoltage / 1100;
  Serial.print(batteryVoltageADC);
  Serial.print(".");
  Serial.print(batteryPinVoltage);
  Serial.print(".");
  Serial.print(actualVoltageDrop);
  */
 
  // What else do we want?
  // Convert the voltage to a current
  // Instantaneous power = I^2 R
  // Create a variable to sum the power consumption over multiple cycles to obtain the total power consumed


  

  US.servoSweep(30, 150, 3, measPerAngle);

  // SPEED CONTROLLED BY ULTRASOUND SENSOR
  if(US.objectDetected(distance)){
    speedL = speed; speedR = -speed;
  }
  else{
    speedL = speed; speedR = speed;
  }

  //step1.setTargetSpeedRad(speedR);
  //step2.setTargetSpeedRad(-speedL);


  // PID SPEED CONTROL

  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;

    const float dt = LOOP_INTERVAL / 1000.0f;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Angle in degrees via complementary filter
    // NOTE: Removed the word 'float' so it uses the global ones above
    accelAngleDeg = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
    gyroRateDeg   = -(g.gyro.y - gyroYBias)    * RAD_TO_DEG;

    if (firstRun) { angle = accelAngleDeg; firstRun = false; }

    angle = ALPHA * (angle + gyroRateDeg * dt) + (1.0f - ALPHA) * accelAngleDeg;

    // Read volatile gains/setpoint once per iteration
    float sp = balanceAngleDeg;
    float kp = Kp, ki = Ki, kd = Kd;

    float error = angle - sp;


    // FALL DETECTION
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


    // --- NEW: THE ANTI-JITTER OUTPUT FILTER ---
    // 0.8 = 80% old speed, 20% new speed. The higher the decimal (up to 0.99), 
    // the smoother the motor, but the slower the reaction time.
    static float smoothed_output = 0.0f;
    smoothed_output = (0.80f * smoothed_output) + (0.20f * output);

    // --- SPEED DEADBAND (KILL THE HUM) ---
    // If the requested speed is incredibly tiny, just lock the coils to stop limit cycling.
    if (abs(smoothed_output) < 0.5f) {
        smoothed_output = 0.0f;
    }

    step1.setTargetSpeedRad(-smoothed_output);
    step2.setTargetSpeedRad(smoothed_output);

    //step1.setTargetSpeedRad(-output);
    //step2.setTargetSpeedRad(output);

    motorSpeed = output;
    g_angleDeg = angle;
    g_errorDeg = error;
    g_motorSpd = motorSpeed;
  }


  //Print updates every PR  INT_INTERVAL ms
  //Line format: X-axis tilt, Motor speed, A0 Voltage, Ultrasound Distance (cm)
  if(millis() > printTimer){
    printTimer += PRINT_INTERVAL;
    
    // Read the current distance synchronously right before printing
    distance = US.calculateDistance();

    //Serial.print(tiltx*1000);
    //Serial.print(' ');
    //Serial.print(step1.getSpeedRad());
    //Serial.print(' ');
    //Serial.print((readADC(0) * VREF)/4095.0);
    //Serial.print(' ');
    //Serial.print(distance);
    //Serial.println();
  }
}
