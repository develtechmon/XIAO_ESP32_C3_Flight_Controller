#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ===== PPM Definitions =====
#define PPM_PIN 10             // Pin where the PPM signal is connected
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Pulse width (µs) above which a sync pulse is assumed
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// ===== Flight Controller / MPC Declarations =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float AccXCalibration, AccYCalibration, AccZCalibration;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;

int ESCfreq = 500;
int channelValues[NUM_CHANNELS];

// (All PID gain variables are removed – we use MPC instead)

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

// Kalman filter variables for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4; // (2*2)
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float DesiredRateYaw;  // For yaw, we'll use a simple control

// Servo objects to control motors
Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
const int mot1_pin = 2;
const int mot2_pin = 3;
const int mot3_pin = 4;
const int mot4_pin = 21;

// Time step (seconds)
const float t = 0.004; 

// ===== PPM Interrupt Handler =====
// Use IRAM_ATTR for fast interrupt handling on the ESP32.
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    if (pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if (pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
    ReceiverValue[channelIndex] = pulseWidth;
    channelIndex++;
  }
}

// ===== Helper Function to Safely Copy Receiver Values =====
void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ===== Simple 1D Kalman Filter =====
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState = KalmanState + (t * KalmanInput);
    KalmanUncertainty = KalmanUncertainty + (t * t * 16); // 4^2 = 16 (variance of IMU)
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9); // error variance (3^2 = 9)
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState; 
    Kalman1DOutput[1] = KalmanUncertainty;
}

// ===== MPC Control Function =====
// A simple brute-force MPC that searches over a set of candidate control inputs.
// The model: state x = [angle; rate]
// x[k+1] = A*x[k] + B*u, where A = [1, t; 0, 1] and B = [0; t]
// The cost over a prediction horizon (N steps) is the sum of squared angle errors plus a control effort penalty.
float mpc_control(float desiredAngle, float measuredAngle, float measuredRate) {
    int N = 10;         // Prediction horizon
    float lambda = 0.1; // Weight on control effort
    float maxU = 10.0;  // Maximum candidate control input (tunable)
    float bestCost = 1e9;
    float bestU = 0.0;
    
    // Try candidate control inputs from -maxU to maxU (step size: 0.1)
    for (float u = -maxU; u <= maxU; u += 0.1) {
         float cost = 0;
         float angle = measuredAngle;
         float rate = measuredRate;
         // Simulate evolution over the prediction horizon with constant control u
         for (int i = 0; i < N; i++) {
             float newAngle = angle + t * rate;
             float newRate = rate + t * u;
             angle = newAngle;
             rate = newRate;
             float error = angle - desiredAngle;
             cost += error * error + lambda * u * u;
         }
         if (cost < bestCost) {
             bestCost = cost;
             bestU = u;
         }
    }
    return bestU;
}

void setup() {
  Serial.begin(115200);

  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  // ----- Setup MPU6050 (I2C) -----
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ----- Setup ESP32 PWM Timers for Motor ESCs -----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ----- Attach Motors -----
  delay(1000);
  mot1.attach(mot1_pin, 1000, 2000);
  delay(1000);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);
  mot2.attach(mot2_pin, 1000, 2000);
  delay(1000);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);
  mot3.attach(mot3_pin, 1000, 2000);
  delay(1000);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);
  mot4.attach(mot4_pin, 1000, 2000);
  delay(1000);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);

  // Initialize motors to low throttle
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);

  // ----- Calibration Values -----
  RateCalibrationRoll = 0.57;
  RateCalibrationPitch = -2.34;
  RateCalibrationYaw = -0.40;
  AccXCalibration = 0.03;
  AccYCalibration = -0.01;
  AccZCalibration = 0.20;

  LoopTimer = micros();
}

void loop() {
  read_receiver(channelValues);

  // ----- Read MPU6050 Data -----
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  // Apply calibration offsets
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;
  AccX      -= AccXCalibration;
  AccY      -= AccYCalibration;
  AccZ      -= AccZCalibration;

  // ----- Calculate Angles from Accelerometer -----
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Clamp angles to ±20 degrees
  KalmanAngleRoll  = (KalmanAngleRoll  > 20) ? 20 : ((KalmanAngleRoll  < -20) ? -20 : KalmanAngleRoll);
  KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);
  
  // ----- Determine Desired Angles from RC input -----
  DesiredAngleRoll  = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle = ReceiverValue[2];
  // For Yaw we use a simple proportional mapping (or you could implement a similar MPC routine)
  float desiredYawRate = 0.15 * (ReceiverValue[3] - 1500);
  
  // ----- MPC Controller for Roll and Pitch -----
  // Compute control input (acceleration command) that drives the angle toward its desired value.
  float controlRoll  = mpc_control(DesiredAngleRoll,  KalmanAngleRoll,  RateRoll);
  float controlPitch = mpc_control(DesiredAnglePitch, KalmanAnglePitch, RatePitch);
  
  // In this example, we interpret the computed control input as the desired rate adjustment.
  float InputRoll  = controlRoll;
  float InputPitch = controlPitch;
  // For Yaw, we use the directly mapped desired rate.
  float InputYaw = desiredYawRate;

  // ----- Motor Mixing -----
  if (InputThrottle > 1800) { InputThrottle = 1800; }
  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw; // front right (counter clockwise)
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw; // rear right (clockwise)
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw; // rear left (counter clockwise)
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw; // front left (clockwise)

  // Saturate motor outputs
  MotorInput1 = (MotorInput1 > 2000) ? 1999 : MotorInput1;
  MotorInput2 = (MotorInput2 > 2000) ? 1999 : MotorInput2;
  MotorInput3 = (MotorInput3 > 2000) ? 1999 : MotorInput3;
  MotorInput4 = (MotorInput4 > 2000) ? 1999 : MotorInput4;
  MotorInput1 = (MotorInput1 < ThrottleIdle) ? ThrottleIdle : MotorInput1;
  MotorInput2 = (MotorInput2 < ThrottleIdle) ? ThrottleIdle : MotorInput2;
  MotorInput3 = (MotorInput3 < ThrottleIdle) ? ThrottleIdle : MotorInput3;
  MotorInput4 = (MotorInput4 < ThrottleIdle) ? ThrottleIdle : MotorInput4;

  if (ReceiverValue[2] < 1030) { // disarm if throttle too low
      MotorInput1 = ThrottleCutOff;
      MotorInput2 = ThrottleCutOff;
      MotorInput3 = ThrottleCutOff;
      MotorInput4 = ThrottleCutOff;
  }

  // ----- Output to Motors -----
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  while (micros() - LoopTimer < (t * 1000000)) { }
  LoopTimer = micros();
}
