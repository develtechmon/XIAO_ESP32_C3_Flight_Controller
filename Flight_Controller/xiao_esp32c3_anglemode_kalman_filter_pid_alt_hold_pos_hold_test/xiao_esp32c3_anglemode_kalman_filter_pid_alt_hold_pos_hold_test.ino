#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>

// ===== PPM Definitions =====
#define PPM_PIN 10             // Pin where the PPM signal is connected
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Pulse width (µs) above which a sync pulse is assumed
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;

int ESCfreq = 500;
int channelValues[NUM_CHANNELS];
float PAngleRoll=2;       float PAnglePitch=PAngleRoll;
float IAngleRoll=0.5;     float IAnglePitch=IAngleRoll;
float DAngleRoll=0.007;   float DAnglePitch=DAngleRoll;

float PRateRoll = 0.625;  float PRatePitch = PRateRoll;
float IRateRoll = 2.1;    float IRatePitch = IRateRoll;
float DRateRoll = 0.0088; float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0};

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

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

// ===== New: GPS and Baro =====

// Create a TinyGPSPlus object for the Ublox NEO-M8N
TinyGPSPlus gps;
// Use HardwareSerial2 for GPS (ESP32 pins: RX=16, TX=17)
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600
HardwareSerial gpsSerial(2);

// Create an Adafruit_BMP280 object for the barometer
Adafruit_BMP280 bmp; // I2C, address 0x76 (adjust if needed)

// Variables for GPS position hold
float currentLatitude = 0, currentLongitude = 0;
float targetLatitude = 0, targetLongitude = 0;
bool gpsTargetSet = false;  // set true once target is latched

// Variables for altitude hold
float currentAltitude = 0;
float targetAltitude = 0;
bool altTargetSet = false;

// PID parameters for position hold (convert error in meters to desired angle correction)
// These constants will need to be tuned for your vehicle.
float P_GPS_Pitch = 0.05;  // for north-south error (affecting pitch)
float I_GPS_Pitch = 0.0;
float D_GPS_Pitch = 0.005;
float ItermGPSPitch = 0;
float prevErrorGPSPitch = 0;

float P_GPS_Roll = 0.05;   // for east-west error (affecting roll)
float I_GPS_Roll = 0.0;
float D_GPS_Roll = 0.005;
float ItermGPSRoll = 0;
float prevErrorGPSRoll = 0;

// PID parameters for altitude hold (adjust throttle)
// Example constants; tune these as needed.
float P_Alt = 2.0;
float I_Alt = 0.5;
float D_Alt = 0.1;
float ItermAlt = 0;
float prevErrorAlt = 0;

// Sea level pressure for altitude calculation (hPa)
#define SEALEVELPRESSURE_HPA 1013.25

// ===== PPM Interrupt Handler =====
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // A long pulse is assumed to be the sync pulse – reset channel index.
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Store a valid channel pulse (constrained to valid range)
    if(pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if(pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
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
    KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4); // variance of IMU = 4 deg/s
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3); // error std = 3 deg
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState; 
    Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  Serial.begin(115200);
  
  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  // ----- Setup I2C and MPU6050 (for attitude) -----
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ----- Setup BMP280 Barometer -----
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1) delay(10);
  }
  // Optional: set BMP280 sampling parameters (using Adafruit defaults)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500);

  // ----- Setup GPS (Ublox NEO-M8N) -----
  // Use Serial2 for GPS; pins defined above.
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Serial2 started at 9600 baud rate");

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
  RateCalibrationRoll = 0.60;
  RateCalibrationPitch = -2.34;
  RateCalibrationYaw = -0.44;
  AccXCalibration = 0.03;
  AccYCalibration = -0.02;
  AccZCalibration = 0.20;

  // (Optional) Wait for a safe throttle reading from RC
  LoopTimer = micros();
}

void loop() {
  read_receiver(channelValues);

  // ----- Read MPU6050 Data (accelerometer and gyroscope) -----
  // (Your original code for reading MPU6050 remains unchanged)
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
  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;
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

  // Clamp EKF filter roll and pitch to ±20 degrees
  KalmanAngleRoll  = constrain(KalmanAngleRoll, -20, 20);
  KalmanAnglePitch = constrain(KalmanAnglePitch, -20, 20);

  // ----- Read RC Channels (Original Desired Angles from RC) -----
  // (For example, channels 0 and 1 for roll/pitch commands, channel 2 for throttle, channel 3 for yaw)
  // These are scaled such that 1500 is neutral.
  float rcDesiredRoll  = 0.1 * (ReceiverValue[0] - 1500);
  float rcDesiredPitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle        = ReceiverValue[2];
  DesiredRateYaw       = 0.15 * (ReceiverValue[3] - 1500);

  // ----- New: Read GPS Data -----
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    currentLatitude  = gps.location.lat();
    currentLongitude = gps.location.lng();
  }
  // Optionally, display or log GPS info:
  // Serial.print("GPS: ");
  // Serial.print(currentLatitude, 6); Serial.print(" , ");
  // Serial.println(currentLongitude, 6);

  // If a certain RC channel (say channel 7) is above 1500, enable position hold
  bool positionHold = (ReceiverValue[6] > 1500);
  if (positionHold) {
    // Latch target position once when hold is activated
    if (!gpsTargetSet) {
      targetLatitude  = currentLatitude;
      targetLongitude = currentLongitude;
      gpsTargetSet = true;
      Serial.println("GPS target latched.");
    }
    // Compute error (in degrees) and convert to meters (approximation)
    float errorLat = targetLatitude - currentLatitude; // north-south error
    float errorLon = targetLongitude - currentLongitude; // east-west error
    float errorLat_m = errorLat * 111320.0;  // approx conversion
    float errorLon_m = errorLon * 111320.0 * cos(currentLatitude * DEG_TO_RAD);
    
    // Compute PID corrections for position hold (affecting desired pitch and roll)
    float errorGPSPitch = errorLat_m; // positive error means you are south of target: need to pitch forward to move north
    float errorGPSRoll  = errorLon_m;  // positive error means you are west of target: need to roll right to move east

    // Simple PID for pitch correction
    ItermGPSPitch += I_GPS_Pitch * (errorGPSPitch + prevErrorGPSPitch) * (t / 2);
    ItermGPSPitch = constrain(ItermGPSPitch, -50, 50);
    float DtermGPSPitch = D_GPS_Pitch * ((errorGPSPitch - prevErrorGPSPitch) / t);
    float correctionPitch = P_GPS_Pitch * errorGPSPitch + ItermGPSPitch + DtermGPSPitch;
    prevErrorGPSPitch = errorGPSPitch;

    // Simple PID for roll correction
    ItermGPSRoll += I_GPS_Roll * (errorGPSRoll + prevErrorGPSRoll) * (t / 2);
    ItermGPSRoll = constrain(ItermGPSRoll, -50, 50);
    float DtermGPSRoll = D_GPS_Roll * ((errorGPSRoll - prevErrorGPSRoll) / t);
    float correctionRoll = P_GPS_Roll * errorGPSRoll + ItermGPSRoll + DtermGPSRoll;
    prevErrorGPSRoll = errorGPSRoll;

    // Add the GPS corrections to the RC desired angles
    DesiredAnglePitch = rcDesiredPitch + correctionPitch;
    DesiredAngleRoll  = rcDesiredRoll  + correctionRoll;
  } else {
    // If position hold is not active, reset latch
    gpsTargetSet = false;
    DesiredAnglePitch = rcDesiredPitch;
    DesiredAngleRoll  = rcDesiredRoll;
  }

  // ----- New: Read BMP280 for Altitude Hold -----
  currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  // If a certain RC channel (say channel 8) is above 1500, enable altitude hold
  bool altitudeHold = (ReceiverValue[7] > 1500);
  if (altitudeHold) {
    if (!altTargetSet) {
      targetAltitude = currentAltitude;
      altTargetSet = true;
      Serial.println("Altitude target latched.");
    }
    float errorAlt = targetAltitude - currentAltitude;
    ItermAlt += I_Alt * (errorAlt + prevErrorAlt) * (t / 2);
    ItermAlt = constrain(ItermAlt, -100, 100);
    float DtermAlt = D_Alt * ((errorAlt - prevErrorAlt) / t);
    float altCorrection = P_Alt * errorAlt + ItermAlt + DtermAlt;
    prevErrorAlt = errorAlt;
    // Override throttle: add altitude correction (positive correction increases throttle)
    InputThrottle += altCorrection;
  } else {
    altTargetSet = false;
    // Use RC throttle value as-is
  }

  // ----- Angle Mode PID (for Roll and Pitch) -----
  // Compute errors for angle PID (desired from above vs. estimated via Kalman filter)
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = constrain(PIDOutputRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = constrain(PIDOutputPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // ----- Rate PID (for Roll, Pitch, Yaw) -----
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Roll Axis PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = constrain(PIDOutputRoll, -400, 400);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Axis PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = constrain(PIDOutputPitch, -400, 400);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Axis PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = constrain(ItermYaw, -400, 400);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = constrain(PIDOutputYaw, -400, 400);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  // Limit throttle
  if (InputThrottle > 1800) { InputThrottle = 1800; }

  // Compute final motor inputs (mixing)
  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw; // front right
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw; // rear right
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw; // rear left
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw; // front left

  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);

  // Disarm motors if throttle is very low
  if (ReceiverValue[2] < 1030) {
      MotorInput1 = ThrottleCutOff;
      MotorInput2 = ThrottleCutOff;
      MotorInput3 = ThrottleCutOff;
      MotorInput4 = ThrottleCutOff;

      PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
      PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
      PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
      PrevItermAngleRoll = PrevItermAnglePitch = 0;
  }

  // Write motor outputs
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  // Maintain loop timing
  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
