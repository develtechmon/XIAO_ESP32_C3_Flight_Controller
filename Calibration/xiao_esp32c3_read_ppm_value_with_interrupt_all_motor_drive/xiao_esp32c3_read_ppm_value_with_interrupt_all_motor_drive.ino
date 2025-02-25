#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ===== PPM Definitions =====
#define PPM_PIN 10             // Pin where the PPM signal is connected
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Pulse width (µs) above which a sync pulse is assumed
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;

int ESCfreq = 500;
int channelValues[NUM_CHANNELS];

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

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
// Use IRAM_ATTR for faster interrupt handling on the ESP32.
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

void setup() {
  Serial.begin(115200);

  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

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
  delay(500);

    // Wait until a valid throttle reading is received via PPM (channel index 2)

  while (true) {
    read_receiver(channelValues);
    // Assume channel 2 (index 2) is throttle; wait until it is near mid-range
    if (channelValues[2] > 1020 && channelValues[2] < 1050)
      break;
    delay(4);
  }
}

void loop() {
  read_receiver(channelValues);

  InputThrottle = channelValues[2];

  MotorInput1 = InputThrottle;
  MotorInput2 = InputThrottle;
  MotorInput3 = InputThrottle;
  MotorInput4 = InputThrottle;

    // ----- Write Motor PWM Signals -----
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  // Optional: Print receiver channels for debugging
  Serial.print("Roll [µs]: "); Serial.print(channelValues[0]);
  Serial.print("  Pitch [µs]: "); Serial.print(channelValues[1]);
  Serial.print("  Throttle [µs]: "); Serial.print(channelValues[2]);
  Serial.print("  Yaw [µs]: "); Serial.println(channelValues[3]);
}

