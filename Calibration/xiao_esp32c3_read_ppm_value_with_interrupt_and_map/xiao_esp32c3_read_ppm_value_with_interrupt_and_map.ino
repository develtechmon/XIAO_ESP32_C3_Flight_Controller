#include <ESP32Servo.h>

// Pin where the PPM signal is connected
#define PPM_PIN 4 // GPIO pin for PPM input

// Number of channels
#define NUM_CHANNELS 8

// Timing constants
#define PPM_SYNC_THRESHOLD 3000 // Sync pulse threshold (microseconds)
#define CHANNEL_MIN 1000        // Minimum valid pulse width (microseconds)
#define CHANNEL_MAX 2000        // Maximum valid pulse width (microseconds)

volatile int ReceiverValue[NUM_CHANNELS]; // Store channel values
volatile int channelIndex = 0;            // Current channel index
volatile unsigned long lastTime = 0;      // Last pulse time

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

void ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // Detected sync pulse, reset channel index
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Store the pulse width as channel value
    ReceiverValue[channelIndex] = constrain(pulseWidth, CHANNEL_MIN, CHANNEL_MAX);
    channelIndex++;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("PPM Reader Initialized");

  // Set the PPM pin as input and attach interrupt
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

  // Attach servos to GPIO pins
  servo1.attach(6); // Roll
  servo2.attach(7); // Pitch
  servo3.attach(8); // Throttle
  servo4.attach(9); // Yaw
  servo5.attach(10);
  servo6.attach(11);

  // Initialize channels to default value
  for (int i = 0; i < NUM_CHANNELS; i++) {
    ReceiverValue[i] = CHANNEL_MIN;
  }
}

void read_receiver(int* channelValues) {
  noInterrupts(); // Temporarily disable interrupts
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts(); // Re-enable interrupts
}

void loop() {
  int channelValues[NUM_CHANNELS];

  // Read the receiver values
  read_receiver(channelValues);

  // Write PPM values to servos
  servo1.write(map(channelValues[0], CHANNEL_MIN, CHANNEL_MAX, 0, 180));
  servo2.write(map(channelValues[1], CHANNEL_MIN, CHANNEL_MAX, 0, 180));
  servo3.write(map(channelValues[2], CHANNEL_MIN, CHANNEL_MAX, 0, 180));
  servo4.write(map(channelValues[3], CHANNEL_MIN, CHANNEL_MAX, 0, 180));
  servo5.write(map(channelValues[4], CHANNEL_MIN, CHANNEL_MAX, 0, 180));
  servo6.write(map(channelValues[5], CHANNEL_MIN, CHANNEL_MAX, 0, 180));

  // Print PPM values for debugging
  Serial.print(" Roll [µs]: ");
  Serial.print(channelValues[0]);
  Serial.print(" Pitch [µs]: ");
  Serial.print(channelValues[1]);
  Serial.print(" Throttle [µs]: ");
  Serial.print(channelValues[2]);
  Serial.print(" Yaw [µs]: ");
  Serial.print(channelValues[3]);
  Serial.print(" Aux1 [µs]: ");
  Serial.print(channelValues[4]);
  Serial.print(" Aux2 [µs]: ");
  Serial.println(channelValues[5]);

  delay(50);
}
