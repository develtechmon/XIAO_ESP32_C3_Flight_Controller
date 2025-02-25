#include <ESP32Servo.h>
#include <PPMReader.h>

#define CHANNEL_DEFAULT_VALUE 1500
#define interruptPin 4

#define channelAmount 6
int CH[channelAmount];

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

PPMReader ppm(interruptPin, channelAmount); // PPM Library Object

void setup() {
  Serial.begin(115200);

  // Attach servos to GPIO pins
  servo1.attach(6); // Roll
  servo2.attach(7); // Pitch
  servo3.attach(8); // Throttle
  servo4.attach(9); // Yaw
  servo5.attach(10);
  servo6.attach(11);

  // Initialize channels to default value
  for (int i = 0; i < channelAmount; i++) {
    CH[i] = CHANNEL_DEFAULT_VALUE;
  }

  Serial.println("PPM Reader Initialized");
}

void PPM_width_values() {
  // Retrieve PPM values for each channel
  CH[0] = ppm.latestValidChannelValue(1, CHANNEL_DEFAULT_VALUE);
  CH[1] = ppm.latestValidChannelValue(2, CHANNEL_DEFAULT_VALUE);
  CH[2] = ppm.latestValidChannelValue(3, CHANNEL_DEFAULT_VALUE);
  CH[3] = ppm.latestValidChannelValue(4, CHANNEL_DEFAULT_VALUE);
  CH[4] = ppm.latestValidChannelValue(5, CHANNEL_DEFAULT_VALUE);
  CH[5] = ppm.latestValidChannelValue(6, CHANNEL_DEFAULT_VALUE);

  // Write PPM values to servos
  servo1.write(map(CH[0], 1000, 2000, 0, 180));
  servo2.write(map(CH[1], 1000, 2000, 0, 180));
  servo3.write(map(CH[2], 1000, 2000, 0, 180));
  servo4.write(map(CH[3], 1000, 2000, 0, 180));
  servo5.write(map(CH[4], 1000, 2000, 0, 180));
  servo6.write(map(CH[5], 1000, 2000, 0, 180));

  // Print PPM values for debugging
  Serial.print("CH1 (Roll): ");
  Serial.print(CH[0]);
  Serial.print(" CH2 (Pitch): ");
  Serial.print(CH[1]);
  Serial.print(" CH3 (Throttle): ");
  Serial.print(CH[2]);
  Serial.print(" CH4 (Yaw): ");
  Serial.print(CH[3]);
  Serial.print(" CH5: ");
  Serial.print(CH[4]);
  Serial.print(" CH6: ");
  Serial.println(CH[5]);
}

void loop() {
  PPM_width_values();
}
