#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <ESP32Encoder.h>

#define MAX_ENCODERS 4

class EncoderManager {
   public:
    EncoderManager(uint8_t pins[][2], const uint8_t encoderCount, const uint16_t encoder_resolutions[]);

    int *getCounts();

    float *getAngularVel();

   private:
    uint8_t encoderCount;                // Number of encoders
    ESP32Encoder *encoders;              // Dynamic array of encoder objects
    const uint16_t *encoderResolutions;  // Array of encoder resolutions
    unsigned long prevTime;              // Previous time for RPM calculation
    int *prevCounts;                     // Previous encoder counts for RPM calculation
};

#endif