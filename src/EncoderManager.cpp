#include "EncoderManager.h"

#include <Arduino.h>  // Include Arduino core library

EncoderManager::EncoderManager(uint8_t pins[][2], const uint8_t encoderCount, const uint16_t encoder_resolutions[]) {
    this->encoderCount = encoderCount;
    this->encoderResolutions = encoder_resolutions;

    // Initialize the encoders
    encoders = new ESP32Encoder[encoderCount];
    prevCounts = new int[encoderCount];
    for (uint8_t i = 0; i < encoderCount; i++) {
        encoders[i].attachSingleEdge(pins[i][0], pins[i][1]);
        encoders[i].clearCount();
        encoders[i].setCount(0);
        prevCounts[i] = 0;
    }
    prevTime = millis();  // millis() is now recognized
}

int *EncoderManager::getCounts() {
    static int counts[MAX_ENCODERS];

    for (int i = 0; i < encoderCount; i++) {
        counts[i] = encoders[i].getCount();
    }

    return counts;
}

float *EncoderManager::getAngularVel() {
    static float vels[MAX_ENCODERS];

    unsigned long currentTime = millis();  // millis() is now recognized
    float deltaTime = (currentTime - prevTime) / 1000.0;
    int *counts = getCounts();
    for (int i = 0; i < this->encoderCount; i++) {
        int encoderDiff = counts[i] - prevCounts[i];
        if (encoderDiff == 0) {
            vels[i] = 0;
            continue;
        }
        vels[i] = (2 * 3.14159 * encoderDiff / this->encoderResolutions[i]) / deltaTime;
        prevCounts[i] = counts[i];
    }

    prevTime = currentTime;

    return vels;
}