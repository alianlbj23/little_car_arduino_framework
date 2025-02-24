#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include <PID_v1.h>

#define MAX_MOTORS 8

class PIDManager {
   private:
    uint8_t motorCount;    // Number of motors
    double *inputs;        // Array of motor inputs
    double *outputs;       // Array of motor outputs
    double *setpoints;     // Array of motor setpoints
    PID **pidControllers;  // Array of PID controllers

   public:
    PIDManager(double kp, double ki, double kd, uint8_t motorCount) {
        this->motorCount = motorCount;

        // Allocate memory for the arrays
        inputs = new double[motorCount];
        outputs = new double[motorCount];
        setpoints = new double[motorCount];
        pidControllers = new PID *[motorCount];

        // Initialize the PID controllers
        for (int i = 0; i < motorCount; i++) {
            pidControllers[i] = new PID(&inputs[i], &outputs[i], &setpoints[i], kp, ki, kd, DIRECT);
            pidControllers[i]->SetOutputLimits(0, 255);  // Set the output limits for the PID controllers

            pidControllers[i]->SetMode(AUTOMATIC);  // Set the PID mode to AUTOMATIC
        }
    }

    double *compute(double *inputs) {
        // Update the inputs for each motor
        for (int i = 0; i < motorCount; i++) {
            this->inputs[i] = fabs(inputs[i]);
        }

        // Compute the output for each PID controller
        for (int i = 0; i < motorCount; i++) {
            pidControllers[i]->Compute();
        }

        // Return the outputs
        return outputs;
    }

    double *getOutputs() {
        // Return the outputs
        return outputs;
    }

    void setSetpoints(double *setpointArray) {
        // Set the setpoints for each motor
        for (int i = 0; i < motorCount; i++) {
            setpoints[i] = fabs(setpointArray[i]);
        }
    }
};

#endif