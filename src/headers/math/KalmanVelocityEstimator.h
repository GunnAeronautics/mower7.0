#ifndef KALMAN_VELOCITY_ESTIMATOR_H
#define KALMAN_VELOCITY_ESTIMATOR_H

#include "../config/config.h"

class KalmanVelocityEstimator {
private:
    float processVar; // Process noise (model uncertainty)
    float sensorVar;  // Sensor noise (measurement uncertainty)

    float altitudeEst; // Estimated altitude
    float velocityEst; // Estimated vertical velocity

    float P_alt;       // Uncertainty in altitude estimate
    float P_vel;       // Uncertainty in velocity estimate

public:
    // Constructor
    KalmanVelocityEstimator(float processVar, float sensorVar);

    // Update with new acceleration and altitude measurement
    float update(float accel, float measuredAltitude, float dt);

    // Getters
    float getAltitude() const;
    float getVelocity() const;
};

#endif // KALMAN_VELOCITY_ESTIMATOR_H
