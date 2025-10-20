#include "headers/math/KalmanVelocityEstimator.h"

// Constructor
KalmanVelocityEstimator::KalmanVelocityEstimator(float processVar, float sensorVar)
    : processVar(processVar), sensorVar(sensorVar),
      altitudeEst(0.0f), velocityEst(0.0f),
      P_alt(1.0f), P_vel(1.0f) {}

// Update step: acceleration input and altitude measurement
float KalmanVelocityEstimator::update(float accel, float measuredAltitude,float dt) {
    // -------- Predict Step --------
    velocityEst += accel * dt;
    altitudeEst += velocityEst * dt;

    // Increase uncertainty due to motion
    P_vel += processVar;
    P_alt += P_vel * dt + 0.5f * processVar;

    // -------- Correction Step --------
    float error = measuredAltitude - altitudeEst;
    float K = P_alt / (P_alt + sensorVar); // Kalman gain

    altitudeEst += K * error;
    velocityEst += K * (error / dt);

    P_alt *= (1.0f - K);
    P_vel *= (1.0f - K);

    return velocityEst;
}

// Getters
float KalmanVelocityEstimator::getAltitude() const {
    return altitudeEst;
}

float KalmanVelocityEstimator::getVelocity() const {
    return velocityEst;
}
