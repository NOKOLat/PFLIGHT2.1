#include "cascade_pid_manager.hpp"

CascadePIDManager::CascadePIDManager(float dt)
    : angle_pitch_pid_(CascadePidConfig::Pitch::Angle::KP,
                       CascadePidConfig::Pitch::Angle::KI,
                       CascadePidConfig::Pitch::Angle::KD,
                       dt),
      rate_pitch_pid_(CascadePidConfig::Pitch::Rate::KP,
                      CascadePidConfig::Pitch::Rate::KI,
                      CascadePidConfig::Pitch::Rate::KD,
                      dt),
      angle_roll_pid_(CascadePidConfig::Roll::Angle::KP,
                      CascadePidConfig::Roll::Angle::KI,
                      CascadePidConfig::Roll::Angle::KD,
                      dt),
      rate_roll_pid_(CascadePidConfig::Roll::Rate::KP,
                     CascadePidConfig::Roll::Rate::KI,
                     CascadePidConfig::Roll::Rate::KD,
                     dt),
      angle_yaw_pid_(CascadePidConfig::Yaw::Angle::KP,
                     CascadePidConfig::Yaw::Angle::KI,
                     CascadePidConfig::Yaw::Angle::KD,
                     dt),
      rate_yaw_pid_(CascadePidConfig::Yaw::Rate::KP,
                    CascadePidConfig::Yaw::Rate::KI,
                    CascadePidConfig::Yaw::Rate::KD,
                    dt) {
}

void CascadePIDManager::calcCascadePIDAllAxes(
    float target_pitch, float measured_pitch,
    float target_roll, float measured_roll,
    float target_yaw_rate, float measured_yaw,
    float pid_result[3],
    float measured_pitch_rate,
    float measured_roll_rate,
    float measured_yaw_rate
) {
    // Calculate cascade PID for all three axes
    pid_result[0] = calcPitch(target_pitch, measured_pitch, measured_pitch_rate);
    pid_result[1] = calcRoll(target_roll, measured_roll, measured_roll_rate);
    pid_result[2] = calcYaw(target_yaw_rate, measured_yaw, measured_yaw_rate);
}

float CascadePIDManager::calcPitch(float target_angle, float measured_angle,
                                   float measured_rate) {
    // Outer loop: angle error -> target rate
    angle_pitch_pid_.calc(target_angle, measured_angle);
    float target_pitch_rate = angle_pitch_pid_.getData();

    // Inner loop: run every control cycle so dt matches the shared 10 ms loop
    rate_pitch_pid_.calc(target_pitch_rate, measured_rate);
    return rate_pitch_pid_.getData();
}

float CascadePIDManager::calcRoll(float target_angle, float measured_angle,
                                  float measured_rate) {
    // Outer loop: angle error -> target rate
    angle_roll_pid_.calc(target_angle, measured_angle);
    float target_roll_rate = angle_roll_pid_.getData();

    // Inner loop: run every control cycle so dt matches the shared 10 ms loop
    rate_roll_pid_.calc(target_roll_rate, measured_rate);
    return rate_roll_pid_.getData();
}

float CascadePIDManager::calcYaw(float target_rate, float /*measured_angle*/,
                                 float measured_rate) {
    // Rate-only control: skip angle outer loop
    rate_yaw_pid_.calc(target_rate, measured_rate);
    return rate_yaw_pid_.getData();
}

float CascadePIDManager::calcAnglePitch(float target_angle, float measured_angle) {
    angle_pitch_pid_.calc(target_angle, measured_angle);
    return angle_pitch_pid_.getData();
}

float CascadePIDManager::calcAngleRoll(float target_angle, float measured_angle) {
    angle_roll_pid_.calc(target_angle, measured_angle);
    return angle_roll_pid_.getData();
}

float CascadePIDManager::calcAngleYaw(float target_angle, float measured_angle) {
    angle_yaw_pid_.calc(target_angle, measured_angle);
    return angle_yaw_pid_.getData();
}

void CascadePIDManager::reset() {
    angle_pitch_pid_.reset();
    rate_pitch_pid_.reset();
    angle_roll_pid_.reset();
    rate_roll_pid_.reset();
    angle_yaw_pid_.reset();
    rate_yaw_pid_.reset();
}
