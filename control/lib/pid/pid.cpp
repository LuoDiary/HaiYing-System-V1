#include "pid.h"
#include <stdlib.h>

// ═══════════════════════════════════════════════════════════
// Pid class implementation
// ═══════════════════════════════════════════════════════════

Pid::Pid(float kp, float ki, float kd, float maxOut, float minOut)
    : kp(kp), ki(ki), kd(kd),
      maxOutput(maxOut), minOutput(minOut),
      setpoint(0.0f), lastError(0.0f), lastLastError(0.0f),
      integral(0.0f), lastOutput(0.0f),
      mode(PID_MODE_POSITION)
{
}

/**
 * @brief Position PID calculation with anti-windup
 *
 * u(k) = Kp * e(k) + Ki * Σe(k) + Kd * (e(k) - e(k-1))
 *
 * Anti-windup: integral term only accumulates when output
 * is within the valid range.
 */
float Pid::updatePosition(float currentValue) {
    float error = setpoint - currentValue;
    float derivative = (error - lastError) * kd;
    float output = kp * error + derivative;
    float tempIntegral = integral + error * ki;
    float tempOutput = output + tempIntegral;

    // Anti-windup: only update integral if output is within limits
    if (tempOutput <= maxOutput && tempOutput >= minOutput) {
        integral = tempIntegral;
        output = tempOutput;
    } else {
        // Output is saturated — clamp without accumulating integral
        output = kp * error + integral + derivative;
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < minOutput) {
            output = minOutput;
        }
    }

    lastError = error;
    lastOutput = output;
    return output;
}

/**
 * @brief Incremental PID calculation
 *
 * Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 * u(k) = u(k-1) + Δu(k)
 */
float Pid::updateIncremental(float currentValue) {
    float error = setpoint - currentValue;

    float deltaP = kp * (error - lastError);
    float deltaI = ki * error;
    float deltaD = kd * (error - 2.0f * lastError + lastLastError);

    float deltaOutput = deltaP + deltaI + deltaD;
    float output = lastOutput + deltaOutput;

    // Limit output range
    if (output > maxOutput) {
        output = maxOutput;
    } else if (output < minOutput) {
        output = minOutput;
    }

    // Shift error history
    lastLastError = lastError;
    lastError = error;
    lastOutput = output;
    return output;
}

float Pid::update(float currentValue) {
    if (mode == PID_MODE_INCREMENTAL) {
        return updateIncremental(currentValue);
    } else {
        return updatePosition(currentValue);
    }
}

void Pid::setSetpoint(float sp) {
    setpoint = sp;
}

void Pid::setTunings(float kp_, float ki_, float kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void Pid::setLimits(float maxOut, float minOut) {
    maxOutput = maxOut;
    minOutput = minOut;
}

void Pid::reset() {
    setpoint = 0.0f;
    lastError = 0.0f;
    lastLastError = 0.0f;
    integral = 0.0f;
    lastOutput = 0.0f;
}

void Pid::setMode(PidMode_t mode_) {
    mode = mode_;
    reset();
}

// ═══════════════════════════════════════════════════════════
// C-linkage API wrappers
// ═══════════════════════════════════════════════════════════

Pid_t* Pid_Create(void) {
    Pid_t* pid = new Pid();
    return pid;
}

void Pid_Destroy(Pid_t* self) {
    delete self;
}

void Pid_Init(Pid_t* self, float kp, float ki, float kd,
              float maxOut, float minOut) {
    self->kp = kp;
    self->ki = ki;
    self->kd = kd;
    self->maxOutput = maxOut;
    self->minOutput = minOut;
    self->setpoint = 0.0f;
    self->lastError = 0.0f;
    self->lastLastError = 0.0f;
    self->integral = 0.0f;
    self->lastOutput = 0.0f;
    self->mode = PID_MODE_POSITION;
}

float Pid_Update(Pid_t* self, float currentValue) {
    return self->update(currentValue);
}

void Pid_SetSetpoint(Pid_t* self, float setpoint) {
    self->setSetpoint(setpoint);
}

void Pid_SetTunings(Pid_t* self, float kp, float ki, float kd) {
    self->setTunings(kp, ki, kd);
}

void Pid_SetLimits(Pid_t* self, float maxOut, float minOut) {
    self->setLimits(maxOut, minOut);
}

void Pid_Reset(Pid_t* self) {
    self->reset();
}

void Pid_SetMode(Pid_t* self, PidMode_t mode) {
    self->setMode(mode);
}
