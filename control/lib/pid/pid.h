#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

/**
 * @brief PID mode enumeration
 */
typedef enum {
    PID_MODE_POSITION = 0,  // Position PID (for servo/position control)
    PID_MODE_INCREMENTAL    // Incremental PID (for motor speed control)
} PidMode_t;

#ifdef __cplusplus

/**
 * @brief PID controller class
 *
 * Supports both position PID and incremental PID modes.
 * In C++ mode, use directly: Pid pid(...); pid.update(val);
 * In C mode, use C-linkage API: Pid_Create(), Pid_Update(), etc.
 */
class Pid {
public:
    /**
     * @brief Constructor
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param maxOut Maximum output value
     * @param minOut Minimum output value
     */
    Pid(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
        float maxOut = 100.0f, float minOut = -100.0f);

    /**
     * @brief Update PID controller and calculate output
     * @param currentValue Current process value
     * @return Calculated PID output
     */
    float update(float currentValue);

    /** @brief Set target value */
    void setSetpoint(float setpoint);

    /** @brief Set PID tunings */
    void setTunings(float kp, float ki, float kd);

    /** @brief Set output limits */
    void setLimits(float maxOut, float minOut);

    /** @brief Reset PID state (zeros all history) */
    void reset();

    /**
     * @brief Set PID mode and reset state
     * @param mode PID_MODE_POSITION or PID_MODE_INCREMENTAL
     */
    void setMode(PidMode_t mode);

    // ── Public data members (same layout as C struct below) ──
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain
    float maxOutput;       // Maximum output
    float minOutput;       // Minimum output
    float setpoint;        // Target value
    float lastError;       // Last error
    float lastLastError;   // Last last error (for incremental PID)
    float integral;        // Integral accumulator
    float lastOutput;      // Last output (for incremental PID)
    PidMode_t mode;        // PID mode

private:
    float updatePosition(float currentValue);
    float updateIncremental(float currentValue);
};

// In C++ mode, Pid_t is an alias for the Pid class
typedef Pid Pid_t;

extern "C" {

#else  /* !__cplusplus */

/**
 * @brief PID controller structure (C-compatible layout)
 *
 * Memory layout is identical to the C++ Pid class,
 * so pointers are interchangeable between C and C++.
 */
typedef struct {
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain
    float maxOutput;       // Maximum output
    float minOutput;       // Minimum output
    float setpoint;        // Target value
    float lastError;       // Last error
    float lastLastError;   // Last last error (for incremental PID)
    float integral;        // Integral accumulator
    float lastOutput;      // Last output (for incremental PID)
    PidMode_t mode;        // PID mode
} Pid;

typedef Pid Pid_t;

#endif /* __cplusplus */

// ═══════════════════════════════════════════════════════════
// C-linkage API — works from both C and C++ callers
// ═══════════════════════════════════════════════════════════

/** @brief Create a PID controller instance (heap allocated) */
Pid_t* Pid_Create(void);

/** @brief Destroy a PID controller instance */
void Pid_Destroy(Pid_t* self);

/** @brief Initialize PID parameters */
void Pid_Init(Pid_t* self, float kp, float ki, float kd,
              float maxOut, float minOut);

/** @brief Update PID and get output */
float Pid_Update(Pid_t* self, float currentValue);

/** @brief Set target value */
void Pid_SetSetpoint(Pid_t* self, float setpoint);

/** @brief Set PID tunings */
void Pid_SetTunings(Pid_t* self, float kp, float ki, float kd);

/** @brief Set output limits */
void Pid_SetLimits(Pid_t* self, float maxOut, float minOut);

/** @brief Reset PID state */
void Pid_Reset(Pid_t* self);

/** @brief Set PID mode (resets state) */
void Pid_SetMode(Pid_t* self, PidMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
