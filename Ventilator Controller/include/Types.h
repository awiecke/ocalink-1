// Define types used across the controller
#ifndef _VENTILATOR_TYPES_H_
#define _VENTILATOR_TYPES_H_

#include <cstdint>

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE,
    NUM_BREATHCYCLE_STEPS
}BreathCycleStep;

const char* BreathCycleStepNames[NUM_BREATHCYCLE_STEPS]
{
    "INHALE_RAMP",
    "INHALE_HOLD",
    "EXHALE"
};

typedef struct{
    BreathCycleStep CurrCycleStep;
    uint32_t CycleStartTimeFromSysClockMilliseconds;
    uint32_t CurrTimeInCycleMilliseconds;
}BreathCycleState;

typedef struct{
    uint32_t InhaleRampDurationMilliseconds;
    uint32_t InhaleDurationMilliseconds;
    uint32_t ExhaleDurationMilliseconds;
    uint32_t BreathCycleDurationMilliseconds;
    float PeepPressureCentimetersH2O;
    float PipPressureCentimetersH2O;
}BreathCycleSettings;

typedef struct{
    float PatientCircuitPressureCentimetersH2O;
    float OxygenFlowRateLitersPerMinute;
    float TotalFlowRateLitersPerMinute;
}SensorReadin;


#endif