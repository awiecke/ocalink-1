#ifndef __BLOWERSPEED_H__
#define __BLOWERSPEED_H__

#define BLOWER_SPEED_OUPUT_PIN 3
#define BLOWER_SPEED_FEEDBACK_INTERRUPT (digitalPinToInterrupt(A1))

#include <PID_v1.h>

extern double blowerTargetSpeedPercent;
extern double blowerPWMCommandPercent;
extern unsigned long averageBlowerPulsePeriod;
extern double blowerAverageSpeedPercent;

extern double blowerKp;
extern double blowerKi;
extern double blowerKd;

extern PID blowerPIDLoop;

void initBlower();
void recompute_blower_control_loop();
double getCurrentBlowerSpeed();
double blowerPressureToBlowerSpeed(double pressure);

#endif