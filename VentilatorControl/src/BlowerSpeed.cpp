#include "BlowerSpeed.h"
#include <Arduino.h>
#include <PID_v1.h>
#include "helpers.h"
#include <math.h>

#define BLOWER_KP 1.2
#define BLOWER_KI 2.0
#define BLOWER_KD 0.0
#define BLOWER_MAX_SPEED_FREQUENCY_HERTZ 2300.0 
#define MICROSECONDS_TO_SECONDS 1000000.0

unsigned long averageBlowerPulsePeriod;
unsigned long lastBlowerSpeedRecalc;
volatile unsigned long blowerPulseSum;
volatile unsigned long lastBlowerPulseMicroseconds;

// All percentages go from 0.0-100.0
double blowerCurrentSpeedPercent;
double blowerAverageSpeedPercent;
double blowerPWMCommandPercent;
double blowerTargetSpeedPercent;

#define NUM_SPEED_SAMPLES 10
uint8_t blowerSampleIndex = 0;
double blowerSpeedSamples[NUM_SPEED_SAMPLES];

double blowerKp = BLOWER_KP;
double blowerKi = BLOWER_KI;
double blowerKd = BLOWER_KD;

PID blowerPIDLoop = PID(&blowerAverageSpeedPercent, &blowerPWMCommandPercent, &blowerTargetSpeedPercent, blowerKp, blowerKi, blowerKd,DIRECT);

void setupTimers();
void setBlowerPWMPercent(double speedPercent);
void onBlowerSpeedRisingEdge();

void initBlower()
{
    blowerPIDLoop.SetOutputLimits(10.0, 100.0);
    blowerPIDLoop.SetSampleTime(100);
    blowerPIDLoop.SetMode(AUTOMATIC);
    lastBlowerSpeedRecalc = micros();
    blowerPulseSum = 0;
    averageBlowerPulsePeriod = -1;
    blowerTargetSpeedPercent = 0;
    lastBlowerPulseMicroseconds = 0;

    for(int idx = 0; idx < NUM_SPEED_SAMPLES; idx++ )
    {
        blowerSpeedSamples[idx] = 0;
    }

    attachInterrupt(BLOWER_SPEED_FEEDBACK_INTERRUPT,onBlowerSpeedRisingEdge, FALLING);
    setupTimers();
    setBlowerPWMPercent(0);
}

void reset_pid_integrator(void)
{
   blowerPIDLoop.SetMode(MANUAL);
   blowerPWMCommandPercent = 0;
   blowerPIDLoop.SetMode(AUTOMATIC);
}

void recompute_blower_control_loop(void)
{
    unsigned long currTime = micros();
    //Average pulse count over 100ms
    if( (currTime - lastBlowerSpeedRecalc) > 10000)
    {
        if(blowerPulseSum > 0)
        {
            averageBlowerPulsePeriod = (currTime - lastBlowerSpeedRecalc)/blowerPulseSum;
            blowerCurrentSpeedPercent = mapf(MICROSECONDS_TO_SECONDS/(double)averageBlowerPulsePeriod, 0.0, BLOWER_MAX_SPEED_FREQUENCY_HERTZ, 0.0, 100.0);
        }
        else
        {
            blowerCurrentSpeedPercent = 0;
        }
        blowerPulseSum = 0;
        lastBlowerSpeedRecalc = currTime;

        blowerSpeedSamples[blowerSampleIndex] = blowerCurrentSpeedPercent;
        blowerSampleIndex++;
        if(blowerSampleIndex >= NUM_SPEED_SAMPLES)
        {
            blowerSampleIndex = 0;
        }

        blowerAverageSpeedPercent = 0;
        for(int idx = 0;idx < NUM_SPEED_SAMPLES; idx++)
        {
            blowerAverageSpeedPercent+=blowerSpeedSamples[idx];
        }
        blowerAverageSpeedPercent = blowerAverageSpeedPercent/(double)NUM_SPEED_SAMPLES;


    }

    if(blowerTargetSpeedPercent*1.2 < blowerAverageSpeedPercent)
    {
        reset_pid_integrator();
    }

    blowerPIDLoop.Compute();
    setBlowerPWMPercent(blowerPWMCommandPercent);
    //setBlowerPWMPercent(blowerTargetSpeedPercent);
}

double getCurrentBlowerSpeed()
{
    return blowerAverageSpeedPercent ;
}

//Average 8 samples, so left shift 3 places 
#define  NUM_SAMPLES_TO_AVERAGE_SHIFT_AMOUNT 3

void onBlowerSpeedRisingEdge()
{
    unsigned long currTime = micros();
    if( currTime-lastBlowerPulseMicroseconds >= 450 )
    {
        blowerPulseSum++;
        lastBlowerPulseMicroseconds = currTime;
    }
}

void onTimerPeriodElapsed(){
    if(blowerPulseSum > 3)
        {
            averageBlowerPulsePeriod = 1e5/blowerPulseSum;
            blowerCurrentSpeedPercent = mapf(MICROSECONDS_TO_SECONDS/(double)averageBlowerPulsePeriod, 0.0, BLOWER_MAX_SPEED_FREQUENCY_HERTZ, 0.0, 100.0);
            blowerPulseSum = 0;
        }
        else
        {
            blowerCurrentSpeedPercent = 0;
        }
}

void setBlowerPWMPercent(double speedPercent)
{
    uint32_t speedPercentScaled = (uint32_t)(speedPercent*10.0);
    REG_TCC1_CC1 = speedPercentScaled;
}

// Found via regression analysis from static pressure testing of sample blower
// r^2 = 0.99 for this regression on the blower tested.
// see data at https://docs.google.com/spreadsheets/d/1GVmF7gMihPsArEmjk8MefS8RsMpl3gKHL-qg1a0rInc/edit?usp=sharing
double blowerPressureToBlowerSpeed(double pressure)
{
    return sqrt(pressure)*14.16199 + pressure*0.07209 + 3.6105;
}

// Output PWM 24Khz on digital pin D3  and D11 using timer TCC1 (10-bit resolution)
void setupTimers()
{  
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor N=1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin D3 and D11  **** g_APinDescription() converts Arduino Pin to SAMD21 pin
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC1 timer to digital output D3 and D11 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_E;  // D3 is on PA11 = odd, use Device E on TCC1/WO[1]

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC1_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH;     // Setup dual slope PWM on TCC0
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: Freq = 48Mhz/(2*N*PER)
  REG_TCC1_PER = 1000;                           // Set the FreqTcc of the PWM on TCC1 to 24Khz
  while (TCC1->SYNCBUSY.bit.PER);                // Wait for synchronization
 
  // Set the PWM signal to output , PWM ds = 2*N(TOP-CCx)/Freqtcc => PWM=0 => CCx=PER, PWM=50% => CCx = PER/2
  REG_TCC1_CC1 = 500;                             // TCC1 CC1 - on D3  50%
  while (TCC1->SYNCBUSY.bit.CC1);                   // Wait for synchronization
 
  // Divide the GCLOCK signal by 1 giving  in this case 48MHz (20.83ns) TCC1 timer tick and enable the outputs
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}