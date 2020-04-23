// ------------------------------VENTILATOR CONTROL---------------------------------- //

//TODO: LOW Refactor to have all variables extremely modular and not hard coded (later)

// ----------------------------------LIBRARIES--------------------------------------- //
#include "includes.h"
// ----------------------------------LIBRARIES--------------------------------------- //



// ----------------------------------DEBUGGING--------------------------------------- //
#define SYSTEM__SERIAL_DEBUG__STATEMACHINE 1
// ----------------------------------DEBUGGING--------------------------------------- //



// ----------------------------------CONSTANTS--------------------------------------- //
#define INCHES_2_CM (double)2.54

#define DEFAULT_BAUD_RATE 256000

#define PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS (double)1450
#define PINCH_VALVE_DRIVER_FULL_CLOSE_PULSE_MICROSECONDS (double)1725

#define MIN_PERCENTAGE (double)0
#define MAX_PERCENTAGE (double)100

#define BUZZER_PIN 2
#define PINCH_VALVE_PIN 4
// #define SOLENOID_PIN 4
#define BLOWER_PIN 5

#define DEFAULT_PEEP (double)5.000000
#define DEFAULT_PIP (double)20.000000
#define DEFAULT_BM 10  // breaths per minute
#define DEFAULT_RISE 1000  // 1 second
#define DEFAULT_IE 2 // inhale/exhale ratio
#define DEFAULT_KP (double)1.000000
#define DEFAULT_KI (double)1.000000
#define DEFAULT_KD (double)0.000000

#define DEFAULT_INHALE_RAMP (uint32_t)250

#define BREATHS_PER_MINUTE_TO_SEC (double)60.000000
#define SEC_TO_MS (double)1000.000000

#define DEFAULT_PINCH_VALVE_MIN_DWELL_TIME (uint32_t)250

#define IE_DEFAULT_SCALING_FACTOR (double)10.000000

#define PEEP_LOW_ALARM 1
#define PIP_ALARM 2
#define PEEP_ALARM 2

#define DEFAULT_INHALE_DURATION (uint32_t)1500
#define DEFAULT_BREATH_CYCLE_DURATION (uint32_t)6000
#define DEFAULT_CONTROL_LOOP_INIT_STABILIZATION (uint32_t)3000

#define DEFAULT_PID_SAMPLE_TIME 100

typedef enum{
    INHALE_RAMP,
    INHALE_HOLD,
    EXHALE_RAMP,
    EXHALE_HOLD,
    IDLE
}BreathCycleStep;
BreathCycleStep CurrCycleStep;

//Servo blower;
Servo pinch_valve;

AllSensors_DLHR_L60D_8 gagePressure(&Wire);
// ----------------------------------CONSTANTS--------------------------------------- //



// --------------------------------USER SETTINGS------------------------------------- //
double pressure_reading;
double blower_speed;

double PeepPressureCentimetersH2O = DEFAULT_PEEP;
double PipPressureCentimetersH2O = DEFAULT_PIP;

String string_from_pi;
String property_name;
double value;

double valve_position, valve_state;

double peep_low_alarm, peep_alarm, pip_alarm;
bool buzzer_state = 1;
double RespritoryRate;
double InhalationExhalationRatio;
double FlowOfOxygen;
double IEScalingFactor = IE_DEFAULT_SCALING_FACTOR;
bool CMD;
// --------------------------------USER SETTINGS------------------------------------- //



// --------------------------------STATE TIMINGS------------------------------------- //
uint32_t CurrTimeInCycleMilliseconds; // Time since the start of the current breath cycle. Resets at the beginning of every breath cycle
uint32_t CycleStartTimeFromSysClockMilliseconds;  // Time that the current breath cycle started ( in terms of system clock millis() )
uint32_t ControlLoopStartTimeMilliseconds; // Time, in terms of millis(), the state machine last switched out of IDLE
uint32_t ControlLoopInitialStabilizationTimeMilliseconds = DEFAULT_CONTROL_LOOP_INIT_STABILIZATION; // Length of time after transitioning out of IDLE that the system waits before transitioning to INHALE_RAMP
uint32_t InhaleRampDurationMilliseconds = DEFAULT_INHALE_RAMP; // Length of the INHALE_RAMP period for a breath cycle. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD .User configurable
uint32_t InhaleDurationMilliseconds = DEFAULT_INHALE_DURATION; // Combined length of the INHALE_RAMP and INHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to EXHALE_HOLD. User configurable.
uint32_t ExhaleDurationMilliseconds = 1500; // Combined length of the EXHALE_RAMP and EXHALE_HOLD periods. AKA Value of CurrTimeInCycleMilliseconds when the state changes to INHALE_HOLD. User configurable.
uint32_t BreathCycleDurationMilliseconds = InhaleDurationMilliseconds + ExhaleDurationMilliseconds; // Total length of breath cycle, AKA when cycle step resets to INHALE_RAMP and CurrTimeInCycleMilliseconds resets to 0

uint32_t TimeOfLastSolenoidToggleMilliseconds = 0; // Time, in terms of millis(), that the solenoid last changed states
// --------------------------------STATE TIMINGS------------------------------------- //



// --------------------------------PID SETTINGS-------------------------------------- //
// TODO: MEDIUM reorganize constants
// Pressure Controlled Blower PID
double pressure_system_input, blower_pressure_adjustment, pinch_valve_output_openness_in_percentage, CurrPressureSetpointCentimetersH2O;
// double Blower_Kp = DEFAULT_KP, Blower_Ki = DEFAULT_KI, Blower_Kd = DEFAULT_KD;
double Blower_Kp=1, Blower_Ki=.01, Blower_Kd=0.008000;
double PinchValve_Kp = 5, PinchValve_Ki = 0, PinchValve_Kd = 1;
PID Blower_PID(&pressure_system_input,
                &blower_pressure_adjustment,
                &CurrPressureSetpointCentimetersH2O,
                Blower_Kp, Blower_Ki, Blower_Kd, DIRECT);

PID PinchValve_PID(&pressure_system_input,
                &pinch_valve_output_openness_in_percentage,
                &CurrPressureSetpointCentimetersH2O,
                PinchValve_Kp, PinchValve_Kd, PinchValve_Kd, REVERSE);

// --------------------------------PID SETTINGS-------------------------------------- //



void breath_cycle_timer_reset(bool hardreset = false)
{
  CurrTimeInCycleMilliseconds = 0;
  CycleStartTimeFromSysClockMilliseconds = millis();
  if(hardreset)
  {
    ControlLoopStartTimeMilliseconds = CycleStartTimeFromSysClockMilliseconds;
  }
}

void pinch_valve_init (void)
{
  pinMode(PINCH_VALVE_PIN, OUTPUT);
  pinch_valve.attach(PINCH_VALVE_PIN);
  pinch_valve.writeMicroseconds(PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS);
}

void pressure_sensors_init (void)
{
  Wire.begin();  // Each pressure sensor will be unique I2C addresses based on MPN
  gagePressure.setPressureUnit(AllSensors_DLHR::PressureUnit::IN_H2O);
  gagePressure.startMeasurement(AllSensors_DLHR::AVERAGE4);
}

void pid_init (void)
{
  Blower_PID.SetMode(AUTOMATIC);  // Set PID Mode to Automatic, may change later
  Blower_PID.SetOutputLimits(-10, 10);
  Blower_PID.SetSampleTime(DEFAULT_PID_SAMPLE_TIME);

  PinchValve_PID.SetMode(AUTOMATIC);  // Set PID Mode to Automatic, may change later
  PinchValve_PID.SetOutputLimits(MIN_PERCENTAGE, MAX_PERCENTAGE);
  PinchValve_PID.SetSampleTime(DEFAULT_PID_SAMPLE_TIME);

}

void buzzer_init(void)
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

double get_pressure_reading (void)
{
  if(!gagePressure.readData(false))
  {
    // Get a pressure reading and convert to units of cmH2O
    if( gagePressure.pressure > 0 )
       pressure_reading = gagePressure.pressure * INCHES_2_CM;
    gagePressure.startMeasurement();
  }
  return pressure_reading;
}

void get_values_from_raspberry_pi (void)
{
  // TODO: HIGH Break out serial protocol into seperate file
  // TODO: HIGH ensure serial operations do not dirupt control loop
  // TODO: Add error checking for data out of range
  // TODO: Ensure that all fields are populated before going into running test

  // $<property_name>,<value><LF>

  if (Serial.available())
  {
    string_from_pi = Serial.readStringUntil('*');
    if(string_from_pi[0] == '$')
    {
      property_name = string_from_pi.substring(string_from_pi.indexOf('$') + 1, string_from_pi.indexOf(','));
      value = string_from_pi.substring(string_from_pi.indexOf(',') + 1, string_from_pi.indexOf('*')).toFloat();

      if(property_name.equalsIgnoreCase("PIP"))
      {
        PipPressureCentimetersH2O = value;  // PIP Value
        Serial.print("PIP: ");
        Serial.println(PipPressureCentimetersH2O);
      }

      else if(property_name.equalsIgnoreCase("PEEP"))
      {
        PeepPressureCentimetersH2O = value;  // PEEP Value
        Serial.print("PEEP: ");
        Serial.println(PeepPressureCentimetersH2O);
      }

      else if(property_name == "FIO2")
      {
        FlowOfOxygen = value;  // Flow of O2 in %
        Serial.print("FIO2: ");
        Serial.println(FlowOfOxygen);
      }

      else if(property_name.equalsIgnoreCase("TRISE"))
      {
        InhaleRampDurationMilliseconds = value * (double)100;  // Rise time in seconds
        Serial.print("TRISE: ");
        Serial.println(InhaleRampDurationMilliseconds);
      }

      else if(property_name.equalsIgnoreCase("RR"))
      {
        RespritoryRate = value;  // Respritory Rate in breathes per minute
        Serial.print("RR: ");
        Serial.println(RespritoryRate);
      }

      else if(property_name.equalsIgnoreCase("IE"))
      {
        InhalationExhalationRatio = value / IEScalingFactor;  // Inhalation/Exhalation Ratio
        InhaleDurationMilliseconds = (BREATHS_PER_MINUTE_TO_SEC * SEC_TO_MS) / ((InhalationExhalationRatio + 1.0) * RespritoryRate);
        ExhaleDurationMilliseconds = (BREATHS_PER_MINUTE_TO_SEC * SEC_TO_MS * (1.0 - (1.0 / (InhalationExhalationRatio + 1.0)))) / RespritoryRate;
        Serial.print("IE: ");
        Serial.println(InhalationExhalationRatio);
        Serial.println(InhaleDurationMilliseconds);
        Serial.println(ExhaleDurationMilliseconds);
      }

      else if( property_name.equalsIgnoreCase("CMD") )
      {
        CMD = value;

        if(CMD == 0)
        {
          Serial.println("TEST STOPPED");
          CurrCycleStep = IDLE;
        }

        else if(CMD == 1)
        {
          if(CurrCycleStep == IDLE)
          {
            CurrCycleStep = EXHALE_HOLD;
            breath_cycle_timer_reset(true);
            Serial.println("TEST STARTED");
            Serial.print("PEEP: ");         Serial.print(PeepPressureCentimetersH2O);                     Serial.println("cmH20");
            Serial.print("PIP: ");          Serial.print(PipPressureCentimetersH2O);                      Serial.println("cmH20");
            Serial.print("FIO2: ");         Serial.print(FlowOfOxygen);                                   Serial.println("cmH20");
            Serial.print("TRISE: ");        Serial.print(InhaleRampDurationMilliseconds);                 Serial.println("ms");
            Serial.print("RR: ");           Serial.print(RespritoryRate);                                 Serial.println("b/m");
            Serial.print("IE: ");           Serial.print((1.00 / InhalationExhalationRatio) * 100.00);    Serial.println("%");
          }
        }

        else
        {
          Serial.println("UNKNOWN CMD CODE, GOING TO IDLE STATE");
          CurrCycleStep = IDLE;
        }
      }

      else
      {
        Serial.println("UNKNOWN MESSAGE, GOING TO IDLE STATE");
        CurrCycleStep = IDLE;
      }
    }
  }
}

void print_pid_setpoint_and_current_value(void)
{
  if( CurrCycleStep != IDLE )
  {
    if( millis() % 50 == 0 )
    {
      Serial.print(pressure_system_input);
      Serial.print(" ");
      switch(CurrCycleStep)
      {
        case INHALE_RAMP:
          
          Serial.print(CurrPressureSetpointCentimetersH2O);
        break;
        case INHALE_HOLD:
          Serial.print(PipPressureCentimetersH2O);
        break;
        case EXHALE_RAMP:
        case EXHALE_HOLD:
        case IDLE:
        default:
          Serial.print(PeepPressureCentimetersH2O);
        break;
      }
      Serial.print(" ");
      Serial.print(getCurrentBlowerSpeed());
      Serial.println();

    }
  }
}

void reset_blower_pid_integrator(void)
{
   Blower_PID.SetMode(MANUAL);
   blower_pressure_adjustment = 0;
   Blower_PID.SetMode(AUTOMATIC);
}

void write_calculated_pid_blower_speed(void)
{
      if( CurrCycleStep != IDLE )
      {
        Blower_PID.SetTunings(Blower_Kp, Blower_Ki, Blower_Kd);
        Blower_PID.Compute();
      }
      else
      {
        reset_blower_pid_integrator();
      }
      // Output PID calcuslated 0-100% to motor
      blowerTargetSpeedPercent = blowerPressureToBlowerSpeed(CurrPressureSetpointCentimetersH2O);// + blower_pressure_adjustment);
}

// Write percent openness to the pinch valve
// 0% = closed
// 100% = wide open
void write_pinch_valve_openness(double opennessPercentage)
{
  long pinchValvePulseLengthMicroseconds = 
  map(opennessPercentage,
      MIN_PERCENTAGE,
      MAX_PERCENTAGE,
      PINCH_VALVE_DRIVER_FULL_CLOSE_PULSE_MICROSECONDS,
      PINCH_VALVE_DRIVER_FULL_OPEN_PULSE_MICROSECONDS);
  pinch_valve.writeMicroseconds(pinchValvePulseLengthMicroseconds);
}

void pinch_valve_control(void)
{
  switch (CurrCycleStep)
  {
  case EXHALE_RAMP:
    pinch_valve_output_openness_in_percentage = 100.0;
    break;
  case EXHALE_HOLD:
    PinchValve_PID.SetTunings(PinchValve_Kp,PinchValve_Ki,PinchValve_Kd);
    PinchValve_PID.Compute();
    break;
  case INHALE_HOLD:
  case INHALE_RAMP:
    pinch_valve_output_openness_in_percentage = 0.0;
    break;
  default:
    pinch_valve_output_openness_in_percentage = 100.0;
    break;
  }

  write_pinch_valve_openness(pinch_valve_output_openness_in_percentage);  
}

void cycle_state_handler (void)
{
  // TODO: LOW Make sure clock overflow is handled gracefully
  if( CurrCycleStep != IDLE )
  {
    CurrTimeInCycleMilliseconds = millis()-CycleStartTimeFromSysClockMilliseconds;
    if(millis()-ControlLoopStartTimeMilliseconds > ControlLoopInitialStabilizationTimeMilliseconds)
    {
      if(CurrTimeInCycleMilliseconds <= InhaleRampDurationMilliseconds)
      {
        CurrCycleStep = INHALE_RAMP;
      }
      else if((InhaleRampDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
              (CurrTimeInCycleMilliseconds <= InhaleDurationMilliseconds))
      { 
        CurrCycleStep = INHALE_HOLD;
      }
      else if((InhaleDurationMilliseconds < CurrTimeInCycleMilliseconds) &&
            (CurrTimeInCycleMilliseconds <= BreathCycleDurationMilliseconds))
      {
        if( pressure_system_input > (PeepPressureCentimetersH2O + 5) )
        {
          CurrCycleStep = EXHALE_RAMP;
        }
        else
        {
          CurrCycleStep = EXHALE_HOLD;
        }
      }
      else if(CurrTimeInCycleMilliseconds > BreathCycleDurationMilliseconds)
      {
        CurrCycleStep = INHALE_RAMP;
        breath_cycle_timer_reset();
      }
    }
    else // if idle == true
    {
      CurrCycleStep = EXHALE_HOLD;
    }
  }
}

// Compensation for DC offset in pressure achieved.
// Found by linear regression of the following experimental datapoints:
// Target Val	Setpoint Req'd
// 15       	16
// 20	        21
// 25	        27.9
// 30	        33.5
// 35	        38.5
// 40	        44.5
// 45	        49.25
// 50	        55

double linear_remap_setpoint_compensation(double setpoint)
{
  return (1.116785714*setpoint)-0.5892857143;
}

void cycle_state_setpoint_handler(void)
{
  // TODO: HIGH breakout into separate file and rewrite for readability
  // TODO: MEDIUM put gains in a header, not in the source

  //Recompute Setpoints
  switch(CurrCycleStep)
  {
    case INHALE_RAMP:
      CurrPressureSetpointCentimetersH2O = (((float)CurrTimeInCycleMilliseconds/(float)InhaleRampDurationMilliseconds)*(PipPressureCentimetersH2O-PeepPressureCentimetersH2O))+PeepPressureCentimetersH2O;
    break;
    case INHALE_HOLD:
      CurrPressureSetpointCentimetersH2O = PipPressureCentimetersH2O;
    break;
    case EXHALE_RAMP:
    case EXHALE_HOLD:
    case IDLE:
    default:
      CurrPressureSetpointCentimetersH2O = PeepPressureCentimetersH2O;
    break;
  }
}

void buzzer_toggle(void)
{
  static unsigned long lastBuzzerToggle = 0;
  if( millis()-lastBuzzerToggle > 500 )
  {
    digitalWrite(BUZZER_PIN, buzzer_state);
    buzzer_state = !buzzer_state;
    lastBuzzerToggle = millis();
  }
}

void alarms_settings(void)
{
  peep_low_alarm = PEEP_LOW_ALARM;
  peep_alarm = PEEP_ALARM;
  pip_alarm = PIP_ALARM;

  // // FOR TESTING
  // // CurrCycleStep = EXHALE_HOLD;
  // CurrCycleStep = INHALE_HOLD;

  // pressure_system_input = 5.0;
  // // FOR TESTING

  // 1 & 2a: High and Low PIP
  if((CurrCycleStep == EXHALE_HOLD && CurrCycleStep != EXHALE_RAMP && CurrCycleStep != INHALE_RAMP)
      && (pressure_system_input <= PipPressureCentimetersH2O - pip_alarm
      || pressure_system_input >= PipPressureCentimetersH2O + pip_alarm))
  {
    // make sound and send Raspberry Pi alarm status flag
    buzzer_toggle();
    Serial.write("PIP ERROR");
  }

  // 2b & 2c: High and Low PEEP
  if((CurrCycleStep == INHALE_HOLD && CurrCycleStep != EXHALE_RAMP && CurrCycleStep != INHALE_RAMP)
      && (pressure_system_input <= PeepPressureCentimetersH2O - peep_alarm
      || pressure_system_input >= PeepPressureCentimetersH2O + peep_alarm))
  {
    // make sound and send Raspberry Pi alarm status flag
    buzzer_toggle();
    Serial.write("PEEP ERROR");
  }

  // 2g: Disconnect Alarm
  if((CurrCycleStep == EXHALE_HOLD && CurrCycleStep != INHALE_RAMP)
      && (pressure_system_input >= -peep_low_alarm
      && pressure_system_input <= peep_low_alarm))
  {
    // make sound and send Raspberry Pi alarm status flag
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.write("DISCONNECT ERROR");
  }
}

void alarms_faults(void)
{
  //
}

void setup()
{
  // buzzer_init();

  // Initializations
  initBlower();
  pinch_valve_init();
  pressure_sensors_init();
  pid_init();

  // Start cycle state in IDLE state
  CurrCycleStep = IDLE;

  // Serial initialization
  #if SYSTEM__SERIAL_DEBUG__STATEMACHINE
  Serial.begin(DEFAULT_BAUD_RATE);
  #endif

  // breath_cycle_timer_reset(true);
}

void loop()
{
  get_values_from_raspberry_pi();

  // alarms_settings();

  pressure_system_input = get_pressure_reading();

  cycle_state_handler();

  cycle_state_setpoint_handler();

  pinch_valve_control();

  write_calculated_pid_blower_speed();

  recompute_blower_control_loop();

  print_pid_setpoint_and_current_value();

  get_values_from_raspberry_pi();

  // TODO: HIGH Implement alarms, with serial protocol to inform the GUI/HMI
}
