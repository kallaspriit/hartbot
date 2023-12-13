#include <Arduino.h>

// printing with stream operator helper functions (adds Serial << "Hey"; functionality)
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}

// pin configuration
const int PIN_REVERSE_BUTTON = 2;
const int PIN_THROTTLE_ADC = A0;
const int PIN_POWER_ADC = A1;
const int PIN_MOTOR_ENABLE = 13;
const int PIN_MOTOR_FORWARD_PWM = 14;
const int PIN_MOTOR_REVERSE_PWM = 15;

// analog input range configuration
const int THROTTLE_MIN = 340; // mininum ADC reading for min throttle
const int THROTTLE_MAX = 715; // maximum ADC reading for max throttle
const int POWER_MIN = 5;      // mininum ADC reading for min speed potentiometer
const int POWER_MAX = 1020;   // maximum ADC reading for max speed potentiometer

// digital-analog-converter settings
const int ANALOG_OUTPUT_RANGE = 512;

// minimum power used when max power potentiometer is at minimal setting (less and it won't move)
constexpr float MIN_POWER_PERCENTAGE = 30.0f;
constexpr int MIN_POWER_OUTPUT = (int)((MIN_POWER_PERCENTAGE / 100.0f) * (float)ANALOG_OUTPUT_RANGE);

// limit reverse speed to 70% max power
constexpr float REVERSE_POWER_LIMIT_PERCENTAGE = 70.0f;
constexpr int REVERSE_POWER_LIMIT_OUTPUT = (int)((REVERSE_POWER_LIMIT_PERCENTAGE / 100.0f) * (float)ANALOG_OUTPUT_RANGE);

// runtime state
bool useReverse = false;
int debugLedState = LOW;

// timing
unsigned long lastDebugTimeMs = 0;

// setup robot
void setup()
{
  // ignore flow control so serial data is received in terminals without DTR
  Serial.ignoreFlowControl(true);

  // open serial connection to pc
  Serial.begin(115200);

  // use pullup for the reverse mode button input
  pinMode(PIN_REVERSE_BUTTON, INPUT_PULLUP);

  // use analog-digital-converters to read throttle and max speed
  pinMode(PIN_THROTTLE_ADC, INPUT);
  pinMode(PIN_POWER_ADC, INPUT);

  // configure motor driver pins
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_FORWARD_PWM, OUTPUT);
  pinMode(PIN_MOTOR_REVERSE_PWM, OUTPUT);

  // motor is always enabled
  digitalWrite(PIN_MOTOR_ENABLE, HIGH);

  // use the built-in debug led on the board
  pinMode(LED_BUILTIN, OUTPUT);

  // set anlog output range
  analogWriteRange(ANALOG_OUTPUT_RANGE);
}

// update robot
void loop()
{
  unsigned long currentTimeMs = millis();

  // detect current reverse mode state
  useReverse = digitalRead(PIN_REVERSE_BUTTON) == LOW;

  // read the analog-digital converter raw values
  int throttleValue = analogRead(PIN_THROTTLE_ADC);
  int maxPowerValue = analogRead(PIN_POWER_ADC);

  // convert max power and throttle values between zero to analog output range
  int maxPower = constrain(map(maxPowerValue, POWER_MIN, POWER_MAX, MIN_POWER_OUTPUT, ANALOG_OUTPUT_RANGE), MIN_POWER_OUTPUT, ANALOG_OUTPUT_RANGE);
  int throttle = constrain(map(throttleValue, THROTTLE_MIN, THROTTLE_MAX, 0, ANALOG_OUTPUT_RANGE), 0, ANALOG_OUTPUT_RANGE);

  // apply reverse power limit
  if (useReverse)
  {
    maxPower = min(maxPower, REVERSE_POWER_LIMIT_OUTPUT);
  }

  // map the throttle to max power range (whole throttle stick range remains usable)
  int outputPower = constrain(map(throttle, 0, ANALOG_OUTPUT_RANGE, 0, maxPower), 0, maxPower);

  // apply motor speed pwm in the requested direction
  if (useReverse)
  {
    analogWrite(PIN_MOTOR_FORWARD_PWM, 0);
    analogWrite(PIN_MOTOR_REVERSE_PWM, outputPower);
  }
  else
  {
    analogWrite(PIN_MOTOR_FORWARD_PWM, outputPower);
    analogWrite(PIN_MOTOR_REVERSE_PWM, 0);
  }

  // print debug info at interval and blink onboard led
  if (currentTimeMs - lastDebugTimeMs >= 1000)
  {
    Serial << "Reverse: " << (useReverse ? "yes" : "no") << "\n";
    Serial << "Throttle: " << ((float)throttle / (float)ANALOG_OUTPUT_RANGE * 100.0f) << "% (" << throttleValue << ")\n";
    Serial << "Max power: " << ((float)maxPower / (float)ANALOG_OUTPUT_RANGE * 100.0f) << "% (" << maxPowerValue << ")\n";
    Serial << "Output power: " << ((float)outputPower / (float)ANALOG_OUTPUT_RANGE * 100.0f) << "% (" << outputPower << ")\n";
    Serial << "\n";

    // blink the debug led to indicate the board is operational
    digitalWrite(LED_BUILTIN, debugLedState);

    lastDebugTimeMs = currentTimeMs;
    debugLedState = !debugLedState;
  }
}
