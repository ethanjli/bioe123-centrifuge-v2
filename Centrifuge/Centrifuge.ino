/*
  Centrifuge

  The following libraries must be installed through the Arduino IDE's Library Manager:
  - EWMA by Arsen Torbarina
  - PID by Brett Beauregard
  - JC_Button by Jack Christensen
  - JLed by Jan Delgado
*/

#include <Ewma.h>
#include <EwmaT.h>
#include <PID_v1.h>
#include <JC_Button.h>
#include <jled.h>

// Timing
unsigned long current_time;

// Indicators
static const unsigned long ramping_blink_on = 100; // ms; duration of ON period of blink during speed ramp-up
static const unsigned long ramping_blink_off = 400; // ms; duration of OFF period of blink during speed ramp-up
static const unsigned long compensating_blink_on = 50; // ms; duration of ON period of blink while PID is compensating to fully ramped-up target speed
static const unsigned long compensating_blink_off = 50; // ms; duration of OFF period of blink while PID is compensating to fully ramped-up target speed
JLed led(LED_BUILTIN);

// Motor
static const uint8_t motor_pin = 5;

// Button
static const uint8_t button_pin = 3;
Button button(button_pin);

// Potentiometer
static const uint8_t potentiometer_pin = A0;
static const unsigned int potentiometer_floor = 0; // unitless (0 - 1023); at or below this threshold, potentiometer value is always just this value
static const unsigned int potentiometer_ceiling = 1000; // unitless (0 - 1023); at or above this threshold, potentiometer value is always just this value
static const double potentiometer_smoothing = 0.1; // unitless (0 - 1); amount of smoothing of potentiometer readings, with lower numbers giving stronger smoothing
Ewma potentiometer_smoother(potentiometer_smoothing);

// Tachometer
static const uint8_t tachometer_pin = 2;
static const unsigned long counts_per_rev = 4; // includes both rising and falling edges
unsigned long tachometer_counter = 0; // edges; total number of edges accumulated
unsigned long tachometer_counter_previous = 0; // edges; number of edges accumulated in previous tachometer sampling step
unsigned long tachometer_time_previous = 0; // ms; time of previous tachometer sampling step
unsigned long tachometer_counter_sampling_threshold = 20; // Hz; preferred number of edges accumulated for a rate estimate
unsigned long tachometer_time_sampling_threshold = 100; // Hz; minimum allowed frequency for tachometer sampling
double angular_velocity = 0; // rpm; raw estimate of angular velocity
static const double angular_velocity_smoothing = 0.2; // unitless (0 - 1); amount of smoothing of tachometer samples, with lower numbers giving stronger smoothing
Ewma angular_velocity_smoother(angular_velocity_smoothing);
static const double speed_epsilon = 1; // rpm; margin of error within which two speeds are considered equal

// Plotting
static const unsigned long plot_interval = 100; // ms; preferred duration between plotting steps
unsigned long plot_time_previous = 0; // ms; time of previous plotting step

// Target speeds
static const unsigned long min_speed = 0; // rpm; minimum settable centrifugation speed
static const unsigned long max_speed = 4000; // rpm; maximum settable centrifugation speed

// Motor control
static const unsigned long motor_control_interval = 100; // ms; preferred duration between motor control PID sampling steps
static const double stationary_threshold = min_speed + speed_epsilon; // rpm; maximum speed below which centrifuge is considered stationary
double motor_duty = 0; // unitless (0 - 255); PWM duty cycle for motor
double potentiometer_speed = 0; // rpm; speed set by the potentiometer, regardless of whether centrifuge is operating
double motor_setpoint = 0; // rpm; motor speed setpoint at the current time of ramping
static const double motor_k_p = 0.25;
static const double motor_k_i = 0.1;
static const double motor_k_d = 0.025;
PID motor_pid(&angular_velocity_smoother.output, &motor_duty, &motor_setpoint, motor_k_p, motor_k_i, motor_k_d, DIRECT);

// Speed ramping
static const unsigned long ramp_control_interval = 10; // ms; preferred duration between speed ramping steps
unsigned long ramp_time_previous = 0; // ms; time of previous speed rampling step
double target_speed = 0; // speed target for ramping
static const double ramp_k_p = 1; // does not need to be tuned
static const double ramp_k_i = 0; // does not need to be tuned
static const double ramp_k_d = 0; // does not need to be tuned
double ramp_increment = 0; // rpm; amount by which setpoint will be adjusted
static const double max_increment = 20.0 / ramp_control_interval; // rpm; maximum amount by which setpoint can be adjusted per speed ramping step
PID ramp_pid(&motor_setpoint, &ramp_increment, &target_speed, ramp_k_p, ramp_k_i, ramp_k_d, DIRECT);

// State variables
bool operating = false;
//static const double convergence_margin = 50 * 1000; // rpm * ms; margin of error of the integral of speed error within which a speed is considered to have converged on the target
static const double convergence_margin = 50; // rpm; margin of error within which a speed is considered to have converged on the target
enum class MotorState {
  idle,
  ramping,
  compensating,
  converged
};
MotorState current_state = MotorState::idle;
MotorState previous_state = MotorState::idle;

void tachometerInterruptHandler() {
  ++tachometer_counter;
}


void setup() {
  
  // Indicators
  led.Off();

  // Button
  button.begin();

  // Potentiometer
  pinMode(potentiometer_pin, INPUT);

  // Tachometer
  pinMode(tachometer_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), tachometerInterruptHandler, CHANGE);

  // Speed Ramping
  ramp_pid.SetSampleTime(ramp_control_interval);
  ramp_pid.SetOutputLimits(-max_increment, max_increment);
  ramp_pid.SetMode(AUTOMATIC);

  // PID Control
  motor_pid.SetSampleTime(motor_control_interval);
  motor_pid.SetMode(AUTOMATIC);
  
  // Plotting
  Serial.begin(115200);
  Serial.print("target_speed, ");
  Serial.print("motor_setpoint, ");
  //Serial.print("angvel, ");
  Serial.print("angvel_smooth, ");
  Serial.print("motor_effort");
  Serial.println();
}


void loop() {
  unsigned long new_current_time = millis();

  // Read the button
  button.read();
  if (button.wasPressed()) {
    operating = !operating;
  }

  // Read the tachometer
  unsigned long tachometer_counter_current = tachometer_counter;
  unsigned long tachometer_counter_delta = tachometer_counter_current - tachometer_counter_previous;
  const unsigned long tachometer_time_delta = new_current_time - tachometer_time_previous;
  if ((tachometer_counter_delta > tachometer_counter_sampling_threshold && tachometer_time_delta > 0) || tachometer_time_delta > tachometer_time_sampling_threshold) {
    //digitalWrite(LED_BUILTIN, tachometer_time_delta > tachometer_time_sampling_threshold);
    angular_velocity = 60.0 * 1000 * tachometer_counter_delta / counts_per_rev / tachometer_time_delta;
    angular_velocity_smoother.filter(angular_velocity);
    tachometer_counter_previous = tachometer_counter_current;
    tachometer_time_previous = new_current_time;
  }

  // Read the potentiometer
  potentiometer_smoother.filter(constrain(analogRead(potentiometer_pin), potentiometer_floor, potentiometer_ceiling));
  potentiometer_speed = map(potentiometer_smoother.output, potentiometer_floor, potentiometer_ceiling, min_speed, max_speed);

  // Update the target speed
  if (operating) {
    target_speed = potentiometer_speed;
  } else {
    target_speed = 0;
  }

  // Update motor setpoint
  if (target_speed == 0) { // no ramping when we want to completely stop
    ramp_pid.SetMode(MANUAL);
    motor_setpoint = min_speed;
  } else {
    ramp_pid.SetMode(AUTOMATIC);
    ramp_pid.Compute();
    if (new_current_time - ramp_time_previous > ramp_control_interval) {
      motor_setpoint += ramp_increment;
      ramp_time_previous = new_current_time;
    }
  }

  // Update motor effort
  if (motor_setpoint <= stationary_threshold) { // no PID when we want to completely stop
    motor_pid.SetMode(MANUAL);
    motor_duty = 0;
  } else {
    motor_pid.SetMode(AUTOMATIC);
    motor_pid.SetTunings(motor_k_p, motor_k_i, motor_k_d);
  }
  motor_pid.Compute();
  analogWrite(motor_pin, motor_duty);

  // Update motor state
  previous_state = current_state;
  if (motor_duty == 0 && angular_velocity_smoother.output <= stationary_threshold) {
    current_state = MotorState::idle;
  } else if (abs(target_speed - motor_setpoint) > speed_epsilon) {
    current_state = MotorState::ramping;
  } else if (abs(target_speed - angular_velocity_smoother.output) <= convergence_margin) {
    current_state = MotorState::converged;
  } else {
    current_state = MotorState::compensating;
  }

  // Update indicators
  if (current_state == MotorState::idle && previous_state != MotorState::idle) {
    led.Off();
  } else if (current_state == MotorState::ramping && previous_state != MotorState::ramping) {
    led.Blink(ramping_blink_on, ramping_blink_off).Forever();
  } else if (current_state == MotorState::compensating && previous_state != MotorState::compensating) {
    led.Blink(compensating_blink_on, compensating_blink_off).Forever();
  } else if (current_state == MotorState::converged && previous_state != MotorState::converged) {
    led.On();
  }
  led.Update();

  // Plot the data
  if (new_current_time - plot_time_previous > plot_interval) {
    Serial.print(target_speed);
    Serial.print(", ");
    Serial.print(motor_setpoint);
    Serial.print(", ");
    //Serial.print(angular_velocity);
    //Serial.print(", ");
    Serial.print(angular_velocity_smoother.output);
    Serial.print(", ");
    Serial.print(map(motor_duty, 0, 255, min_speed, max_speed));
    Serial.println();
    plot_time_previous = new_current_time;
  }

  current_time = new_current_time;
}
