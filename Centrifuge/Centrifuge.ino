/*
  Centrifuge

  The following libraries must be installed through the Arduino IDE's Library Manager:
  - EWMA
*/

#include <Ewma.h>
#include <EwmaT.h>
#include <PID_v1.h>
#include <JC_Button.h>

// Timing
unsigned long current_time;

// Motor
static const uint8_t motor_pin = 5;

// Button
static const uint8_t button_pin = 3;
Button button(button_pin);

// Potentiometer
static const uint8_t potentiometer_pin = A0;
static const unsigned int potentiometer_floor = 0;
static const unsigned int potentiometer_ceiling = 1000;
Ewma potentiometer_smoother(0.1);

// Tachometer
static const uint8_t tachometer_pin = 2;
static const unsigned long counts_per_rev = 4;
unsigned long tachometer_counter = 0;
unsigned long tachometer_counter_previous = 0;
unsigned long tachometer_time_previous = 0;
unsigned long tachometer_counter_sampling_threshold = 20;
unsigned long tachometer_time_sampling_threshold = 100;
Ewma angular_velocity_smoother(0.2);

// State variables
double angular_velocity = 0;
bool running = false;

// Plotting
static const unsigned long plot_interval = 100;
unsigned long plot_time_previous = 0;

// Motor control
static const unsigned long motor_control_interval = 100;
static const unsigned long min_speed = 0;
static const unsigned long max_speed = 4000;
double motor_duty = 0;
double target_speed = 0;
double motor_setpoint = 0;
static const double motor_k_p = 0.1;
static const double motor_k_i = 1;
static const double motor_k_d = 0.2;
PID motor_pid(&angular_velocity_smoother.output, &motor_duty, &motor_setpoint, motor_k_p, motor_k_i, motor_k_d, DIRECT);

// Speed ramping
static const unsigned long ramp_control_interval = 10;
unsigned long ramp_time_previous = 0;
static const double ramp_k_p = 1; // does not need to be tuned
static const double ramp_k_i = 0; // does not need to be tuned
static const double ramp_k_d = 0; // does not need to be tuned
double ramp_increment = 0;
static const double max_increment = 20.0 / ramp_control_interval;
PID ramp_pid(&motor_setpoint, &ramp_increment, &target_speed, ramp_k_p, ramp_k_i, ramp_k_d, DIRECT);


void tachometerInterruptHandler() {
  ++tachometer_counter;
}

// the setup routine runs once when you press reset:
void setup() {
  
  // Indicators
  pinMode(LED_BUILTIN, OUTPUT);

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

// the loop routine runs over and over again forever:
void loop() {
  unsigned long new_current_time = millis();

  // Read the button
  button.read();
  if (button.wasPressed()) {
    running = !running;
    digitalWrite(LED_BUILTIN, running);
  }

  // Read the tachometer
  unsigned long tachometer_counter_current = tachometer_counter;
  unsigned long tachometer_counter_delta = tachometer_counter_current - tachometer_counter_previous;
  const unsigned long tachometer_time_delta = new_current_time - tachometer_time_previous;
  if ((tachometer_counter_delta > tachometer_counter_sampling_threshold && tachometer_time_delta > 0) || tachometer_time_delta > tachometer_time_sampling_threshold) {
    //digitalWrite(LED_BUILTIN, tachometer_time_delta > tachometer_time_sampling_threshold);
    angular_velocity = 60.0 * 1000 * tachometer_counter_delta / counts_per_rev / tachometer_time_delta; // rpm
    angular_velocity_smoother.filter(angular_velocity);
    tachometer_counter_previous = tachometer_counter_current;
    tachometer_time_previous = new_current_time;
  }

  // Read the potentiometer
  potentiometer_smoother.filter(constrain(analogRead(potentiometer_pin), potentiometer_floor, potentiometer_ceiling));

  // Update the target speed
  if (running) {
    target_speed = map(potentiometer_smoother.output, potentiometer_floor, potentiometer_ceiling, min_speed, max_speed);
  } else {
    target_speed = 0;
  }

  // Update motor setpoint
  if (target_speed == 0) {
    ramp_pid.SetMode(MANUAL);
    motor_setpoint = 0;
  } else {
    ramp_pid.SetMode(AUTOMATIC);
    ramp_pid.Compute();
    if (new_current_time - ramp_time_previous > ramp_control_interval) {
      motor_setpoint += ramp_increment;
      ramp_time_previous = new_current_time;
    }
  }

  // Update motor effort
  if (motor_setpoint <= min_speed + 1) {
    motor_pid.SetTunings(motor_k_p, motor_k_i, motor_k_d);
  } else {
    motor_pid.SetTunings(motor_k_p, 0, 0);
  }
  motor_pid.Compute();
  analogWrite(motor_pin, motor_duty);

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
