/*
  Centrifuge

  The following libraries must be installed through the Arduino IDE's Library Manager:
  - EWMA
*/

#include <Ewma.h>
#include <EwmaT.h>
#include <PID_v1.h>

// Timing
unsigned long current_time;

// Motor
static const uint8_t motor_pin = 5;

// Potentiometer
static const uint8_t potentiometer_pin = A0;
static const unsigned int potentiometer_floor = 0;
static const unsigned int potentiometer_ceiling = 960;

// Tachometer
static const uint8_t tachometer_pin = 2;
static const unsigned long counts_per_rev = 4;
unsigned long tachometer_counter = 0;
unsigned long tachometer_counter_previous = 0;
unsigned long tachometer_time_previous = 0;
unsigned long tachometer_counter_sampling_threshold = 20;
unsigned long tachometer_time_sampling_threshold = 100;

// State variables
double angular_velocity;

// Plotting
Ewma angular_velocity_smoother(0.2);
static const unsigned long plot_interval = 100;
unsigned long plot_time_previous = 0;

// Control
static const unsigned long control_sampling_interval = plot_interval;
static const unsigned long min_speed = 0;
static const unsigned long max_speed = 3000;
double motor_duty;
double setpoint;
double k_p = 0.5;
double k_i = 0.01;
double k_d = 0.01;
PID pid(&angular_velocity_smoother.output, &motor_duty, &setpoint, k_p, k_i, k_d, DIRECT);


void tachometerInterruptHandler() {
  ++tachometer_counter;
}

// the setup routine runs once when you press reset:
void setup() {
  
  // Indicators
  pinMode(LED_BUILTIN, OUTPUT);

  // Tachometer
  pinMode(tachometer_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), tachometerInterruptHandler, CHANGE);

  // PID Control
  pid.SetSampleTime(control_sampling_interval);
  pid.SetMode(AUTOMATIC);
  
  // Plotting
  Serial.begin(115200);
  Serial.println("setpoint, angvel, angvel_smooth, motor_effort");
}

// the loop routine runs over and over again forever:
void loop() {
  unsigned long new_current_time = millis();

  // Read the tachometer
  unsigned long tachometer_counter_current = tachometer_counter;
  unsigned long tachometer_counter_delta = tachometer_counter_current - tachometer_counter_previous;
  const unsigned long tachometer_time_delta = millis() - tachometer_time_previous;
  if ((tachometer_counter_delta > tachometer_counter_sampling_threshold && tachometer_time_delta > 0) || tachometer_time_delta > tachometer_time_sampling_threshold) {
    digitalWrite(LED_BUILTIN, tachometer_time_delta > tachometer_time_sampling_threshold);
    angular_velocity = 60.0 * 1000 * tachometer_counter_delta / counts_per_rev / tachometer_time_delta; // rpm
    angular_velocity_smoother.filter(angular_velocity);
    tachometer_counter_previous = tachometer_counter_current;
    tachometer_time_previous = new_current_time;
  }

  // Read the potentiometer
  int potentiometer_value = constrain(analogRead(potentiometer_pin), potentiometer_floor, potentiometer_ceiling);

  // Update motor effort
  setpoint = map(potentiometer_value, potentiometer_floor, potentiometer_ceiling, min_speed, max_speed);
  pid.Compute();
  analogWrite(motor_pin, motor_duty);

  // Plot the data
  if (new_current_time - plot_time_previous > plot_interval) {
    Serial.print(setpoint);
    Serial.print(", ");
    Serial.print(angular_velocity);
    Serial.print(", ");
    Serial.print(angular_velocity_smoother.output);
    Serial.print(", ");
    Serial.print(map(motor_duty, 0, 255, min_speed, max_speed));
    Serial.println();
    plot_time_previous = new_current_time;
  }

  current_time = new_current_time;
}
