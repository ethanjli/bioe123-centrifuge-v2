/*
  Centrifuge

  The following libraries must be installed through the Arduino IDE's Library Manager:
  - EWMA
*/

#include <Ewma.h>
#include <EwmaT.h>

unsigned long current_time = 0;
unsigned long blink_period = 100;
bool blink_state = false;
unsigned long blink_start_time = 0;

static const uint8_t motor_pin = 5;

// Potentiometer
static const uint8_t potentiometer_pin = A0;
static const unsigned int potentiometer_floor = 0;
static const unsigned int potentiometer_ceiling = 960;

// Tachometer
static const uint8_t tachometer_pin = 2;
static const unsigned long counts_per_rev = 4;
static const unsigned long tachometer_sample_interval = 100;
unsigned long tachometer_counter = 0;
unsigned long tachometer_counter_previous = 0;
unsigned long tachometer_time_previous = 0;

// State variables
float angular_velocity;
Ewma angular_velocity_filter(0.2);

// Plot
static const unsigned long plot_interval = 250;
unsigned long plot_time_previous = 0;


void tachometerInterruptHandler() {
  ++tachometer_counter;
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(tachometer_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), tachometerInterruptHandler, CHANGE);
  current_time = millis();
}

// the loop routine runs over and over again forever:
void loop() {
  unsigned long new_current_time = millis();

  // Blink
  if (new_current_time - blink_start_time > blink_period) {
    blink_state = !blink_state;
    digitalWrite(LED_BUILTIN, blink_state);
    blink_start_time = new_current_time;
  }

  // Read the potentiometer
  int potentiometer_value = constrain(analogRead(potentiometer_pin), potentiometer_floor, potentiometer_ceiling);

  // Set the motor speed
  uint8_t motor_pwm = map(potentiometer_value, potentiometer_floor, potentiometer_ceiling, 0, 255);
  analogWrite(motor_pin, motor_pwm);

  // Read the tacohmeter
  const unsigned long tachometer_time_delta = new_current_time - tachometer_time_previous;
  if (tachometer_time_delta > tachometer_sample_interval) {
    unsigned long tachometer_counter_current = tachometer_counter;
    unsigned long tachometer_counter_delta = tachometer_counter_current - tachometer_counter_previous;
    angular_velocity = 60.0 * 1000 * tachometer_counter_delta / counts_per_rev / tachometer_time_delta; // rpm
    angular_velocity_filter.filter(angular_velocity);
    tachometer_counter_previous = tachometer_counter_current;
    tachometer_time_previous = new_current_time;
  }

  // Plot the data
  if (new_current_time - plot_time_previous > plot_interval) {
    Serial.print(motor_pwm);
    Serial.print(", ");
    Serial.print(angular_velocity);
    Serial.print(", ");
    Serial.println(angular_velocity_filter.output);
    plot_time_previous = new_current_time;
  }

  current_time = new_current_time;
}
