/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

unsigned long current_time = 0;
unsigned long blink_period = 100;
bool blink_state = false;
unsigned long blink_start_time = 0;

static const uint8_t motor_pin = 5;

// Motor encoder
static const uint8_t encoder_pin = 2;
static const unsigned long counts_per_rev = 4;
static const unsigned long encoder_sample_interval = 100;
unsigned long encoder_counter = 0;
unsigned long encoder_counter_previous = 0;
unsigned long encoder_time_previous = 0;


void motorEncoderInterruptHandler() {
  ++encoder_counter;
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(encoder_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), motorEncoderInterruptHandler, CHANGE);
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
  int potentiometer_value = analogRead(A0);

  // Set the motor speed
  uint8_t motor_pwm = map(potentiometer_value, 0, 1023, 0, 255);
  analogWrite(motor_pin, motor_pwm);

  // Read the encoder
  const unsigned long encoder_time_delta = new_current_time - encoder_time_previous;
  if (encoder_time_delta > encoder_sample_interval) {
    unsigned long encoder_counter_current = encoder_counter;
    unsigned long encoder_counter_delta = encoder_counter_current - encoder_counter_previous;
    float angular_velocity = 60.0 * 1000 * encoder_counter_delta / counts_per_rev / encoder_time_delta; // rpm
    encoder_counter_previous = encoder_counter_current;
    encoder_time_previous = new_current_time;
    Serial.println(angular_velocity);
  }

  current_time = new_current_time;
}
