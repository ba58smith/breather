#include <Arduino.h>
#include <ESP32Encoder.h>
#include <heltec.h>
#include <ESP_FlexyStepper.h>
#include <elapsedMillis.h>

#define HOME 0
#define BPM_MULTIPLIER 30  // SWAG at this point
#define VOLUME_TO_MM_CONVERSION 33.3 // BAS: 33.3 is according to Forrest's spec document

int topcornerX = 1;
int topcornerY = 30;
int bottomcornerX = 128;
int bottomcornerY = 64;

/**
 * @brief Updates only the area of the OLED that displays the Volume
 */
void update_oled(float volume, uint8_t bpm) {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(topcornerX, topcornerY, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  String rpm_vol_str = String(bpm) + "       " + String(volume, 1);
  Heltec.display->drawString(10, 26, rpm_vol_str);
  Heltec.display->display();
  Serial.println("BPM: " + String(bpm) + "   Volume: " + String(volume, 1));
}

static bool led = LOW;
static bool bpm_knob_interrupt_fired = false;
static bool home_btn_interrupt_fired = false;
static bool volume_knob_interrupt_fired = false;
static bool pause_btn_interrupt_fired = false;
static bool limit_switch_interrupt_fired = false;

const uint8_t pause_btn_pin = 38;
const uint8_t home_btn_pin = 5;
const uint8_t bpm_CLK_pin = 36;
const uint8_t bpm_DT_pin = 37;
const uint8_t volume_CLK_pin = 23;
const uint8_t volume_DT_pin = 18;
const uint8_t motor_step_pin = 27;
const uint8_t motor_direction_pin = 14;
const uint8_t limit_switch_pin = 39;

// define all the ISR's and ISR callbacks
static IRAM_ATTR void bpm_knob_cb(void *arg) {
  digitalWrite(LED_BUILTIN, !led);
  bpm_knob_interrupt_fired = true;
}

static IRAM_ATTR void volume_knob_cb(void *arg) {
  digitalWrite(LED_BUILTIN, !led);
  volume_knob_interrupt_fired = true;
}

// BAS: check this interrupt inside the two while() loops, to execute an emergency stop?
void limit_switch_isr() {
  limit_switch_interrupt_fired = true;
}

ESP32Encoder bpm_knob(true, bpm_knob_cb);
ESP32Encoder volume_knob(true, volume_knob_cb);

/**
 * @brief Read the current value of the Volume knob, convert it to a float,
 * and return 1/10 of the value, so the display can be to 1 decimal. Always
 * use this function to read the knob, so that all volume values will be
 * consistent! 
 */
float get_volume_as_float() { // BAS: make all the values in this function come from #define statements
  float count = (float)volume_knob.getCount() / 10;
  if (count > 6.0)
  {
    count = 1.0;
    volume_knob.setCount(10);
  }
  else if (count < 1.0)
  {
    count = 6.0;
    volume_knob.setCount(60);
  }
  return count;
}

/**
 * @brief Read the current value of the BPM knob.. Always
 * use this function to read the knob, so that all BPM values will be
 * consistent! 
 */
uint8_t get_bpm_as_int() { // BAS: make all the values in this function come from #define statements
  uint8_t count = bpm_knob.getCount();
  if (count > 40)
  {
    count = 20;
    bpm_knob.setCount(20);
  }
  else if (count < 20)
  {
    count = 40;
    bpm_knob.setCount(40);
  }
  return count;
}

ESP_FlexyStepper stepper;
elapsedMillis bpm_timer = 0;

float motorStepsPerMillimeter = 25; // BAS: verified
float distanceToMoveInMillimeters = 76;  // just a starting value - is really controlled by volume knob
float speedInMillimetersPerSecond = 200; // just a starting value - controlled by bpm knob
float accelerationInMillimetersPerSecondPerSecond = 100; // just a starting value - controlled by bpm knob
float decelerationInMillimetersPerSecondPerSecond = 100; // just a starting value - controlled by bpm knob

float target_position_in_mm = VOLUME_TO_MM_CONVERSION; // sets the initial target, until the volume knob is turned
uint8_t bpm_desired = 1 * BPM_MULTIPLIER; // sets the initial BPM, until the BPM knob is turned

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(limit_switch_pin, INPUT_PULLUP);
  Serial.begin(115200);

  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  bpm_knob.attachSingleEdge(bpm_DT_pin, bpm_CLK_pin);
  // following line, along with a 0.1uF cap to GND, is for debouncing
  bpm_knob.setFilter(1023);
  bpm_knob.setCount(25); // set the initially-displayed BPM

  volume_knob.attachSingleEdge(volume_DT_pin, volume_CLK_pin);
  volume_knob.setFilter(1023);
  volume_knob.setCount(30); // set the initially-displayed volume (30 is displayed as 3.0)

  //attachInterrupt(limit_switch_pin, LOW);

  stepper.connectToPins(motor_step_pin, motor_direction_pin);
  stepper.setStepsPerMillimeter(motorStepsPerMillimeter);
  stepper.setSpeedInMillimetersPerSecond(speedInMillimetersPerSecond);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(accelerationInMillimetersPerSecondPerSecond);
  stepper.setDecelerationInMillimetersPerSecondPerSecond(decelerationInMillimetersPerSecondPerSecond);

  Heltec.begin(true, false, true);
  Heltec.display->clear();
  Heltec.display->display();
  Heltec.display->resetOrientation();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "BPM   |   Volume");
  Heltec.display->display();
  Heltec.display->setFont(ArialMT_Plain_24);
  update_oled(get_volume_as_float(), get_bpm_as_int());

  // Not working yet
  /* stepper.moveToHomeInSteps(char_directionToHome, f_speedInStepsPerSecond, long_maxDistanceToMoveInSteps,
                            int_homeLimitSwitchPin);
                            */
  /*
  Serial.println("Starting homing operation");
  if (!stepper.moveToHomeInSteps(-1, 20.0, (200.0 * 10.0), limit_switch_pin)) {
    for (int x = 0; x < 10; x++) {
      digitalWrite(LED_BUILTIN, led);
      led = !led;
      delay(100);
    }
  }
  */

  // BAS: Need code here to "home" the piston
  stepper.setCurrentPositionInMillimeters(HOME);
  stepper.setTargetPositionInMillimeters(get_volume_as_float() * 25.0);
} // setup;

void loop() {  
  if (volume_knob_interrupt_fired) {

    update_oled(get_volume_as_float(), get_bpm_as_int());
    target_position_in_mm = get_volume_as_float() * VOLUME_TO_MM_CONVERSION;
    stepper.setTargetPositionInMillimeters(target_position_in_mm);
    volume_knob_interrupt_fired = false;
  }

  if (bpm_knob_interrupt_fired) {
    update_oled(get_volume_as_float(), get_bpm_as_int());
    bpm_desired = get_bpm_as_int(); float desired_piston_speed = abs(target_position_in_mm) * 2.0 * bpm_desired  / 60;
    stepper.setSpeedInMillimetersPerSecond(desired_piston_speed);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(desired_piston_speed * desired_piston_speed);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(desired_piston_speed * desired_piston_speed);
    bpm_knob_interrupt_fired = false;
  }

  delay(200);
  
  // begine the inhale stroke  
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    bpm_timer = 0;
    stepper.processMovement();
  }
  delay(1500); // for testing when not connected to the stepper
  // save the value of bpm_timer immediately on exiting the while()
  uint32_t while_loop_time = bpm_timer;
  // calculate BPM from bpm_timer: ms_in_a_half_minute / while_loop_ms
  float calculated_bpm = 30000.0 / while_loop_time;
  Serial.println("inhale while() bpm_timer and calculated BPM: " + String(while_loop_time) + " / " + String(calculated_bpm, 1));
 
  // now reverse it for the exhale stroke
  // check the interrupts between the strokes for better updating of the OLED
  if (volume_knob_interrupt_fired) {

    update_oled(get_volume_as_float(), get_bpm_as_int());
    target_position_in_mm = get_volume_as_float() * VOLUME_TO_MM_CONVERSION;
    // do NOT setTargetPosition at this point - on the exhale stroke, targetPosition is always HOME (eventually TDC)
    volume_knob_interrupt_fired = false;
  }

  if (bpm_knob_interrupt_fired) {
    update_oled(get_volume_as_float(), get_bpm_as_int());
    bpm_desired = get_bpm_as_int(); float desired_piston_speed = abs(target_position_in_mm) * 2.0 * bpm_desired  / 60;
    stepper.setSpeedInMillimetersPerSecond(desired_piston_speed);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(desired_piston_speed * desired_piston_speed);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(desired_piston_speed * desired_piston_speed);
    bpm_knob_interrupt_fired = false;
  }

  delay(200);
  // BAS: be careful here - if the volume interrupt fired between strokes, target_position_in_mm will be a positive
  // number at this point because it came from the knob. When we start playing with homing, we may flip directionToHome,
  // so target_position_in_mm might need to be a negative number at this point, so we'd have to make it negative
  // inside if (volume_knob_interrupt_fired) {}, so when the next line executes, it would become positive.
  target_position_in_mm *= -1.0;
  stepper.setTargetPositionInMillimeters(HOME);
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    bpm_timer = 0;
    stepper.processMovement();
  }
  delay(1234); // for testing when not connected to the stepper
  while_loop_time = bpm_timer;
  calculated_bpm = 60000.0 / (while_loop_time * 2.0); // BAS: does ths round the way I want?
  Serial.println("exhale while_loop_time and calculated BPM: " + String(while_loop_time) + " / " + String(calculated_bpm, 1));

  //delay(100);
  // change directions for the next inhale
  target_position_in_mm *= -1.0;
  stepper.setTargetPositionInMillimeters(target_position_in_mm);
  
} // end loop()
