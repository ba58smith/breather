#include <Arduino.h>
#include <ESP32Encoder.h>
#include <heltec.h>
#include <ESP_FlexyStepper.h>
#include <elapsedMillis.h>

#define VOLUME_TO_MM_CONVERSION 33.3 // 33.3 is according to Forrest's spec document
#define ACTUAL_LENGTH_OF_SCREW_IN_MM 300.0 // BAS: this is a guess - make it the actual length!
#define MAX_STROKE_LENGTH_IN_MM 254.0 // BAS: 10 inches - just a guess! Need to change to the actual distance
                                      // necessary to get the piston from the "homed" position to the desired 
                                      // starting point for every inhale cycle.
float home_in_mm = 0.0; // will be set by the homing operation

// knob display min, max, and starting values
const float VOL_KNOB_MIN_VAL = 1.0;
const float VOL_KNOB_MAX_VAL = 6.0;
const uint16_t VOL_KNOB_START_VAL = 3;
const uint16_t BPM_KNOB_MIN_VAL = 10;
const uint16_t BPM_KNOB_MAX_VAL = 30;
const uint16_t BPM_KNOB_START_VAL = 20;

// defines the region of the OLED to be updated with knob turns
int topcornerX = 1;
int topcornerY = 30;
int bottomcornerX = 128;
int bottomcornerY = 64;

// this is LED_BUILTIN. It's currently used in the isr's to give an immediate indication that the isr
// has been called, since the OLED won't update until the end of the next cycle.
// Can be removed once everything is working, to make the isr's even smaller
static bool led = LOW;

static bool bpm_knob_interrupt_fired = false;
static bool volume_knob_interrupt_fired = false;

// not yet implented:
static bool home_btn_interrupt_fired = false;
static bool pause_btn_interrupt_fired = false;

// define all pins to match Forrest's schematic
const uint8_t pause_btn_pin = 38; // the pushbutton on the bpm encoder
const uint8_t home_btn_pin = 35; // the pushbutton on the volume encoder
const uint8_t co2_btn_pin = 25; // the pushbutton on the co2 encoder
const uint8_t bpm_CLK_pin = 36; // encoder A pins are CLK
const uint8_t bpm_DT_pin = 37; // encoder B pins are DT
const uint8_t volume_CLK_pin = 39;
const uint8_t volume_DT_pin = 34;
const uint8_t co2_CLK_pin = 32; // sets the timing of the introduction of co2 into the cycle
const uint8_t co2_DT_pin = 33;
const uint8_t motor_pulse_pin = 27;
const uint8_t motor_direction_pin = 14;
const uint8_t limit_switch_data_pin = 12; // BAS: make sure this is the DATA line of the limit switch
const uint8_t limit_switch_led_pin = 13;

// define the knob turn callbacks
static IRAM_ATTR void bpm_knob_cb(void *arg) {
  digitalWrite(LED_BUILTIN, led);
  led = !led;
  bpm_knob_interrupt_fired = true;
}

static IRAM_ATTR void volume_knob_cb(void *arg) {
  digitalWrite(LED_BUILTIN, led);
  led = !led;
  volume_knob_interrupt_fired = true;
}

// instantiate the encoders with their associated callback functions
ESP32Encoder bpm_knob(true, bpm_knob_cb);
ESP32Encoder volume_knob(true, volume_knob_cb);

/**
 * @brief Updates only the area of the OLED that displays the BPM and Volume values
 */
void update_oled(float volume, uint8_t bpm) {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(topcornerX, topcornerY, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  String rpm_vol_str = String(bpm) + "       " + String(volume, 1);
  Heltec.display->drawString(10, 26, rpm_vol_str);
  Heltec.display->display();
  // BAS: for testing only - remove before final, to make the ISRs run faster
  Serial.println("BPM: " + String(bpm) + "   Volume: " + String(volume, 1));
}

/**
 * @brief Read the current value of the Volume knob, convert it to a float,
 * make sure it's within the define minimum and maximum values for this knob,
 * and return 1/10 of the value, so the display can be to 1 decimal. Always
 * use this function to read the knob, so that all volume values will be
 * consistent!
 * 
 * Note - since the encoders deal in ints, getCount() and setCount() do too.
 * So getCount() values are divided by 10 to convert, for example, 30 to 3.0,
 * and setCount() values are multiplied by 10, so 3.0 becomes 30.
 */
float get_volume_as_float() {
  float count_as_float = (float)(volume_knob.getCount() / 10);
  if (count_as_float > VOL_KNOB_MAX_VAL) {
    count_as_float = VOL_KNOB_MIN_VAL;
    volume_knob.setCount((uint16_t)(VOL_KNOB_MIN_VAL * 10));
  }
  else if (count_as_float < VOL_KNOB_MIN_VAL) {
    count_as_float = VOL_KNOB_MAX_VAL;
    volume_knob.setCount((uint16_t)(VOL_KNOB_MAX_VAL * 10));
  }
  return count_as_float;
}

/**
 * @brief Read the current value of the BPM knob.. Always
 * use this function to read the knob, so that all BPM values will be
 * consistent! 
 */
uint8_t get_bpm_as_int() {
  uint8_t count_as_int = bpm_knob.getCount();
  if (count_as_int > BPM_KNOB_MAX_VAL) {
    count_as_int = BPM_KNOB_MIN_VAL;
    bpm_knob.setCount(BPM_KNOB_MIN_VAL);
  }
  else if (count_as_int < BPM_KNOB_MIN_VAL) {
    count_as_int = BPM_KNOB_MAX_VAL;
    bpm_knob.setCount(BPM_KNOB_MAX_VAL);
  }
  return count_as_int;
}

// Instantiate the stepper
ESP_FlexyStepper stepper;


/**
 * @brief calculate and set the speed, acceleration, and deceleration factors of 
 * the stepper based on the given volume and bpm, based on Forrest's formula.
 * 
 * @param volume - typically, get_volume_as_float()
 * @param bpm - typically, get_bpm_as_int()
 */
void set_speed_factors(float volume, uint16_t bpm) {
  float desired_speed = volume * VOLUME_TO_MM_CONVERSION * 2.0 * bpm / 60; // Forrest's formula
  stepper.setSpeedInMillimetersPerSecond(desired_speed);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(desired_speed * desired_speed);
  stepper.setDecelerationInMillimetersPerSecondPerSecond(desired_speed * desired_speed);
}

float motorStepsPerMillimeter = 25; // verified on Jim's test jig

float target_position_in_mm = VOL_KNOB_START_VAL * VOLUME_TO_MM_CONVERSION; // sets the initial target, until the volume knob is turned


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(limit_switch_data_pin, INPUT_PULLUP);
  Serial.begin(115200);

  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  bpm_knob.attachSingleEdge(bpm_DT_pin, bpm_CLK_pin);
  bpm_knob.setFilter(1023);
  bpm_knob.setCount(BPM_KNOB_START_VAL); // set the initially-displayed BPM

  volume_knob.attachSingleEdge(volume_DT_pin, volume_CLK_pin);
  volume_knob.setFilter(1023);
  volume_knob.setCount(VOL_KNOB_START_VAL * 10); // set the initially-displayed Volume (3 * 10; will be divide by 10 for display)

  //attachInterrupt(limit_switch_data_pin, LOW);

  stepper.connectToPins(motor_pulse_pin, motor_direction_pin);
  stepper.startAsService(); // with no param, defaults to 1, which is the second core, which is what we want
  stepper.setStepsPerMillimeter(motorStepsPerMillimeter);
  set_speed_factors(get_volume_as_float(), get_bpm_as_int());

  Heltec.begin(true, false, true);
  Heltec.display->clear();
  Heltec.display->display();
  Heltec.display->resetOrientation();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "BPM   |   Volume");
  Heltec.display->display();
  Heltec.display->setFont(ArialMT_Plain_24);
  update_oled(get_volume_as_float(), get_bpm_as_int());

  /* 
  These are the parameters to moveToHomeInMillimeters:
  stepper.moveToHomeInMillimeters(signed char directionTowardHome,
                                  float speedInMillimetersPerSecond, 
                                  long maxDistanceToMoveInMillimeters,
                                  int homeLimitSwitchPin)      */
  Serial.println("Starting homing operation");
  // BAS: if the piston moves the wrong direction at startup, change the first parameter below to -1
  bool homing_successful = stepper.moveToHomeInMillimeters(1, 15, ACTUAL_LENGTH_OF_SCREW_IN_MM, limit_switch_data_pin);
  Serial.println("Homing successful? " + String(homing_successful));

  // at this point, the piston should be just past the point where the limit switch has gone HIGH again.
  // for safety, move the piston another inch or so away from the limit switch
  stepper.setTargetPositionRelativeInMillimeters(25.4); // BAS: if piston moves towards limit switch, make this NEGATIVE
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }
  Serial.println("Should be 1 inch past the 'homed' position");

  // the piston is now at a position that's the closest it should ever get to the limit switch again,
  // so now move it to the desired position for the start of each inhale stroke - MAX_STROKE_LENGTH_IN_MM farther.

  // NOTE: there's no real reason to do this move, and the previous move, separately. I did it that way
  // for now just to illustrate what's happening
  stepper.setTargetPositionRelativeInMillimeters(MAX_STROKE_LENGTH_IN_MM); // BAS: if piston moves towards limit switch, make this NEGATIVE
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }
  Serial.println("Should now be at the starting point for every inhale stroke");

  // now we should be at the point where every inhale stroke begins, and the point where every
  // exhale stroke returns to. Save it, to be used by the exhale stroke.
  home_in_mm = stepper.getCurrentPositionInMillimeters();

  // read the current value of the volume encoder and set it as the target position for the first inhale stroke
  stepper.setTargetPositionInMillimeters(home_in_mm + (get_volume_as_float() * VOLUME_TO_MM_CONVERSION));
  update_oled(get_volume_as_float(), get_bpm_as_int());
  set_speed_factors(get_volume_as_float(), get_bpm_as_int());

} // setup;

void loop() {  
  
  if (volume_knob_interrupt_fired) {
    float current_volume = get_volume_as_float();
    uint16_t current_bpm = get_bpm_as_int();
    update_oled(current_volume, current_bpm);
    set_speed_factors(current_volume, current_bpm);
    target_position_in_mm = home_in_mm + (current_volume * VOLUME_TO_MM_CONVERSION);
    stepper.setTargetPositionInMillimeters(target_position_in_mm);
    volume_knob_interrupt_fired = false;
  }

  if (bpm_knob_interrupt_fired) {
    float current_volume = get_volume_as_float();
    uint16_t current_bpm = get_bpm_as_int();
    update_oled(current_volume, current_bpm);
    set_speed_factors(current_volume, current_bpm);
    bpm_knob_interrupt_fired = false;
  }

  delay(200);
  
  // begine the inhale stroke  
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }
  
  // now begin the exhale stroke
  delay(200);
  stepper.setTargetPositionInMillimeters(home_in_mm);
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }
  // now the piston is back at the start of a new inhale cycle
    stepper.setTargetPositionInMillimeters(target_position_in_mm);
  
} // end loop()
