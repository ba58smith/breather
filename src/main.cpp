#include <Arduino.h>
#include <ESP32Encoder.h>
#include <heltec.h>
#include <ESP_FlexyStepper.h>
#include <elapsedMillis.h>

#define VOLUME_TO_MM_CONVERSION 33.3 // 33.3 is according to Forrest's spec document
#define ACTUAL_LENGTH_OF_SCREW_IN_MM 160.0 // BAS: this is from Jim's working code
#define MAX_STROKE_LENGTH_IN_MM 150.0 // BAS: also from Jim's code: the actual distance
                                      // necessary to get the piston from the "homed" position to the desired 
                                      // starting point for every inhale cycle.
float home_in_mm = 0.0; // will be set by the homing operation, which should set it to 0.0

// knob display min, max, and starting values
const float VOL_KNOB_MIN_VAL = 1.0;
const float VOL_KNOB_MAX_VAL = 6.0;
const uint16_t VOL_KNOB_START_VAL = 3;
const uint16_t BPM_KNOB_MIN_VAL = 10;
const uint16_t BPM_KNOB_MAX_VAL = 30;
const uint16_t BPM_KNOB_START_VAL = 20;
const float CO2_KNOB_MIN_VAL = 0.0;
const float CO2_KNOB_MAX_VAL = 3.0;
const uint16_t CO2_KNOB_START_VAL = 0;

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
static bool pause_btn_interrupt_fired = false;
static bool pause_btn_action_is_pause = true; // true = pause, false = resume. This switches with each button press, in the isr.
                                               // start with true here, so when it's first pushed, to start the first inhale cycle,
                                               // it becomes false (resume) before being acted on.
static bool home_btn_interrupt_fired = false;

static float co2_duration = 0.0;
static bool co2_enabled = false;
static bool co2_btn_action_is_disable = false; // true = disable, false = enable. This switches with each button press, in the isr.
                                               // start with false here, so when it's first pushed, it becomes true before being acted on.
elapsedMillis co2_timer = 0.00;                                                

// define all pins to match Forrest's schematic
const uint8_t pause_btn_pin = 38; // the pushbutton on the bpm encoder
const uint8_t home_btn_pin = 35; // the pushbutton on the volume encoder
const uint8_t co2_btn_pin = 25; // the pushbutton on the co2 encoder
const uint8_t co2_valve_pin = 17; // opens / closes the co2 valve
const uint8_t bpm_CLK_pin = 36; // encoder A pins are CLK
const uint8_t bpm_DT_pin = 37; // encoder B pins are DT
const uint8_t volume_CLK_pin = 39;
const uint8_t volume_DT_pin = 34;
const uint8_t co2_CLK_pin = 32; // sets the length of time the CO2 valve is open
const uint8_t co2_DT_pin = 33;
const uint8_t motor_pulse_pin = 27;
const uint8_t motor_direction_pin = 14;
const uint8_t limit_switch_data_pin = 12;
const uint8_t limit_switch_led_pin = 13;

/**
 * @brief Open the CO2 valve. Called at the beginning of each exhale stroke.
 * Valve will be closed by close_co2_valve() after co2_duration (which is a global variable)
 * 
 * return value can be used to determine if close_co2_valve() needs to be called.
 */

bool open_co2_valve() {
  if (co2_enabled && co2_duration > 0.0) {
    // start the elapsedMillis timer
    co2_timer = 0;
    // open the valve
    digitalWrite(co2_valve_pin, LOW); // Forrest always makes things active LOW
    return true;
  }
  else { // not supposed to open the valve
    return false;
  }
}

/**
 * @brief Close the CO2 valve. Called after co2_duration inside the stepper while() loop (IF co2_timer > co2_duration),
 * and immediately if the co2_btn_pin goes LOW (which is connected to the co2_btn_isr, which calls this function)
 */

void close_co2_valve() {
  digitalWrite(co2_valve_pin, HIGH);
}


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

// define the button push isrs

static IRAM_ATTR void pause_btn_isr() {
  digitalWrite(LED_BUILTIN, led);
  led = !led;
  close_co2_valve(); // don't wait for the pause_btn_interrupt_fired flag to be evaluated in loop() - close it NOW.
  pause_btn_action_is_pause = !pause_btn_action_is_pause; // flip the action btwn "pause" and "resume" BEFORE setting the interrupt flag
  pause_btn_interrupt_fired = true;
}

static IRAM_ATTR void home_btn_isr() {
  digitalWrite(LED_BUILTIN, led);
  led = !led;
  close_co2_valve(); // don't wait for the home_btn_interrupt_fired flag to be evaluated in loop() - close it NOW.
  home_btn_interrupt_fired = true;
}

static IRAM_ATTR void co2_btn_isr() {
  // BAS: remove the led stuff after testing is complete
  digitalWrite(LED_BUILTIN, led);
  led = !led;
  co2_btn_action_is_disable = !co2_btn_action_is_disable; // flip the action btwn "disable" and "enable" BEFORE acting on the button push
  if (co2_btn_action_is_disable) { // button press at this time means "disable"
    // close the co2 valve immediately
    digitalWrite(co2_valve_pin, HIGH);
    // disable it until a future button push
    co2_enabled = false;
  }
  else { // button means "enable"
    // enable co2 until a future button push, but don't open the valve at this point
    co2_enabled = true;
  }
}


// instantiate the encoders with their associated callback functions
ESP32Encoder bpm_knob(true, bpm_knob_cb);
ESP32Encoder volume_knob(true, volume_knob_cb);
// Forrest's code doesn't use the callbacks, I don't think
ESP32Encoder co2_knob;

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
 * make sure it's within the defined minimum and maximum values for this knob,
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

/**
 * @brief Read the current value of the CO2 knob, convert it to a float,
 * make sure it's within the defined minimum and maximum values for this knob,
 * and return 1/100 of the value, so the display will be to 2 decimals (0.03 seconds). Always
 * use this function to read the knob, so that all CO2 values will be
 * consistent!
 * 
 * To the user (on the LED), CO2 values are in seconds: 0.05 seconds, 1.03 seconds, 2.20 seconds, etc.
 * 
 * Note - since the encoders deal in ints, getCount() and setCount() do too.
 * So getCount() values are divided by 100 to convert, for example, 20 to 0,20 seconds,
 * and setCount() values are multiplied by 100, so 0.20 seconds becomes 20.
 */
float get_co2_duration_as_float() {
  float count_as_float = (float)(co2_knob.getCount() / 100);
  if (count_as_float > CO2_KNOB_MAX_VAL) {
    count_as_float = CO2_KNOB_MIN_VAL;
    volume_knob.setCount((uint16_t)(CO2_KNOB_MIN_VAL * 100));
  }
  else if (count_as_float < CO2_KNOB_MIN_VAL) {
    count_as_float = CO2_KNOB_MAX_VAL;
    volume_knob.setCount((uint16_t)(CO2_KNOB_MAX_VAL * 100));
  }
  return count_as_float;
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
  volume_knob.setCount(VOL_KNOB_START_VAL * 10); // set the initially-displayed Volume (3 * 10; will be divided by 10 for display)

  co2_knob.attachSingleEdge(co2_DT_pin, co2_CLK_pin);
  co2_knob.setFilter(1023);
  co2_knob.setCount(CO2_KNOB_START_VAL * 100); // set the initially-displayed CO2 timing (0 * 100; will be divided by 100 for display)

  attachInterrupt(co2_btn_pin, co2_btn_isr, LOW);
  attachInterrupt(pause_btn_pin, pause_btn_isr, LOW);
  attachInterrupt(home_btn_pin, home_btn_isr, LOW);

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

  // at this point, the piston should be just past the point where the limit switch has gone HIGH again, and
  // moveToHomeInMM() has set this position = to 0. Positions closer to the limit switch are positive, and
  // positions farther from the limit switch are negative.
  // for safety, move the piston another inch away from the limit switch
  stepper.setTargetPositionInMillimeters(-25.4);
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }
  Serial.println("Should be 1 inch past the 'homed' position");

  // the piston is now at a position that's the closest it should ever get to the limit switch again,
  // so now move it to the desired position for the start of each inhale stroke - MAX_STROKE_LENGTH_IN_MM farther.

  // NOTE: there's no real reason to do this move, and the previous move, separately. I did it that way
  // for now just to illustrate what's happening
  stepper.setTargetPositionRelativeInMillimeters(-1 * MAX_STROKE_LENGTH_IN_MM);
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }
  Serial.println("Should now be at the starting point for every inhale stroke");

  // now we should be at the point where every inhale stroke begins, and the point where every
  // exhale stroke returns to. Set it to 0, and save it as home_in_mm, to be used by the exhale stroke.
  stepper.setCurrentPositionInMillimeters(0);
  home_in_mm = 0.0;

  // read the current value of the volume encoder and set it as the target position for the first inhale stroke
  stepper.setTargetPositionInMillimeters(get_volume_as_float() * VOLUME_TO_MM_CONVERSION);
  update_oled(get_volume_as_float(), get_bpm_as_int());
  set_speed_factors(get_volume_as_float(), get_bpm_as_int());

} // setup;

void loop() {  
  
  if (volume_knob_interrupt_fired) {
    float current_volume = get_volume_as_float();
    uint16_t current_bpm = get_bpm_as_int();
    update_oled(current_volume, current_bpm);
    set_speed_factors(current_volume, current_bpm);
    target_position_in_mm = current_volume * VOLUME_TO_MM_CONVERSION;
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

  // BAS: I think this needs to be inside the while() that processes the movement. See the example.
  // It ALSO needs to be looked at by the code that controls the "state machine", because if this button
  // is pushed while the state is STATE_IDLE, it needs to move the state to STATE_INHALE - if the value 
  // of pause_btn_action_is_pause is false (resume) at that time.
  // OR - when IDLE, "pause" doesn't really make sense. So, if STATE_IDLE when this button is pushed,
  // regardless of the value of pause_btn_action_is_pause, change state to STATE_INHALE and set the
  // value of pause_btn_action_is_pause to false (resume), so the NEXT time the button is pushed, it will
  // flip to true (pause), so that emergencyStop() will be called.
    if (pause_btn_interrupt_fired) {
    if (pause_btn_action_is_pause) { // button press at this time means "pause"
      // co2 valve was already closed, in the isr for this button
      stepper.emergencyStop(true); // "true" means "wait for a call to stepper.releaseEmergencyStop()"
    }
    else { // button means "resume"
      stepper.releaseEmergencyStop(); // Is this right? Will that just resume where we left off? I think so.
    }
  }

  // BAS: I think this needs to be inside the while() that processes the movement. See the example.
  if (home_btn_interrupt_fired) {
    // co2 valve was already closed, in the isr for this button
    // do not change the status of co2_enabled flag - if it was enabled, it should still be enabled.
    stepper.emergencyStop(false); // "false" means "don't wait for a call to stepper.releaseEmergencyStop()" - is this correct?
    // go through the entire homing process, like at startup (which needs to be in a function)
  }

  delay(200);
  
  // begine the inhale stroke  
  while (!stepper.motionComplete()) { // direction == 0 and currentPostion == targePosition
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
  // now the piston is back at the start of a new inhale cycle, so set the target for that cycle
    stepper.setTargetPositionInMillimeters(target_position_in_mm);
  
} // end loop()
