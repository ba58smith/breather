#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL_LOG_VERBOSE
#include "esp_log.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <heltec.h>
#include <ESP_FlexyStepper.h>
#include <elapsedMillis.h>

#define VOLUME_TO_MM_CONVERSION 33.3 // 33.3 is according to Forrest's spec document
#define ACTUAL_LENGTH_OF_SCREW_IN_MM 275.0 // this is the actual length
#define MAX_STROKE_LENGTH_IN_MM 250.0 // this is the actual distance
#define PAUSE 1
#define RESUME 0
#define ENABLE 1
#define DISABLE 2

// knob display min, max, and starting values
const float VOL_KNOB_MIN_VAL = 1.0;
const float VOL_KNOB_MAX_VAL = 7.0;
const uint16_t VOL_KNOB_START_VAL = 4;
const uint16_t BPM_KNOB_MIN_VAL = 10;
const uint16_t BPM_KNOB_MAX_VAL = 90;
const uint16_t BPM_KNOB_START_VAL = 10;
const float CO2_KNOB_MIN_VAL = 0.0;
const float CO2_KNOB_MAX_VAL = 3.0;
const uint16_t CO2_KNOB_START_VAL = 0;
const float RMV_MAX_VAL = 280.00;

// defines the region of the OLED to be updated with knob turns
int topcornerX = 1;
int topcornerY = 29;
int bottomcornerX = 128;
int bottomcornerY = 64;

// this is LED_BUILTIN. It's currently used in the isr's to give an immediate indication that the isr
// has been called, since the OLED won't update until the end of the next cycle.
// Can be removed once everything is working, to make the isr's even smaller
static bool led = LOW;

static bool bpm_knob_interrupt_fired = false;
static bool volume_knob_interrupt_fired = false;
static bool home_btn_interrupt_fired = false;
static bool pause_btn_interrupt_fired = false;
static uint8_t pause_btn_action = PAUSE; // other action is RESUME
static bool co2_enabled = false;
static uint8_t co2_btn_action = ENABLE; // other action is DISABLE
elapsedMillis co2_timer = 0.00;
bool co2_valve_opened = false;                                              

// define all pins to match Forrest's schematic
const uint8_t pause_btn_pin = 38; // the pushbutton on the bpm encoder
const uint8_t home_btn_pin = 35; // the pushbutton on the volume encoder
const uint8_t co2_btn_pin = 25; // the pushbutton on the co2 encoder
const uint8_t co2_valve_pin = 17; // opens / closes the co2 valve
const uint8_t bpm_CLK_pin = 36; // encoder A pins are CLK
const uint8_t bpm_DT_pin = 37; // encoder B pins are DT
const uint8_t volume_CLK_pin = 39;
const uint8_t volume_DT_pin = 34;
const uint8_t co2_CLK_pin = 32; // sets the timing of the introduction of co2 into the cycle
const uint8_t co2_DT_pin = 33;
const uint8_t motor_direction_pin = 27; // BAS: my schematic is wrong - it flips 26 and 27
const uint8_t motor_pulse_pin = 26;
const uint8_t limit_switch_data_pin = 13;

float motorStepsPerMillimeter = 25; // verified on Jim's test jig
float inhale_start = 0.0; // will be set by the homing operation

enum State_t {
    HOMING,
    IDLE,
    INHALE,
    EXHALE,
    CALIBRATE_HOME_SWITCH  // BAS: implement this
};

State_t state = HOMING; // the first thing it will do after setup()
float current_volume = 0.0;
uint32_t current_bpm = 0;
float current_co2_duration = 0.0;

/**
 * @brief Open the CO2 valve. Called at the beginning of each exhale stroke.
 * Valve will be closed by close_co2_valve() after current_co2_duration (which is a global variable)
 * return value can be used to determine if close_co2_valve() needs to be called.
 */

bool open_co2_valve() {
  if (co2_enabled && current_co2_duration > 0.0) {
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
 * @brief Close the CO2 valve. Called after current_co2_duration inside the stepper while() loop
 * (if co2_timer > current_co2_duration), and immediately if any of the 3 buttons are pushed.
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
  close_co2_valve(); // whether the button is in PAUSE or RESUME mode, this is still OK
  pause_btn_interrupt_fired = true;
}

static IRAM_ATTR void home_btn_isr() {
  digitalWrite(LED_BUILTIN, led);
  led = !led;
  close_co2_valve();
  home_btn_interrupt_fired = true;
}

static IRAM_ATTR void co2_btn_isr() {
  // BAS: remove the led stuff after testing is complete
  digitalWrite(LED_BUILTIN, led);
  led = !led;
  if (co2_btn_action == DISABLE) {
    close_co2_valve();
    co2_enabled = false;
    co2_btn_action = ENABLE; // set for next button push
  }
  else { // button means ENABLE
    co2_enabled = true;
    co2_btn_action = DISABLE; // set for next button push
  }
}

// Instantiate the stepper
ESP_FlexyStepper stepper;
// instantiate the encoders with their associated callback functions
ESP32Encoder bpm_knob(true, bpm_knob_cb);
ESP32Encoder volume_knob(true, volume_knob_cb);
ESP32Encoder co2_knob;

/**
 * @brief Updates only the area of the OLED that displays the BPM and Volume values
 */
void update_oled() {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(topcornerX, topcornerY, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  // BAS: add CO2 value and all switches to the display
  String values_str = String(current_bpm) + "      " + String(current_volume, 1) + "     " + String(current_co2_duration,2);
  Heltec.display->drawString(10, 26, values_str);
  Heltec.display->display();
  // BAS: for testing only - remove before final, to make the ISRs run faster
  // Serial.println("BPM: " + String(current_bpm) + "   Volume: " + String(current_volume, 1));
}

void update_homeing_oled() {} // first screen to show on boot. BAS: "Homing the machine", something like that.

void update_pasued_oled() {} // next screen to show after homeing. BAS: "Set desired valued, then Push START button", maybe?

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
  float count_as_float = ((float)(volume_knob.getCount()) / 10.0);
  if (count_as_float > VOL_KNOB_MAX_VAL) {
    count_as_float = VOL_KNOB_MAX_VAL;
    volume_knob.setCount((uint16_t)(VOL_KNOB_MAX_VAL * 10));
  }
  else if (count_as_float < VOL_KNOB_MIN_VAL) {
    count_as_float = VOL_KNOB_MIN_VAL;
    volume_knob.setCount((uint16_t)(VOL_KNOB_MIN_VAL * 10));
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
    count_as_int = BPM_KNOB_MAX_VAL;
    bpm_knob.setCount(BPM_KNOB_MAX_VAL);
  }
  else if (count_as_int < BPM_KNOB_MIN_VAL) {
    count_as_int = BPM_KNOB_MIN_VAL;
    bpm_knob.setCount(BPM_KNOB_MIN_VAL);
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
 * On the OLED), CO2 values are in seconds: 0.05 seconds, 1.03 seconds, 2.20 seconds, etc.
 * 
 * Note - since the encoders deal in ints, getCount() and setCount() do too.
 * So getCount() values are divided by 100 to convert, for example, 23 to 0.23 seconds,
 * and setCount() values are multiplied by 100, so 0.23 seconds becomes 23.
 */
float get_co2_duration_as_float() {
  float count_as_float = ((float)(co2_knob.getCount()) / 100.0);
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

/**
 * @brief Does exactly what Forrest's get_encoder_values() does, but does it by calling
 * some other functions, to make it a little easier to see what's going on. This will
 * still be used the same way.
 */

void get_encoder_values() {
  bool some_value_changed = false;
  float new_vol = get_volume_as_float();
  if (new_vol != current_volume) {
    current_volume = new_vol;
    some_value_changed = true;
  }

  uint8_t new_bpm = get_bpm_as_int();
  if (new_bpm != current_bpm) {
    current_bpm = new_bpm;
    some_value_changed = true;
  }

  float new_co2 = get_co2_duration_as_float();
  if (new_co2 != current_co2_duration) {
    current_co2_duration = new_co2;
    some_value_changed = true;
  }
  // Serial.println("BPM: " + (String)current_bpm + "  VOL: " + (String)current_volume + "  CO2: " + current_co2_duration);
  // make sure the volume * bpm is within the machine's limit
  if (current_volume * current_bpm > RMV_MAX_VAL) {
    Serial.println("Values too large: reduce Volume or BPM");
    // BAS: Update the OLED to give the same indication
  }
  else {    
    if (some_value_changed) {
      Serial.println("BPM: " + (String)current_bpm + "  VOL: " + (String)current_volume + "  CO2: " + current_co2_duration);
      update_oled();
    }
  }
}

/**
 * @brief calculate and set the speed, acceleration, and deceleration factors of 
 * the stepper based on the given volume and bpm, based on Forrest's formula.
 * 
 * @param volume - typically, get_volume_as_float()
 * @param bpm - typically, get_bpm_as_int()
 */
void set_speed_factors(float volume, uint16_t bpm) {
  float time = (60.0 / (float)bpm / 2.0); // time in seconds per cycle
  float distance = (volume * VOLUME_TO_MM_CONVERSION);
  float speed = (distance / time); // (TVL* MM per liter / seconds per cycle
  float accel = ((distance / pow((time / 2.0), 2.0) * 4.0)); // this is still up to 4% error
  stepper.setSpeedInMillimetersPerSecond(speed);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(accel); 
  stepper.setDecelerationInMillimetersPerSecondPerSecond(accel);
}

// ************************ SETUP() ********************************** //

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(limit_switch_data_pin, INPUT_PULLUP);
  pinMode(pause_btn_pin, INPUT_PULLUP);
  pinMode(home_btn_pin, INPUT_PULLUP);
  pinMode(co2_btn_pin, INPUT_PULLUP);
  pinMode(co2_valve_pin, OUTPUT);
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
  co2_knob.setCount(CO2_KNOB_START_VAL * 100); // set the initially-displayed CO2 duration (20 * 100; will be divided by 100 for display)

  attachInterrupt(co2_btn_pin, co2_btn_isr, LOW);
  attachInterrupt(pause_btn_pin, pause_btn_isr, LOW);
  attachInterrupt(home_btn_pin, home_btn_isr, LOW);

  Heltec.begin(true, false, true);
  Heltec.display->clear();
  Heltec.display->display();
  Heltec.display->resetOrientation();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "BPM  | Vol  | CO2");
  Heltec.display->display();
  // Heltec.display->setFont(ArialMT_Plain_24); I don't think we can use the big font anymore
  update_oled();

  stepper.connectToPins(motor_pulse_pin, motor_direction_pin);
  stepper.setStepsPerMillimeter(motorStepsPerMillimeter);
  stepper.startAsService(0);
  if (stepper.isStartedAsService()) {
    Serial.println("Stepper started in Core 0");
  }
  else {
    Serial.println("Stepper failed to start in Core 0");
  }
  delay(1000);
  
} // setup;

// ************************ LOOP() ************************** //

void loop() {

  switch (state) {

  case HOMING: { // will come to this first, because of "state = HOMING" in setup()
    Serial.println("Case HOMING");
    stepper.setSpeedInMillimetersPerSecond(250);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(150);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(75);
    if (!stepper.moveToHomeInMillimeters(-1, 40, ACTUAL_LENGTH_OF_SCREW_IN_MM, limit_switch_data_pin)) {
      Serial.println("Something wrong in moveToHomeInMillimeters()");
    }
    else {
    Serial.println("moveToHomeInMillimeters() was successful");
    // at this point, the piston should be just past the point where the limit switch has gone HIGH again, and
    // moveToHomeInMM() has set this position = to 0. Positions closer to the limit switch are negative, and
    // positions farther from the limit switch are positive.
    // the piston is now at a position that's the closest it should ever get to the limit switch again,
    // so now move it to the desired position for the start of each inhale stroke - MAX_STROKE_LENGTH_IN_MM farther.
    stepper.setTargetPositionInMillimeters(MAX_STROKE_LENGTH_IN_MM);
    while (!stepper.motionComplete()) {
      // BAS: monitor the PAUSE button here? Forrest says YES
      stepper.processMovement();
    }
    Serial.println("Should now be at the starting point for every inhale stroke");
    // now we should be at the point where every inhale stroke begins, and the point where every
    // exhale stroke returns to. Set it to 0, and save it as inhale_start, to be used by the exhale stroke.
    stepper.setCurrentPositionInMillimeters(0.0);
    inhale_start = 0.0;
    }
    state = IDLE;
    break;
  }  

  case IDLE: { //  Idle - getting knob values and waiting for the START button
    Serial.println("Case IDLE");
    // BAS: design the idle oled and display it
    while (state == IDLE) {
      if (pause_btn_interrupt_fired) { // any push of the pause_btn acts as the "START" button in this situation
        state = INHALE;
        pause_btn_action = PAUSE; // set for the next time the button is pushed
        pause_btn_interrupt_fired = false;
        break;
      }
      if (home_btn_interrupt_fired) {
        state = HOMING;
        home_btn_interrupt_fired = false;
        break;
      }
      get_encoder_values();
      delay(100);
    }
    break;
  }  

  case INHALE: {
    Serial.println("Case INHALE");
    // begin the inhale stroke - show oled display
    set_speed_factors(current_volume, current_bpm);
    stepper.setTargetPositionInMillimeters(-1 * current_volume * VOLUME_TO_MM_CONVERSION);
    pause_btn_action = PAUSE; // so the next button press means PAUSE
    while (!stepper.motionComplete() && state == INHALE) {
      
      // if the co2_duration is > the time of the exhale stroke, the valve may still be open
      if (co2_valve_opened) {
        if (co2_timer > current_co2_duration) {
          close_co2_valve();
          co2_valve_opened = false;
        }
      }
           
      if (pause_btn_interrupt_fired) {
        if (pause_btn_action == PAUSE) {
          // co2 valve was already closed, in the isr for this button
          pause_btn_action = RESUME; // set for the next button press
          stepper.emergencyStop(true); // "true" means "wait for a call to stepper.releaseEmergencyStop()"
        }
        else { // pause button press means RESUME
          pause_btn_action = PAUSE; // set for the next button press
          stepper.releaseEmergencyStop();
        }
        pause_btn_interrupt_fired = false;
      }
      
      if (home_btn_interrupt_fired) {
         // co2 valve has already been closed, in the isr for this button
         stepper.emergencyStop(false); // "false" means "don't wait for a call to stepper.releaseEmergencyStop()"
         state = HOMING;
         home_btn_interrupt_fired = false;
         break;
      }

      get_encoder_values();
      
      delay(100);
    }
    state = EXHALE;
    break;
  }

  case EXHALE: { // move back to the inhale_start position.
    Serial.println("Case EXHALE");
    stepper.setTargetPositionInMillimeters(inhale_start);
    pause_btn_action = PAUSE; // so the next button press means PAUSE
    co2_valve_opened = open_co2_valve(); // true if valve is enabled and duration is > 0.0
    while (!stepper.motionComplete() && state == EXHALE) {
      
      if (co2_valve_opened) {
        if (co2_timer > current_co2_duration) {
          close_co2_valve();
          co2_valve_opened = false;
        }
      }
      
      if (pause_btn_interrupt_fired) {
        if (pause_btn_action == PAUSE) {
          // co2 valve was already closed, in the isr for this button
          pause_btn_action = RESUME; // set for the next button press
          stepper.emergencyStop(true); // "true" means "wait for a call to stepper.releaseEmergencyStop()"
        }
        else { // pause button press means "resume"
          pause_btn_action = PAUSE; // set for the next button press
          stepper.releaseEmergencyStop();
        }
        pause_btn_interrupt_fired = false;
      }
      
      if (home_btn_interrupt_fired) {
         // co2 valve has already been closed, in the isr for this button
         state = HOMING;
         home_btn_interrupt_fired = false;
         stepper.emergencyStop(false); // "false" means "don't wait for a call to stepper.releaseEmergencyStop()"
         break;
      }

      get_encoder_values();
      
      delay(100);
    }
    state = INHALE;
    break;
  } // end case 4:
  }  // end switch
} // end loop()
