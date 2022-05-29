#define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUM_LEVEL_LOG_VERBOSE
#include "esp_log.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <EEPROM.h>
#include <heltec.h>
#include <ESP_FlexyStepper.h>
#include <elapsedMillis.h>

#define VOLUME_TO_MM_CONVERSION 33.3 // 33.3 is according to spec document
#define ACTUAL_LENGTH_OF_SCREW_IN_MM 275.0 // this is the actual length
int16_t MAX_STROKE_LENGTH_IN_MM  = 225; // get from EEPROM if it's there; if not, use this. Used in HOMING, set in CALIBRATE
#define HUGE_CALIB_MOVE_IN_MM 200 // initial move away from limit switch in CALIBRATE
#define EEPROM_SIZE 10

// knob display min, max, and starting values
const float TVL_KNOB_MIN_VAL = 1.0;
const float TVL_KNOB_MAX_VAL = 7.0;
const uint16_t TVL_KNOB_START_VAL = 4;
const uint16_t RMV_KNOB_MIN_VAL = 10;
const uint16_t RMV_KNOB_MAX_VAL = 90;
const uint16_t RMV_KNOB_START_VAL = 10;
const float CO2_KNOB_MIN_VAL = 0.0;
const float CO2_KNOB_MAX_VAL = 3.0;
const uint16_t CO2_KNOB_START_VAL = 0;
const float RMV_MAX_VAL = 280.00;

// defines the region of the OLED to be updated with knob turns
int topcornerX = 1;
int topcornerY = 20;
int bottomcornerX = 128;
int bottomcornerY = 64;

enum Btn_Action_t {
    PAUSE,
    RESUME,
    ENABLE,
    DISABLE,
    NO_ACTION
};

enum State_t {
    HOMING,
    IDLE,
    INHALE_NEXT,
    INHALE,
    EXHALE,
    FAILED_HOMING,
    CALIBRATE
};

bool tvl_btn_interrupt_fired = false;
bool rmv_btn_interrupt_fired = false;
Btn_Action_t pause_btn_action = RESUME; // PAUSE or RESUME - start w/ RESUME (at end of HOMING)
bool emx_stop_in_effect = false;
bool co2_enabled = false;
Btn_Action_t co2_btn_action = ENABLE; // ENABLE or DISABLE or NO_ACTION (during calibration)
elapsedMillis co2_timer = 0.00;
bool co2_valve_opened = false;    
bool btn_state_changed = false; // triggers update_oled_values()                                        

// define all pins
const uint8_t rmv_btn_pin = 5; // the pushbutton on the rmv encoder //BAS: s/b 38 - I have to use 5 b/c of a hardware problem
const uint8_t tvl_btn_pin = 35; // the pushbutton on the tvl encoder
const uint8_t co2_btn_pin = 19; // the pushbutton on the co2 encoder
const uint8_t co2_valve_pin = 17; // opens / closes the co2 valve
const uint8_t rmv_A_pin = 37; // encoder A pins are CLK
const uint8_t rmv_B_pin = 36; // encoder B pins are DT
const uint8_t tvl_A_pin = 34;
const uint8_t tvl_B_pin = 39;
const uint8_t co2_A_pin = 33; // sets the timing of the introduction of co2 into the cycle
const uint8_t co2_B_pin = 32;
const uint8_t motor_direction_pin = 26;
const uint8_t motor_pulse_pin = 27;
const uint8_t limit_switch_data_pin = SCK; //18

float motorStepsPerMillimeter = 25; // verified on Jim's test jig
float inhale_start = 0.0; // after HOMING or CALIBRATE, it's 0.0, and it's not used until EXHALE

State_t state; // set at the very end of setup()
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
    digitalWrite(co2_valve_pin, LOW);
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

// define the button push isrs
static IRAM_ATTR void pause_btn_isr() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 1000 ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 1000)
  {
  close_co2_valve(); // whether the pause_btn is in PAUSE or RESUME mode, this is still OK
  rmv_btn_interrupt_fired = true;
  }
  last_interrupt_time = interrupt_time;
}

static IRAM_ATTR void home_btn_isr() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 1000 ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 1000)
  {
  Serial.println("home_btn_isr fired");
  close_co2_valve();
  tvl_btn_interrupt_fired = true;
  }
  last_interrupt_time = interrupt_time;
}

static IRAM_ATTR void co2_btn_isr() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 1000 ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 1000) {
    if (co2_btn_action == DISABLE) {
      close_co2_valve();
      co2_enabled = false;
      co2_btn_action = ENABLE; // set for next button push
    }
    else { // button means ENABLE
      co2_enabled = true;
      co2_btn_action = DISABLE; // set for next button push
    }
    btn_state_changed = true;
  }
  last_interrupt_time = interrupt_time;
}

// Instantiate the stepper
ESP_FlexyStepper stepper;
// instantiate the encoders
ESP32Encoder rmv_knob;
ESP32Encoder tvl_knob;
ESP32Encoder co2_knob;

/**
 * @brief Draws the "header" for the main screen that display values and states.
 */

void draw_oled_header() {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(0, 0, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "TVL | RMV | CO2");
  Heltec.display->display();
}

/**
 * @brief Updates only the area of the OLED that displays all knob values and
 * button states - but not the top line: "TVL | RMV | CO2".
 */
void update_oled_values() {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(topcornerX, topcornerY, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  Heltec.display->setFont(ArialMT_Plain_16);
  String display_str = String(current_volume, 1) + "      " + String(current_bpm) + "      " + String(current_co2_duration,2);
  Heltec.display->drawString(4, 20, display_str);
  Heltec.display->setFont(ArialMT_Plain_10);
  display_str = "RMV btn push will: " + String(pause_btn_action == PAUSE ? "PAUSE" : "RESUME");
  Heltec.display->drawString(0, 41, display_str);
  display_str = "CO2: " + String(co2_enabled ? "ENABLED" : "NOT ENABLED");
  Heltec.display->drawString(0, 54, display_str);
  Heltec.display->display();
}

/**
 * @brief Updates screen to indicate the machine is Homing, and no value changes are displayed
 * during this process.
 */
void update_oled_homing() {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(topcornerX, topcornerY, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  Heltec.display->setFont(ArialMT_Plain_16);
  String display_str = "    HOMING:";
  Heltec.display->drawString(10, 20, display_str);
  display_str = "   Please wait...";
  Heltec.display->drawString(0, 41, display_str);
  Heltec.display->display();
}

/**
 * @brief Updates screen to indicate homing failed.
 */
void update_oled_failed_homing() {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(0, 0, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  Heltec.display->setFont(ArialMT_Plain_10);
  String display_str = "Homing failed, piston";
  Heltec.display->drawString(1, 20, display_str);
  display_str = "position unknown. Check";
  Heltec.display->drawString(1, 30, display_str);
  display_str = "wiring or try HOME button.";
  Heltec.display->drawString(1, 40, display_str);
  Heltec.display->display();
}

/**
 * @brief Updates screen to indicate that the RMV * TVL selected on
 * the knobs exceeds the limits of RMV.
 */
void update_oled_rmv_max() {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(topcornerX, topcornerY, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  Heltec.display->setFont(ArialMT_Plain_10);
  String display_str = "RMV * TVL is";
  Heltec.display->drawString(0, 20, display_str);
  display_str = "> max RMV. Lower 1";
  Heltec.display->drawString(0, 30, display_str);
  display_str = "or both values.";
  Heltec.display->drawString(0, 40, display_str);
  Heltec.display->display();
}

/**
 * @brief Updates the screen for the various phases of Calibration
 */

void update_oled_calibrate(String header = "CALIBRATE HOME SWTCH", String ln_2 = "TVL knob moves 1mm", String ln_3 = "CW = away from you", 
                           String ln_4 = "CCW = towards you", String ln_5 = "Push CO2 when done") {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(0, 0, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE);
  Heltec.display->setFont(ArialMT_Plain_10);
  String display_str = header;
  Heltec.display->drawString(0, 0, display_str);
  display_str = ln_2;
  Heltec.display->drawString(0, 13, display_str);
  display_str = ln_3;
  Heltec.display->drawString(0, 26, display_str);
  display_str = ln_4;
  Heltec.display->drawString(0, 39, display_str);
  display_str = ln_5;
  Heltec.display->drawString(0, 51, display_str);
  Heltec.display->display();
}

/**
 * @brief Read the current value of the TVL knob, convert it to a float,
 * make sure it's within the defined minimum and maximum values for this knob,
 * and return 1/10 of the value, so the display can be to 1 decimal. Always
 * use this function to read the knob, so that all TVL values will be
 * consistent!
 * 
 * Note - since the encoders deal in ints, getCount() and setCount() do too.
 * So getCount() values are divided by 10 to convert, for example, 30 to 3.0,
 * and setCount() values are multiplied by 10, so 3.0 becomes 30.
 */
float get_volume_as_float() {
  float count_as_float = ((float)(tvl_knob.getCount()) / 10.0);
  if (count_as_float > TVL_KNOB_MAX_VAL) {
    count_as_float = TVL_KNOB_MAX_VAL;
    tvl_knob.setCount((uint16_t)(TVL_KNOB_MAX_VAL * 10));
  }
  else if (count_as_float < TVL_KNOB_MIN_VAL) {
    count_as_float = TVL_KNOB_MIN_VAL;
    tvl_knob.setCount((uint16_t)(TVL_KNOB_MIN_VAL * 10));
  }
  return count_as_float;
}

/**
 * @brief Read the current value of the RMV knob.. Always
 * use this function to read the knob, so that all RMV values will be
 * consistent! 
 */
uint8_t get_rmv_as_int() {
  uint8_t count_as_int = rmv_knob.getCount();
  if (count_as_int > RMV_KNOB_MAX_VAL) {
    count_as_int = RMV_KNOB_MAX_VAL;
    rmv_knob.setCount(RMV_KNOB_MAX_VAL);
  }
  else if (count_as_int < RMV_KNOB_MIN_VAL) {
    count_as_int = RMV_KNOB_MIN_VAL;
    rmv_knob.setCount(RMV_KNOB_MIN_VAL);
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
    count_as_float = CO2_KNOB_MAX_VAL;
    co2_knob.setCount((uint16_t)(CO2_KNOB_MAX_VAL * 100));
  }
  else if (count_as_float < CO2_KNOB_MIN_VAL) {
    count_as_float = CO2_KNOB_MIN_VAL;
    co2_knob.setCount((uint16_t)(CO2_KNOB_MIN_VAL * 100));
  }
  return count_as_float;
}

/**
 * @brief Gets current value from all 3 encoders, makes sure the values
 * are in-range (and fixes them if they're not), and updates the OLED
 * if any knob or button changed.
 * */

void get_encoder_values() {
  bool knob_value_changed = false;
  float new_vol = get_volume_as_float();
  uint8_t new_bpm = get_rmv_as_int();
  if (new_vol * new_bpm > RMV_MAX_VAL) {
    update_oled_rmv_max();
  }
  else {
    if (new_vol != current_volume) {
      current_volume = new_vol;
      knob_value_changed = true;
    }
    if (new_bpm != current_bpm) {
      current_bpm = new_bpm;
      knob_value_changed = true;
    }
    float new_co2 = get_co2_duration_as_float();
    if (new_co2 != current_co2_duration) {
      current_co2_duration = new_co2;
      knob_value_changed = true;
    }
    if (knob_value_changed || btn_state_changed) {
        update_oled_values();
        knob_value_changed = false;
        btn_state_changed = false;
      }
  }
}

/**
 * @brief calculate and set the speed, acceleration, and deceleration factors of 
 * the stepper based on the given TVL and RMV.
 * 
 * @param volume - typically, current_volume
 * @param bpm - typically, current_bpm
 */
void set_speed_factors(float volume, uint16_t bpm) {
  float time = ((60.0 / (float)bpm) / 2.0); // time in seconds per cycle
  float distance = (volume * VOLUME_TO_MM_CONVERSION);
  float speed = ((distance * 4.0) / time); // (TVL* MM per liter / seconds per cycle
  float accel = (((distance / 4.0) / pow((time / 4.0), 2.0)));
  Serial.printf("TV=%2.2f BPM=%d time=%4.3f speed=%4.3f accel=%4.3f \n",volume, bpm, time, speed, accel);
  stepper.setSpeedInMillimetersPerSecond(speed);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(accel); 
  stepper.setDecelerationInMillimetersPerSecondPerSecond(accel);
}

/**
 * @brief Gets the piston to inhale_start (0.0),
 * then sets state to INHALE_NEXT, so that no matter what stroke we were in when
 * the Stop was called (INHALE or EXHALE), it will always go to INHALE next.
 */

void release_a_stop()
{
  //set_speed_factors(current_volume, current_bpm); BAS: not necessary - speed factors didn't change
  stepper.setTargetPositionInMillimeters(inhale_start);
  while (!stepper.motionComplete()) {
    // do nothing
  }
  state = INHALE_NEXT;
}

/**
 * @brief Needed to write MAX_STROKE_LENGTH_IN_MM to EEPROM, because
 * EEPROM stores only bytes.
 */
void write_int_to_eeprom(uint8_t address, int16_t number) { 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

/**
 * @brief Needed to read MAX_STROKE_LENGTH_IN_MM from EEPROM.
 */
int16_t read_int_from_eeprom(uint8_t address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

// ************************ SETUP() ********************************** //

void setup() {
  pinMode(limit_switch_data_pin, INPUT_PULLUP);
  pinMode(rmv_btn_pin, INPUT_PULLUP);
  pinMode(tvl_btn_pin, INPUT_PULLUP);
  pinMode(co2_btn_pin, INPUT_PULLUP);
  pinMode(co2_valve_pin, OUTPUT);
  digitalWrite(co2_valve_pin, HIGH); // get the valve closed right awway.
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);

  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  rmv_knob.attachSingleEdge(rmv_B_pin, rmv_A_pin);
  rmv_knob.setFilter(1023);
  rmv_knob.setCount(RMV_KNOB_START_VAL); // set the initially-displayed RMV

  tvl_knob.attachSingleEdge(tvl_B_pin, tvl_A_pin);
  tvl_knob.setFilter(1023);
  tvl_knob.setCount(TVL_KNOB_START_VAL * 10); // set the initially-displayed TVL (3 * 10; will be divided by 10 for display)

  co2_knob.attachSingleEdge(co2_B_pin, co2_A_pin);
  co2_knob.setFilter(1023);
  co2_knob.setCount(CO2_KNOB_START_VAL * 100); // set the initially-displayed CO2 duration (20 * 100; will be divided by 100 for display)

  attachInterrupt(co2_btn_pin, co2_btn_isr, FALLING);
  attachInterrupt(rmv_btn_pin, pause_btn_isr, FALLING);
  attachInterrupt(tvl_btn_pin, home_btn_isr, FALLING);

  Heltec.begin(true, false, true);
  Heltec.display->clear();
  Heltec.display->resetOrientation();
  Heltec.display->display();

  stepper.connectToPins(motor_pulse_pin, motor_direction_pin);
  stepper.setStepsPerMillimeter(motorStepsPerMillimeter);
  stepper.startAsService(0);
  if (stepper.isStartedAsService()) {
    Serial.println("Stepper started in Core 0");
  }
  else {
    Serial.println("Stepper failed to start in Core 0");
  }
  delay(500);
  int16_t max_stroke_from_memory = read_int_from_eeprom(0);
  Serial.println("value retrieved from EEPROM = " + String(max_stroke_from_memory));
  if (digitalRead(co2_btn_pin) == HIGH && max_stroke_from_memory >= HUGE_CALIB_MOVE_IN_MM) { // normal startup w/ a valid value from EEPROM
    state = HOMING;
    MAX_STROKE_LENGTH_IN_MM = max_stroke_from_memory;
  }
  else {
    state = CALIBRATE;
    if (max_stroke_from_memory == 0.0) {
      update_oled_calibrate("No prior calibration", "detected.", "", "Going to calibration", "now.");
      delay(3000); // to leave message on the OLED for a few seconds
    }
  }

} // setup;

// ************************ LOOP() ************************** //

void loop() {

  switch (state) {

  case HOMING: {
    update_oled_homing();
    Serial.println("Case HOMING");
    stepper.setSpeedInMillimetersPerSecond(250);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(150);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(75);
    if (!stepper.moveToHomeInMillimeters(-1, 40, ACTUAL_LENGTH_OF_SCREW_IN_MM, limit_switch_data_pin)) {
      Serial.println("Something wrong in moveToHomeInMillimeters()");
      state = FAILED_HOMING;
    }
    else {
    Serial.println("moveToHomeInMillimeters() was successful");
    // at this point, the piston should be just past the point where the limit switch has gone HIGH again, and
    // moveToHomeInMM() has set this position = to 0. Positions closer to the limit switch are negative, and
    // positions farther from the limit switch are positive.
    // the piston is now at a position that's the closest it should ever get to the limit switch again,
    // so now move it to the desired position for the start of each inhale stroke - MAX_STROKE_LENGTH_IN_MM farther.
    stepper.setTargetPositionInMillimeters((float)MAX_STROKE_LENGTH_IN_MM);
    while (!stepper.motionComplete()) {
      // BAS: monitor the PAUSE/RESUME buttons here? Forrest says YES
    }
    
    Serial.println("Should now be at the starting point for every inhale stroke");
    Serial.println("Actual current position is " + String(stepper.getCurrentPositionInMillimeters()));
    // now we should be at the point where every inhale stroke begins, and the point where every
    // exhale stroke returns to. Set it to 0.0.
    // The following was here, and was working. Suddenly, it wasn't working, so it was moved to
    // the top of INHALE. (The motor would occasionally make a very large move again, at this point.)
     //stepper.setCurrentPositionInMillimeters(0.0);
    }
    // if we made it all the way through HOMING w/o going to FAILED_HOMING, go to IDLE
    if (state == HOMING) {
      state = IDLE;
    }
    break;
  } // end HOMING

  case FAILED_HOMING: {
    Serial.println("Case FAILED_HOMING");
    update_oled_failed_homing();
    while (state == FAILED_HOMING) {
      if (tvl_btn_interrupt_fired) {
        tvl_btn_interrupt_fired = false;
        state = HOMING; 
        break;
      }
      delay(100); // state will NOT switch without this delay!
    }
    break;
  } // case FAILED_HOMING

  case IDLE: { //  Idle - getting knob values and waiting for the START button
    Serial.println("Case IDLE");
    pause_btn_action = RESUME;
    draw_oled_header();
    update_oled_values();
    while (state == IDLE) {
      if (rmv_btn_interrupt_fired) { // any push of the pause_btn acts as the "START" button in this situation
        Serial.println("IDLE: pause/release btn");
        state = INHALE;
        pause_btn_action = PAUSE; // set for the next time the button is pushed
        btn_state_changed = true;
        rmv_btn_interrupt_fired = false;
        break;
      }
      if (tvl_btn_interrupt_fired) {
        Serial.println("IDLE: home btn");
        state = HOMING;
        tvl_btn_interrupt_fired = false;
        break;
      }
      get_encoder_values();
    }
    break;
  } // case IDLE

  case INHALE_NEXT: {
    Serial.println("Case INHALE_NEXT"); // for breaking out of a Stop
    state = INHALE; // BAS: try setting state to IDLE at this point - see if the ensuing stalling stops happening
    break;
  } // case INHALE_NEXT

  case INHALE: {
    Serial.println("Case INHALE");
    // record the current position (immediately after HOMING or CALIBRATE) as 0.0. It was at the end of HOMING, but
    // that resulted in the occasional extra movement - something stuck in the task queue, maybe? It works here.
    stepper.setCurrentPositionInMillimeters(0.0);
    get_encoder_values();
    set_speed_factors(current_volume, current_bpm);
    stepper.setTargetPositionInMillimeters(-1 * current_volume * VOLUME_TO_MM_CONVERSION);
    while (!stepper.motionComplete() || emx_stop_in_effect) {
      // if the co2_duration is > the time of the exhale stroke, the valve may still be open
      if (co2_valve_opened) {
        if ((float)co2_timer / 1000.0 >= current_co2_duration) {
          close_co2_valve();
          co2_valve_opened = false;
        }
      }
      if (rmv_btn_interrupt_fired) {
        if (pause_btn_action == PAUSE) {
          // co2 valve was already closed, in the isr for this button
          pause_btn_action = RESUME; // set for the next button press
          stepper.setTargetPositionToStop();
          Serial.println("INHALE: pause_btn Stop");
          rmv_btn_interrupt_fired = false;
          emx_stop_in_effect = true;
        }
        else { // pause button press means RESUME
          pause_btn_action = PAUSE; // set for the next button press
          release_a_stop();
          Serial.println("INHALE: pause_btn release Stop");
          rmv_btn_interrupt_fired = false;
          emx_stop_in_effect = false;
        }
        btn_state_changed = true;
      }
      if (tvl_btn_interrupt_fired) {
         // co2 valve has already been closed, in the isr for this button
         stepper.setTargetPositionToStop();
         Serial.println("INHALE: home_btn Stop");
         state = HOMING;
         tvl_btn_interrupt_fired = false;
         break; // break out of the while(), but not out of the INHALE case
      }
      get_encoder_values();
    }
    if (state == INHALE) { // it didn't change to HOMING in the while()
      state = EXHALE;
    }  // state changed in the while(), so don't change to EXHALE at this point
    break; // break out of the switch() and start again at the top with the new state: EXHALE or HOMING
  }

  case EXHALE: { // move back to the inhale_start position (0.0)
    Serial.println("Case         EXHALE");
    stepper.setTargetPositionInMillimeters(inhale_start);
    co2_valve_opened = open_co2_valve(); // true if valve is enabled and duration is > 0.0
    while (!stepper.motionComplete() || emx_stop_in_effect) {
      if (co2_valve_opened) {
        if ((float)co2_timer / 1000.0 >= current_co2_duration) {
          close_co2_valve();
          co2_valve_opened = false;
        }
      }
      if (rmv_btn_interrupt_fired) {
        if (pause_btn_action == PAUSE) {
          // co2 valve was already closed, in the isr for this button
          pause_btn_action = RESUME; // set for the next button press
          stepper.setTargetPositionToStop();
          Serial.println("EXHALE: pause_btn Stop");
          rmv_btn_interrupt_fired = false;
          emx_stop_in_effect = true;
        }
        else { // pause button press means "resume"
          pause_btn_action = PAUSE; // set for the next button press
          release_a_stop();
          Serial.println("EXHALE: pause_btn release Stop");
          emx_stop_in_effect = false;
        }
        btn_state_changed = true;
        rmv_btn_interrupt_fired = false;
      }
      if (tvl_btn_interrupt_fired) {
         // co2 valve has already been closed, in the isr for this button
         state = HOMING;
         tvl_btn_interrupt_fired = false;
         stepper.setTargetPositionToStop();
         Serial.println("EXHALE: home_btn Stop");
         break; // break out of the while()
      }
      get_encoder_values();
    }
    if (state == EXHALE) { // it made it all the way through the while() with no state change
      state = INHALE;
    }
    break; // break out of the switch() and start again at the top with the new state: INHALE or HOMING 
  }  // end case EXHALE

   case CALIBRATE: {
    Serial.println("Case CALIBRATE");
    update_oled_calibrate("RELEASE CO2 BUTTON", "", "RELEASE CO2 BUTTON", "", "RELEASE CO2 BUTTON");
    delay(1000);
    update_oled_calibrate("CALIBRATE SWITCH");
    stepper.setSpeedInMillimetersPerSecond(25);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(25);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(25);
    stepper.setCurrentPositionInMillimeters(0);
    tvl_knob.setCount(0);
    int16_t position = 0;
    int16_t last = 0;
    delay(1000);
    while (digitalRead(co2_btn_pin) == HIGH) { // move the piston until it's right where the home switch activates
      position = (tvl_knob.getCount());
      if (position != last) {
        Serial.println("position count " + String(position));
        stepper.setTargetPositionInMillimeters(position);
        while (!stepper.motionComplete()) {
          // do nothing
        }
        last = position;
      }
    }
    update_oled_calibrate("CALIB INHALE STRT_POS", "", (String(HUGE_CALIB_MOVE_IN_MM) + "mm move underway"),
                          "", "No input while moving");    
    stepper.setCurrentPositionInMillimeters(0); // the point where the home switch activates
    stepper.setSpeedInMillimetersPerSecond(50);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(75);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(75);
    stepper.setTargetPositionInMillimeters(HUGE_CALIB_MOVE_IN_MM); // move a lot closer to the inhale beginning point
    while (!stepper.motionComplete()) {
      // do nothing
    }
    stepper.setSpeedInMillimetersPerSecond(25);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(25);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(25);
    tvl_knob.setCount(HUGE_CALIB_MOVE_IN_MM);
    last = HUGE_CALIB_MOVE_IN_MM;
    Serial.println("Huge move complete: position: " + String(stepper.getCurrentPositionInMillimeters()));
    update_oled_calibrate("CALIB INHALE STRT_POS");
    delay(3000); // leave message on OLED for a bit
    while (digitalRead(co2_btn_pin) == HIGH) {
      position = (tvl_knob.getCount());
      if (position != last) {
        Serial.println("position count " + String(position));
        stepper.setTargetPositionInMillimeters(position);
        while (!stepper.motionComplete()) {
          // do nothing
        }
        last = position;
      }
    }
    MAX_STROKE_LENGTH_IN_MM = (int16_t)stepper.getCurrentPositionInMillimeters();
    Serial.println("MAX_STROKE_LENGTH_IN_MM after calibration: " + String(MAX_STROKE_LENGTH_IN_MM));
    // write this value to permanent memory
    write_int_to_eeprom(0, MAX_STROKE_LENGTH_IN_MM);
    delay(100);
    EEPROM.commit();
    delay(100);
    //stepper.setCurrentPositionInMillimeters(0.0);
    tvl_knob.setCount(TVL_KNOB_START_VAL * 10);
    state = HOMING;
    Serial.println("Value written to EEPROM: " + String(read_int_from_eeprom(0)));
    break;
  } // end CALIBRATE

 } // end switch()
} // end loop()