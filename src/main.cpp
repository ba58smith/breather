#include <Arduino.h>
#include <ESP32Encoder.h>
#include <heltec.h>
#include <ESP_FlexyStepper.h>
#include <elapsedMillis.h>

#define HOME 0
#define BPM_MULTIPLIER 30  // SWAG at this point
#define VOLUME_TO_MM_CONVERSION 33.3 // BAS: 33.3 is according to Forrest's spec document
#define MAX_STROKE_LENGTH_IN_MM 254.0 // BAS: 10 inches - just a guess! Need to change 
                                      // to the actual distance necessary to get the pistion to the desired 
                                      // starting point for every inhale cycle.

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

// define all the ISR's and ISR callbacks
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

// BAS: check this interrupt inside the two while() loops, to execute an emergency stop?
void limit_switch_isr() {
  digitalWrite(LED_BUILTIN, led);
  led = !led;
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
//elapsedMillis bpm_timer = 0;

float motorStepsPerMillimeter = 25; // BAS: verified
float distanceToMoveInMillimeters = 76;  // just a starting value - is really controlled by volume knob
float speedInMillimetersPerSecond = 200; // just a starting value - controlled by bpm knob
float accelerationInMillimetersPerSecondPerSecond = 100; // just a starting value - controlled by bpm knob
float decelerationInMillimetersPerSecondPerSecond = 100; // just a starting value - controlled by bpm knob

float target_position_in_mm = VOLUME_TO_MM_CONVERSION; // sets the initial target, until the volume knob is turned
uint8_t bpm_desired = 1 * BPM_MULTIPLIER; // sets the initial BPM, until the BPM knob is turned

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(limit_switch_data_pin, INPUT_PULLUP);
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

  //attachInterrupt(limit_switch_data_pin, LOW);

  stepper.connectToPins(motor_pulse_pin, motor_direction_pin);
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

  /* stepper.moveToHomeInSteps(char_directionToHome, f_speedInStepsPerSecond, long_maxDistanceToMoveInSteps,
                            int_homeLimitSwitchPin);
                            */
  Serial.println("Starting homing operation");
  bool homing_successful = stepper.moveToHomeInMillimeters(-1, 10.0, (100), limit_switch_data_pin);
  Serial.println("Homing successful? " + String(homing_successful));
  // at this point, the piston should be just past the point where the limit switch has gone HIGH again.
  // for safety, move the piston another inch or so away from the limit switch
  stepper.setTargetPositionRelativeInMillimeters(25.4); // BAS: this might need to be NEGATIVE
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }

  // the piston is now at a position that's the closest it should ever get to the limit switch again,
  // so now move it to the desired position for the start of each inhale stroke - XX mm farther
  // NOTE: there's no real reason to do this move, and the previous move, separately. I did it that
  // for now just to illustrate what's happening
  stepper.setTargetPositionRelativeInMillimeters(MAX_STROKE_LENGTH_IN_MM); // BAS: this might need to be NEGATIVE
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }
  // now we should be at Top Dead Center (TDC) - the point where every inhale stroke begins, and the point where every
  // exhale stroke returns to.

  // read the current value of the volume encoder and set it as the target position for the first inhale stroke
  stepper.setTargetPositionInMillimeters(get_volume_as_float() * VOLUME_TO_MM_CONVERSION);
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
    bpm_desired = get_bpm_as_int(); 
    float desired_piston_speed = abs(target_position_in_mm) * 2.0 * bpm_desired  / 60;
    stepper.setSpeedInMillimetersPerSecond(desired_piston_speed);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(desired_piston_speed * desired_piston_speed);
    stepper.setDecelerationInMillimetersPerSecondPerSecond(desired_piston_speed * desired_piston_speed);
    bpm_knob_interrupt_fired = false;
  }

  delay(200);
  
  // begine the inhale stroke  
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    //bpm_timer = 0;
    stepper.processMovement();
  }
  //delay(1500); // for testing when not connected to the stepper
  // save the value of bpm_timer immediately on exiting the while()
  //uint32_t while_loop_time = bpm_timer;
  // calculate BPM from bpm_timer: ms_in_a_half_minute / while_loop_ms
  //float calculated_bpm = 30000.0 / while_loop_time;
  //Serial.println("inhale while() bpm_timer and calculated BPM: " + String(while_loop_time) + " / " + String(calculated_bpm, 1));
 
  // now reverse it for the exhale stroke
  // check the interrupts between the strokes for better updating of the OLED
  /* BAS: remove for now - Forrest says he doesn't want any changes until getting back to TDC
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
  */

  delay(200);
  // BAS: be careful here - if the volume interrupt fired between strokes, target_position_in_mm will be a positive
  // number at this point because it came from the knob. When we start playing with homing, we may flip directionToHome,
  // so target_position_in_mm might need to be a negative number at this point, so we'd have to make it negative
  // inside if (volume_knob_interrupt_fired) {}, so when the next line executes, it would become positive.
  // target_position_in_mm *= -1.0;
  stepper.setTargetPositionInMillimeters(HOME);
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    //bpm_timer = 0;
    stepper.processMovement();
  }
  // now the piston is back at the start of a new inhale cycle
  //delay(1234); // for testing when not connected to the stepper
  //while_loop_time = bpm_timer;
  //calculated_bpm = 60000.0 / (while_loop_time * 2.0); // BAS: does ths round the way I want?
  //Serial.println("exhale while_loop_time and calculated BPM: " + String(while_loop_time) + " / " + String(calculated_bpm, 1));

  //delay(100);
  stepper.setTargetPositionInMillimeters(target_position_in_mm);
  
} // end loop()
