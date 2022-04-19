#include <Arduino.h>
#include <ESP32Encoder.h>
#include "esp_task_wdt.h"
#include <heltec.h>
#include <ESP_FlexyStepper.h>

#define HOME 0
#define BPM_MULTIPLIER 30  // SWAG at this point
#define VOLUME_TO_MM_CONVERSION 25 // BAS: 25.0 is an approximation - need to get it exact

/**
 * @brief Updates only the area of the OLED that displays the Volume
 */

int topcornerX = 1;
int topcornerY = 30;
int bottomcornerX = 128;
int bottomcornerY = 64;

void update_oled(float volume, uint8_t bpm)
{
  Heltec.display->setColor(BLACK); //color to use
  Heltec.display->fillRect(topcornerX, topcornerY, bottomcornerX, bottomcornerY);
  Heltec.display->setColor(WHITE); //color to use
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

static IRAM_ATTR void bpm_encoder_cb(void *arg)
{
  digitalWrite(LED_BUILTIN, !led);
  bpm_knob_interrupt_fired = true;
}

static IRAM_ATTR void volume_encoder_cb(void *arg)
{
  digitalWrite(LED_BUILTIN, !led);
  volume_knob_interrupt_fired = true;
}

void limit_switch_isr()
{
  limit_switch_interrupt_fired = true;
}

//extern bool loopTaskWDTEnabled;
//extern TaskHandle_t loopTaskHandle;

ESP32Encoder bpm_encoder(true, bpm_encoder_cb);
ESP32Encoder volume_encoder(true, volume_encoder_cb);

float get_volume_as_float()
{
  float count = (float)volume_encoder.getCount() / 10;
  if (count > 6.0)
  {
    count = 1.0;
    volume_encoder.setCount(1);
  }
  else if (count < 1.0)
  {
    count = 6.0;
    volume_encoder.setCount(60);
  }
  return count;
}

uint8_t get_bpm_as_int()
{
  uint8_t count = bpm_encoder.getCount();
  if (count > 6)
  {
    count = 1;
    bpm_encoder.setCount(1);
  }
  else if (count < 1)
  {
    count = 6;
    bpm_encoder.setCount(6);
  }
  return count;
}

ESP_FlexyStepper stepper;

float motorStepsPerMillimeter = 25; // BAS: verified
//float motorStepsPerRevolution = 200;
float speedInMillimetersPerSecond = 200; // controlled by bpm knob
float distanceToMoveInMillimeters = 76;  // controlled by volume knob
float accelerationInMillimetersPerSecondPerSecond = 100;
float decelerationInMillimetersPerSecondPerSecond = 100;
uint8_t motor_direction = 1;

float target_position_in_mm = VOLUME_TO_MM_CONVERSION;
uint8_t bpm_desired = 1 * BPM_MULTIPLIER; // for testing

void setup()
{

  //loopTaskWDTEnabled = true;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(limit_switch_pin, INPUT_PULLUP);
  Serial.begin(115200);

  // Enable the weak pull down resistors
  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  // use pin 17 and 16 for the bpm encoder
  bpm_encoder.attachSingleEdge(bpm_DT_pin, bpm_CLK_pin);
  bpm_encoder.setFilter(1023);
  bpm_encoder.setCount(1);

  // use pin 19 and 18 for the volume encoder
  volume_encoder.attachSingleEdge(volume_DT_pin, volume_CLK_pin);
  // BAS: following added, along with a 0.1uF cap to GND, for debouncing
  volume_encoder.setFilter(1023);
  volume_encoder.setCount(30);

  //attachInterrupt(limit_switch_pin, LOW);

  //esp_task_wdt_add(loopTaskHandle);

  stepper.connectToPins(motor_step_pin, motor_direction_pin);
  stepper.startAsService(); // BAS: this s/b 0, not 1
  stepper.setStepsPerMillimeter(motorStepsPerMillimeter);
  //stepper.setStepsPerRevolution(motorStepsPerRevolution);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(accelerationInMillimetersPerSecondPerSecond);
  stepper.setDecelerationInMillimetersPerSecondPerSecond(decelerationInMillimetersPerSecondPerSecond);
  stepper.setSpeedInMillimetersPerSecond(speedInMillimetersPerSecond);

  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
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
    // BAS: put code here to update the OLED with the new value, and the appropriate stepper
    // variable (volume, rate).
    target_position_in_mm = get_volume_as_float() * VOLUME_TO_MM_CONVERSION;
    stepper.setTargetPositionInMillimeters(target_position_in_mm);
    volume_knob_interrupt_fired = false;
  }

  if (bpm_knob_interrupt_fired) {
    update_oled(get_volume_as_float(), get_bpm_as_int());
    // BAS: put code here to update the OLED with the new value, and the appropriate stepper
    // variable (volume, rate).
    bpm_desired = get_bpm_as_int() * BPM_MULTIPLIER;
    stepper.setSpeedInMillimetersPerSecond(bpm_desired);
    stepper.setAccelerationInMillimetersPerSecondPerSecond(bpm_desired * 1.2);
    bpm_knob_interrupt_fired = false;
  }

  delay(200);
  
  // begine the inhale stroke  
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }

  // Now reverse it for the exhale stroke
  target_position_in_mm *= -1.0;
  stepper.setTargetPositionInMillimeters(HOME);
  while (!stepper.motionComplete()) {
    // Note: Any code added to this loop must execute in no more than 0.05 milliseconds.
    stepper.processMovement();
  }

  //delay(100);
  target_position_in_mm *= -1.0;
  stepper.setTargetPositionInMillimeters(target_position_in_mm);
  
}
