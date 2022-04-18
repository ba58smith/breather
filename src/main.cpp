#include <Arduino.h>
#include <ESP32Encoder.h>
#include "esp_task_wdt.h"
#include <heltec.h>
#include <ESP_FlexyStepper.h>

/**
 * @brief Updates only the area of the OLED that displays the Volume
 */

int topcornerX = 40;
int topcornerY = 30;
int bottomcornerX = 100;
int bottomcornerY = 60;

int top2cornerX =1;
int top2cornerY= 30;
int bottom2cornerX=40;
int bottom2cornerY=60;

void displayVolume(uint8_t volume){
     Heltec.display->setColor(BLACK); //color to use
     Heltec.display->fillRect(topcornerX, topcornerY, bottomcornerX, bottomcornerY);
     Heltec.display->setColor(WHITE); //color to use
     Heltec.display->drawString(75, 35, String(volume));
     Heltec.display->display();
     
 }
 void displayResp(uint8_t resp){
     Heltec.display->setColor(BLACK); //color to use
     Heltec.display->fillRect(top2cornerX, top2cornerY, bottom2cornerX, bottom2cornerY);
     Heltec.display->setColor(WHITE); //color to use
     Heltec.display->drawString(20, 35, String(resp));
     Heltec.display->display();
     
 }

static bool led = LOW;
static bool respiration_knob_interrupt_fired = false;
static bool home_btn_interrupt_fired = false;
static bool volume_knob_interrupt_fired = false;
static bool pause_btn_interrupt_fired = false;

const uint8_t pause_btn_pin = 38;
const uint8_t home_btn_pin = 5;
const uint8_t respiration_CLK_pin = 36;
const uint8_t respiration_DT_pin = 37;
const uint8_t volume_CLK_pin = 23;
const uint8_t volume_DT_pin = 18;
const uint8_t motor_step_pin = 27;
const uint8_t motor_direction_pin = 14;

const int STOP_BUTTON_PIN = 9; // From FlexyStepper example


static IRAM_ATTR void respiration_encoder_cb(void *arg)
{
  digitalWrite(LED_BUILTIN, !led);
  respiration_knob_interrupt_fired = true;
}

static IRAM_ATTR void volume_encoder_cb(void *arg)
{
  digitalWrite(LED_BUILTIN, !led);
  volume_knob_interrupt_fired = true;
}

//extern bool loopTaskWDTEnabled;
//extern TaskHandle_t loopTaskHandle;

ESP32Encoder respiration_encoder(true, respiration_encoder_cb);
ESP32Encoder volume_encoder(true, volume_encoder_cb);

uint8_t get_volume() {
  uint8_t count = (float)volume_encoder.getCount();
     if (count > 6) {
       volume_encoder.setCount(1);
     }
     else if (count < 1) {
       volume_encoder.setCount(6);
     }
     return count;
}

uint8_t get_resp() {
  uint8_t count = (float)respiration_encoder.getCount();
     if (count > 6) {
       respiration_encoder.setCount(1);
     }
     else if (count < 1) {
       respiration_encoder.setCount(6);
     }
     return count;
}

ESP_FlexyStepper stepper;

float motorStepPerMillimeter= 25; // verify this
float motorStepPerRevolution = 200;
float speedInMillimetersPerSecond = 0;// controlled by resp knob
float distanceToMoveInMillimeters = 0; // controlled by volume knob
float accelerationInMillimetersPerSecondPerSecond = 400;
float decelerationInMillimetersPerSecondPerSecond = 400;
uint8_t motor_direction = 1;

void setup()
{

  //loopTaskWDTEnabled = true;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  // Enable the weak pull down resistors
  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  // use pin 17 and 16 for the respiration encoder
  respiration_encoder.attachSingleEdge(respiration_DT_pin, respiration_CLK_pin);
  respiration_encoder.setFilter(1023);
  respiration_encoder.setCount(1);

  // use pin 19 and 18 for the volume encoder
  volume_encoder.attachSingleEdge(volume_DT_pin, volume_CLK_pin);
  // BAS: following added, along with a 0.1uF cap to GND, for debouncing
  volume_encoder.setFilter(1023);
  volume_encoder.setCount(1);

  //esp_task_wdt_add(loopTaskHandle);

  stepper.connectToPins(motor_step_pin, motor_direction_pin);
  stepper.setStepsPerMillimeter(motorStepPerMillimeter);
  stepper.setStepsPerRevolution(motorStepPerRevolution);
  stepper.setTargetPositionRelativeInMillimeters(distanceToMoveInMillimeters);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(accelerationInMillimetersPerSecondPerSecond);
  stepper.setDecelerationInMillimetersPerSecondPerSecond(decelerationInMillimetersPerSecondPerSecond);
  stepper.setSpeedInMillimetersPerSecond(speedInMillimetersPerSecond);
  stepper.startAsService();
  stepper.setCurrentPositionInSteps(0);

  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
  Heltec.display->clear();
  Heltec.display->display();
  Heltec.display->resetOrientation();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "BPM   |   Volume");
  Heltec.display->display();
  Heltec.display->setFont(ArialMT_Plain_24);
  displayVolume(get_volume());
  displayResp(get_resp());
}

void loop() {

  if (volume_knob_interrupt_fired) {
    displayVolume(get_volume());
    Serial.println("BPM: " + String((int32_t)respiration_encoder.getCount()) + 
                   "   Volume: " + String((int32_t)volume_encoder.getCount()));
    // BAS: put code here to update the OLED with the new value, and the appropriate stepper
    // variable (volume, rate).
    volume_knob_interrupt_fired = false;
  }

  if (respiration_knob_interrupt_fired) {
    displayResp(get_resp());
    Serial.println("BPM: " + String((int32_t)respiration_encoder.getCount()) + 
                   "   Volume: " + String((int32_t)volume_encoder.getCount()));
    // BAS: put code here to update the OLED with the new value, and the appropriate stepper
    // variable (volume, rate).
    respiration_knob_interrupt_fired = false;
  }

  /*
  if (stepper.getDistanceToTargetSigned() == 0) {
    Serial.println("Target reached");
    delay(100);
    motor_direction *= -1;
    long relativeTargetPosition = distanceToMoveInMillimeters * motor_direction;
    stepper.setTargetPositionRelativeInMillimeters(relativeTargetPosition);
    stepper.setSpeedInMillimetersPerSecond(speedInMillimetersPerSecond);
  }
  */

  uint16_t target_position = 200;

  stepper.setSpeedInStepsPerSecond(100);
  stepper.setAccelerationInStepsPerSecondPerSecond(100);
  //stepper.setCurrentPositionInSteps(0);
  //digitalWrite(LED_BUILTIN, !led);
  bool stopFlag = false;
  stepper.setTargetPositionInSteps(target_position);
  // now execute the move, looping until the motor has finished
  while(!stepper.motionComplete()) {
    // Note: The code added to this loop must execute VERY fast.  
    // Perhaps no longer than 0.05 milliseconds.
    // process motor steps
    //
    stepper.processMovement();
    // check if motor has moved past position 400, if so turn On the LED
    //
    //if (stepper.getCurrentPositionInSteps() == 100)
    //  digitalWrite(LED_BUILTIN, !led);

    // check if the user has pressed the "Stop" button, if so decelerate to a stop
    /*
    if ((digitalRead(STOP_BUTTON_PIN) == LOW) && (stopFlag == false))
    {
      stepper.setTargetPositionToStop();
      stopFlag = true;
    }
    */
  }
  Serial.println("Current position: " + (String)stepper.getCurrentPositionInSteps());
  delay(500);
  stepper.setTargetPositionInSteps(target_position * -1);
  while(!stepper.motionComplete()) {
    // Note: The code added to this loop must execute VERY fast.  
    // Perhaps no longer than 0.05 milliseconds.
    // process motor steps
    //
    stepper.processMovement();
    // check if motor has moved past position 400, if so turn On the LED
    //
    //if (stepper.getCurrentPositionInSteps() == 100)
    //  digitalWrite(LED_BUILTIN, !led);
  }
  Serial.println("Current position: " + (String)stepper.getCurrentPositionInSteps());
  delay(500);
}

     
