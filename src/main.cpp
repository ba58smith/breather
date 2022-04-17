#include <ESP32Encoder.h>
#include "esp_task_wdt.h"

static bool led = false;
static bool respiration_interrupt_status = false;
static bool volume_interrupt_status = false;

static IRAM_ATTR void respiration_encoder_cb(void* arg) { 
  digitalWrite(LED_BUILTIN, (int)led);
  led = !led;
  respiration_interrupt_status = true;
}

/*
static IRAM_ATTR void volume_encoder_cb(void* arg) {
  //ESP32Encoder* enc = (ESP32Encoder*) arg; // BAS: I commented this out - doesn't seem to do anything. It's never used.
  //Serial.printf("enc cb: count: %d\n", enc->getCount()); 
  //static bool led = false;
  digitalWrite(LED_BUILTIN, (int)led);
  led = !led;
  volume_interrupt_status = true;
}
*/

extern bool loopTaskWDTEnabled;
extern TaskHandle_t loopTaskHandle;

ESP32Encoder respiration_encoder(true, respiration_encoder_cb);
//ESP32Encoder volume_encoder(true, volume_encoder_cb);
ESP32Encoder volume_encoder(true, respiration_encoder_cb);

void setup(){
	
	loopTaskWDTEnabled = true;
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
	// Enable the weak pull down resistors

	//ESP32Encoder::useInternalWeakPullResistors=DOWN;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = UP;

	// BAS: this was attachHalfQuad in the examples, but I think mine is a full quad encoder
  // use pin 17 and 16 for the respiration encoder
	respiration_encoder.attachSingleEdge(17, 16);
  respiration_encoder.setFilter(1023);  
  respiration_encoder.setCount(110);

  // use pin 19 and 18 for the volume encoder
	volume_encoder.attachSingleEdge(23, 18);
  // BAS: following added, along with a 0.1uF cap to GND, for debouncing
  volume_encoder.setFilter(1023);
  volume_encoder.setCount(3);

  esp_task_wdt_add(loopTaskHandle);

  // BAS: the following two lines never display
  Serial.println("volume_encoder: " + String((int32_t)volume_encoder.getCount()));
  Serial.println("respiration_encoder: " + String((int32_t)respiration_encoder.getCount()));
}

void loop(){

  if (volume_interrupt_status) {
    if (volume_encoder.getCount() > 6) {
      volume_encoder.setCount(1);
    }
    else if (volume_encoder.getCount() < 1) {
      volume_encoder.setCount(6);
    }
    Serial.println("volume_encoder: " + String((int32_t)volume_encoder.getCount()));
    //Serial.println("respiration_encoder: " + String((int32_t)respiration_encoder.getCount()));
    // BAS: put code here to update the OLED with the new value, and the appropriate stepper
    // variable (volume, rate).
    volume_interrupt_status = false;
  }

  if (respiration_interrupt_status) {
    if (respiration_encoder.getCount() > 120) {
      respiration_encoder.setCount(100);
    }
    else if (respiration_encoder.getCount() < 100) {
      respiration_encoder.setCount(120);
    }
    Serial.println("respiration_encoder: " + String((int32_t)respiration_encoder.getCount()));
    // BAS: put code here to update the OLED with the new value, and the appropriate stepper
    // variable (volume, rate).
    respiration_interrupt_status = false;
  }
  
}
