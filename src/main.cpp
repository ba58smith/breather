#include <ESP32Encoder.h>

ESP32Encoder volume_encoder;
ESP32Encoder respiration_encoder;

void setup(){
	
	Serial.begin(115200);
	// Enable the weak pull down resistors

	//ESP32Encoder::useInternalWeakPullResistors=DOWN;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;

	// BAS: this was attachHalfQuad in the examples, but I think mine is a full quad encoder
  // use pin 19 and 18 for the first encoder
	volume_encoder.attachSingleEdge(23, 18);
  // BAS: following added, along with a 0.1uF cap to GND, for debouncing
  volume_encoder.setFilter(1023);
	// use pin 17 and 16 for the second encoder
	respiration_encoder.attachSingleEdge(17, 16);
  respiration_encoder.setFilter(1023);
		
	// set starting count to something other than 0, for comparison to respiration_encoder
	//volume_encoder.setCount(37);
  volume_encoder.clearCount();

	// clear the encoder's raw count and set the tracked count to zero
	respiration_encoder.clearCount();
	Serial.println("Encoder Start = " + String((int32_t)volume_encoder.getCount()));
}

void loop(){
	// Loop and read the counts
	Serial.println("volume_encoder: " + String((int32_t)volume_encoder.getCount()) 
              + "; respiration_encoder: " + String((int32_t)respiration_encoder.getCount()));
	delay(1000);
  
}
