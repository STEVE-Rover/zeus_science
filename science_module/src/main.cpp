#include <Arduino.h>
#include <science_module.h>

ScienceModule science = ScienceModule();

void setup() {

  Serial.begin(9600);
  science.set_modules_stepper_speed();
  delay(1000);
  science.rotationnal_loader.find_origin();
  delay(1000);
  science.rotationnal_loader.move_to_site(7, 0);
  delay(2000);
  science.rotationnal_loader.move_to_site(7, 1);
  Serial.print("this is the initial life of sample 5: ");
  Serial.println(science.rotationnal_loader.get_info_on_sample(4, "life"));
  Serial.println("WE update life to a presence on sample 5.");
  science.rotationnal_loader.set_info_on_sample(4,"life", 1.0);
  Serial.print("Sample 5 now contains: ");
  Serial.println(science.rotationnal_loader.get_info_on_sample(4, "life"));
}

void loop() {
  
}