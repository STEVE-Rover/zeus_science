#include <Arduino.h>
#include <science_module.h>



ScienceModule science_module = ScienceModule();

void setup() {

  Serial.begin(9600);
  
  while (science_module.rotationnal_loader.stepper1.speed() == 1.0){
    science_module.set_modules_stepper_speed(); //in rpm and rpm/s2
  }
  delay(1000);
  science_module.rotationnal_loader.find_origin();
  //test_class.move_to_site(7, 0);
  //delay(2000);
  //test_class.move_to_site(7, 1);
  Serial.print("this is the initial life of sample 5: ");
  Serial.println(science_module.rotationnal_loader.get_info_on_sample(4, "life"));
  Serial.println("WE update life to a presence on sample 5.");
  science_module.rotationnal_loader.set_info_on_sample(4, "life", 1.0);
  Serial.print("Sample 5 now contains: ");
  Serial.println(science_module.rotationnal_loader.get_info_on_sample(4, "life"));
}

void loop() {
  
}