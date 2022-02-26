#include <Arduino.h>
#include <science_module.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <custom_msg_test.h>
#include <Servo.h>


ScienceModule science = ScienceModule();

void setup() {
  science.screw_module.init();
  Serial.begin(9600);
  science.set_modules_stepper_speed();
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
  /*science.screw_module.change_state(science.screw_module.up_down_motor, CW, 0.5);
  delay(3000);
  science.screw_module.change_state(science.screw_module.up_down_motor, CCW, 0.5);
  delay(3000);
  science.screw_module.change_state(science.screw_module.up_down_motor, 2, 0.0);*/
}

void loop() {
  science.screw_module.change_state(science.screw_module.up_down_motor, CW, 0.5);
  delay(3000);
}