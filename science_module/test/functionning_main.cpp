#include <Arduino.h>
#include <rotationnal_loader.h>

#define stepper_max_speed 1000.0
#define stepper_cruise_speed 100.0
#define stepper_accel 10.0

/*uint8_t direction_pin = 52;
uint8_t step_pin = 53;
uint8_t MS1_pin = 51;
uint8_t MS2_pin = 49;
uint8_t MS3_pin = 47;
int step_mode = 16;
uint8_t position_switch_pin = 50;*/

/*RotationnalLoader test_class = RotationnalLoader(
  direction_pin,
  step_pin,
  MS1_pin,
  MS2_pin,
  MS3_pin,
  step_mode,
  position_switch_pin
  );*/

RotationnalLoader test_class = RotationnalLoader();

void setup() {
  /*pinMode(MS1_pin, OUTPUT);
  pinMode(MS2_pin, OUTPUT);
  pinMode(MS3_pin, OUTPUT);
  pinMode(position_switch_pin, INPUT);*/

  Serial.begin(9600);
  
  while (test_class.stepper1.speed() != stepper_cruise_speed){
    test_class.set_speed_and_accel(stepper_max_speed,stepper_cruise_speed, stepper_accel); //in rpm and rpm/s2
  }
  delay(1000);
  test_class.find_origin();
  //test_class.move_to_site(7, 0);
  //delay(2000);
  //test_class.move_to_site(7, 1);
  Serial.print("this is the initial life of sample 5: ");
  Serial.println(test_class.get_info_on_sample(4, "life"));
  Serial.println("WE update life to a presence on sample 5.");
  test_class.set_info_on_sample(4,"life", 1.0);
  Serial.print("Sample 5 now contains: ");
  Serial.println(test_class.get_info_on_sample(4, "life"));
}

void loop() {
  
}