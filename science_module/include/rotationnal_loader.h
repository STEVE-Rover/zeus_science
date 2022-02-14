#include <AccelStepper.h>
#include <Arduino.h>
#include <sample.h>

class RotationnalLoader
{
private:
    int step_mode;
    int step_per_rev = 200;
    long distance_between_sites;
    int info_about_recipient[8] = {};
    int analysis_site[3] = {};
    int recipient_on_site_0; //site 0 = dirt
    int recipient_on_site_1; //site 1 = eau
    int dist_site_0_to_1;
    

public:
    RotationnalLoader(uint8_t direction_pin_in, uint8_t step_pin_in, uint8_t MS1_pin_in, uint8_t MS2_pin_in, uint8_t MS3_pin_in, int step_mode_in, uint8_t position_switch_pin_in);
    ~RotationnalLoader();

    AccelStepper loader_motor;
    float rpm_to_steps(long required_rpm);
    void set_speed_and_accel(float max_speed, float cruising_speed, float max_accel);
    void find_origin();
    float max_accel;
    float max_speed;
    float one_revolution;

    uint8_t direction_pin;
    uint8_t step_pin;
    uint8_t MS1_pin;
    uint8_t MS2_pin;
    uint8_t MS3_pin;
    uint8_t position_switch_pin;
    
};




RotationnalLoader::RotationnalLoader(
  uint8_t direction_pin_in,
  uint8_t step_pin_in, 
  uint8_t MS1_pin_in, 
  uint8_t MS2_pin_in, 
  uint8_t MS3_pin_in, 
  int step_mode_in, 
  uint8_t position_switch_pin_in){
    direction_pin = direction_pin_in;
    step_pin = step_pin_in;

    loader_motor = AccelStepper(1, step_pin, direction_pin);

    MS1_pin = MS1_pin_in;
    MS2_pin = MS2_pin_in;
    MS3_pin = MS3_pin_in;
    position_switch_pin = position_switch_pin_in;
    step_mode = step_mode_in;
  
    one_revolution = step_per_rev * step_mode;
    distance_between_sites = step_mode * step_per_rev / 360;
    Serial.println(step_mode);
    Serial.println(step_per_rev);
  }

RotationnalLoader::~RotationnalLoader()
{}

void RotationnalLoader::set_speed_and_accel(float max_speed, float cruising_speed, float max_accel){
  //Serial.println("Started setting speed/accel");
  loader_motor.setAcceleration(max_accel);
  loader_motor.setMaxSpeed(max_speed);
  loader_motor.setSpeed(cruising_speed);
  //Serial.println("Finished setting speed/accel");
}

float RotationnalLoader::rpm_to_steps(long required_rpm){
  return (float) (abs(required_rpm * step_mode * step_per_rev)/60);
}

void RotationnalLoader::find_origin(){
  while(digitalRead(position_switch_pin) != 1)
  {
    loader_motor.setSpeed(rpm_to_steps(20.0));
    loader_motor.runSpeed();
  }
  loader_motor.setCurrentPosition(0);
}


