#include <AccelStepper.h>
#include <Arduino.h>
#include <sample.h>

class RotationnalLoader
{
private:
    int step_mode;
    int step_per_rev = 200;
    long steps_between_sites;
    int sites[2] = {0, 4};
    int recipient_on_site_0; //site 0 = dirt
    int recipient_on_site_1; //site 1 = eau
    int samples_between_0_1 = 4;  //so there are 4 samples between station 1 and 2

    int mem_max_speed;
    int mem_cruising_speed;
    int mem_max_acceleration;
    

public:
    RotationnalLoader(uint8_t direction_pin_in, uint8_t step_pin_in, uint8_t MS1_pin_in, uint8_t MS2_pin_in, uint8_t MS3_pin_in, int step_mode_in, uint8_t position_switch_pin_in);
    ~RotationnalLoader();

    AccelStepper loader_motor;
    float rpm_to_steps(long required_rpm);
    void set_speed_and_accel(float max_speed, float cruising_speed, float max_accel);
    void find_origin();
    void update_sites(int sample_on_site_0);
    void move_to_site(int sample, int site);
    float max_accel;
    float max_speed;
    float one_revolution;
    int samples[8] = {};

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
    steps_between_sites = (step_mode * step_per_rev / 8);
    Serial.println(step_mode);
    Serial.println(step_per_rev);
  }

RotationnalLoader::~RotationnalLoader()
{}

void RotationnalLoader::set_speed_and_accel(float max_speed, float cruising_speed, float max_accel){
  mem_max_speed = max_speed;
  mem_cruising_speed = cruising_speed;
  mem_max_acceleration = max_accel;

  loader_motor.setAcceleration(max_accel);
  loader_motor.setMaxSpeed(max_speed);
  loader_motor.setSpeed(cruising_speed);

}

float RotationnalLoader::rpm_to_steps(long required_rpm){
  return (float) (abs(required_rpm * step_mode * step_per_rev)/60);
}

void RotationnalLoader::find_origin(){
  while(digitalRead(position_switch_pin) != 1)
  {
    loader_motor.setSpeed(rpm_to_steps(mem_cruising_speed));
    loader_motor.runSpeed();
  }

  loader_motor.setCurrentPosition(0);
  
  while(loader_motor.currentPosition() != 75) //buffer a patcher
  {
    loader_motor.setSpeed(rpm_to_steps(mem_cruising_speed));
    loader_motor.runSpeed();
  }
  update_sites(0);
}

void RotationnalLoader::update_sites(int sample_on_site_0){ //This function isnt working
  Serial.print("input to update site 0 with: ");
  Serial.println(sample_on_site_0);
  int dummy=sample_on_site_0;

  if (sample_on_site_0<0){
    dummy = sample_on_site_0 + 8;
  }

  sites[0] = (sites[0] + dummy) % 8 ;
  sites[1] = (sites[1] + dummy ) % 8;

  Serial.print("Site 0 is now ");
  Serial.print(sites[0]);
  Serial.print(" and site 1 is now ");
  Serial.println(sites[1]);
}

void RotationnalLoader::move_to_site(int sample, int site){
  int sample_at_site = sites[site];
  int test1 = ((sample - sample_at_site) % 8);
  int test2 = test1 - 8;
  int sample_to_move = 0;

  sample_to_move = (abs(test1) < abs(test2)) ? test1:test2;

  Serial.println(sample_to_move);
 

  if((sample - sample_at_site) != 0){ 
    
    long distance_to_move = sample_to_move * steps_between_sites;
    
    long final_destination = loader_motor.currentPosition() + distance_to_move;
    
    
    update_sites((sample_to_move));
    
    int rotation_dir = (sample_to_move/abs(sample_to_move));

    Serial.println("Starting movement");

    while(loader_motor.currentPosition() != final_destination){
      loader_motor.setSpeed(rotation_dir * rpm_to_steps(mem_cruising_speed));
      loader_motor.runSpeed();
    }

    Serial.println("Finishing movement");
  }
  

  Serial.println(loader_motor.currentPosition());
  Serial.println(digitalRead(position_switch_pin));
}

