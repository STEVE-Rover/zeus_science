#include <AccelStepper.h>
#include <Arduino.h>
#include <sample.h>

class RotationnalLoader
{
private:
    int step_mode;
    int step_per_rev = 200;
    long distance_between_sites;
    int sites[2] = {0, 0};
    int recipient_on_site_0; //site 0 = dirt
    int recipient_on_site_1; //site 1 = eau
    int sites_between_0_1 = 4;  //so there are 4 samples between station 1 and 2

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
    distance_between_sites = (step_mode * step_per_rev / 8);
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
  for (unsigned int i=0; i<sizeof(sites); i++){
    if (i==0){
      int new_pos = sample_on_site_0 + sites_between_0_1;

      if (new_pos > 7){
        sites[i] = new_pos - 8;
      }
      if (new_pos < 0){
        sites[i] = new_pos + 8;
      }
      else{
        sites[i] = new_pos;
      }
      
    }

  if (i==1){
      if(sample_on_site_0 > 7){
        sites[i] = sample_on_site_0 % 8;
      }

      if(sample_on_site_0 < 0){
        sites[i] = sample_on_site_0 + 8;
      }

      else{
        sites[i] = sample_on_site_0;
      }
      
    }
  }

  Serial.print("Site 0 is now ");
  Serial.print(sites[0]);
  Serial.print(" and site 1 is now ");
  Serial.println(sites[1]);
}

void RotationnalLoader::move_to_site(int sample, int site){
  int sample_at_site = sites[site];
  if(sample != sample_at_site){
    Serial.print("Sample at site ");
    Serial.print(site);
    Serial.print(" is ");
    Serial.println(sample_at_site);
    int sample_to_move = sample_at_site - sample;
    Serial.print("We need to move ");
    Serial.print(sample_to_move);
    Serial.println(" increment");
    long distance_to_move = sample_to_move * distance_between_sites;
    Serial.print("This represents ");
    Serial.print(distance_to_move);
    Serial.println(" steps");
    long final_destination = loader_motor.currentPosition() + distance_to_move;
    Serial.print("Starting from the beginning position, we need to move ");
    Serial.print(final_destination);
    Serial.println(" steps");
    
    update_sites((sites[0] + sample_to_move));
    
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

