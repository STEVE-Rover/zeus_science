#include <AccelStepper.h>
#include <Arduino.h>
#include <sample.h>

#define STEP_PER_REV  200
#define num_of_sample_sites 8
#define samples_between_0_1 4 //so there are 4 samples between station 1 and 2
#define gear_ratio 128/76
#define num_of_site 2
#define origin_tolerance 75

#define stepper_max_speed 2500.0
#define stepper_cruise_speed 2000.0
#define stepper_accel 20.0

class RotationnalLoader
{
private:
    uint8_t direction_pin;
    uint8_t step_pin;
    uint8_t MS1_pin; //required?
    uint8_t MS2_pin; //required?
    uint8_t MS3_pin; //required?
    uint8_t position_switch_pin;

    long steps_between_sites;
    int step_mode;
    int sites[num_of_site] = {0, samples_between_0_1}; //Contains which sample is at
    int mem_max_speed;
    int mem_cruising_speed;
    int mem_max_acceleration;

    Sample samples[num_of_sample_sites] = {};
    

public:
    RotationnalLoader(){
      direction_pin = 52;
      step_pin = 53;
      MS1_pin = 51;
      MS2_pin = 49;
      MS3_pin = 47;
      step_mode = 16;
      position_switch_pin = 50;

      stepper1 = AccelStepper(1, step_pin, direction_pin);
      one_revolution = STEP_PER_REV * step_mode; // to the motor and not the drum
      steps_between_sites = (step_mode * STEP_PER_REV / num_of_sample_sites);

      //assigning blank samples to every sample sites
      for(int i; i < num_of_sample_sites; i++){ 
        samples[i] = Sample();
      }

      set_pinout(); //setting pinout outside of arduino setup loop (might not work)
    };

    RotationnalLoader(uint8_t direction_pin_in, uint8_t step_pin_in, uint8_t MS1_pin_in, uint8_t MS2_pin_in, uint8_t MS3_pin_in, int step_mode_in, uint8_t position_switch_pin_in){
      direction_pin = direction_pin_in;
      step_pin = step_pin_in;
      MS1_pin = MS1_pin_in; //required?
      MS2_pin = MS2_pin_in; //required?
      MS3_pin = MS3_pin_in; //required?
      position_switch_pin = position_switch_pin_in;
      step_mode = step_mode_in;

      stepper1 = AccelStepper(1, step_pin, direction_pin);

      one_revolution = STEP_PER_REV * step_mode; // to the motor and not the drum
      steps_between_sites = (step_mode * STEP_PER_REV / num_of_sample_sites);

      //assigning blank samples to every sample sites
      for(int i; i < num_of_sample_sites; i++){ 
        samples[i] = Sample();
      }

      set_pinout(); //setting pinout outside of arduino setup loop (might not work)
    };

    ~RotationnalLoader();

    float max_accel;
    float max_speed;
    float one_revolution;

    void set_pinout();
    float rpm_to_steps(long required_rpm);
    void set_speed_and_accel(float max_speed, float cruising_speed, float max_accel);
    void find_origin();
    void update_sites(int sample_on_site_0);
    void move_to_site(int sample, int site);
    int get_info_on_sample(int sample_number, String param_to_get);
    void set_info_on_sample(int sample_number, String param_to_update, float value);

    AccelStepper stepper1;
};

RotationnalLoader::~RotationnalLoader()
{}

void RotationnalLoader::set_pinout(){
  pinMode(MS1_pin, OUTPUT);
  pinMode(MS2_pin, OUTPUT);
  pinMode(MS3_pin, OUTPUT);
  pinMode(position_switch_pin, INPUT);
}

void RotationnalLoader::set_speed_and_accel(float max_speed=stepper_max_speed, float cruising_speed=stepper_cruise_speed, float max_accel=stepper_accel){
  mem_max_speed = max_speed;
  mem_cruising_speed = cruising_speed;
  mem_max_acceleration = max_accel;

  stepper1.setAcceleration(max_accel);
  stepper1.setMaxSpeed(max_speed);
  stepper1.setSpeed(cruising_speed);

}

float RotationnalLoader::rpm_to_steps(long required_rpm){
  return (float) (abs(required_rpm * gear_ratio * step_mode * STEP_PER_REV)/60);
}

void RotationnalLoader::find_origin(){
  while(digitalRead(position_switch_pin) != 0)
  {
    stepper1.setSpeed(rpm_to_steps(mem_cruising_speed));
    stepper1.runSpeed();
  }

  while(digitalRead(position_switch_pin) != 1)
  {
    stepper1.setSpeed(rpm_to_steps(mem_cruising_speed));
    stepper1.runSpeed();
  }

  stepper1.setCurrentPosition(0);
  
  while(stepper1.currentPosition() != origin_tolerance){  //buffer for origin to fall directly into the hole
    stepper1.setSpeed(rpm_to_steps(mem_cruising_speed));
    stepper1.runSpeed();
  }
  stepper1.setCurrentPosition(0);
    
  update_sites(0);
}

void RotationnalLoader::update_sites(int sample_on_site_0){
  //Serial.println(sample_on_site_0);
  int dummy=sample_on_site_0;

  if (sample_on_site_0 < 0){
    dummy = sample_on_site_0 + num_of_sample_sites;
  }

  for(int i=0; i<num_of_site; i++){
    sites[i] = (sites[i] + dummy) % num_of_sample_sites ;
  }
}

void RotationnalLoader::move_to_site(int sample, int site){
  int sample_at_site = sites[site];
  int path = ((sample - sample_at_site) % num_of_sample_sites);
  int inverse_path = path - num_of_sample_sites;
  int num_of_sample_to_move = 0;

  num_of_sample_to_move = (abs(path) < abs(inverse_path)) ? path:inverse_path;

  Serial.println(num_of_sample_to_move);
 

  if((sample - sample_at_site) != 0){ 
    long distance_to_move = num_of_sample_to_move * steps_between_sites * gear_ratio;
    long final_destination = stepper1.currentPosition() + distance_to_move;  
    int rotation_dir = (num_of_sample_to_move/abs(num_of_sample_to_move));

    while(stepper1.currentPosition() != final_destination){
      stepper1.setSpeed(rotation_dir * rpm_to_steps(mem_cruising_speed));
      stepper1.runSpeed();
    }

    Serial.println(stepper1.currentPosition());
    update_sites((num_of_sample_to_move));
  }
}

void RotationnalLoader::set_info_on_sample(int sample_number, String param_to_update, float value){
  
  if (param_to_update == "life"){
    samples[sample_number].set_life_info(value);
  }
  else if (param_to_update == "humidity"){
    samples[sample_number].set_humidity_info(value);
  }

  else if (param_to_update == "ph"){
    samples[sample_number].set_ph_info(value);
  }
}

int RotationnalLoader::get_info_on_sample(int sample_number, String param_to_get){

  if (param_to_get == "life"){
    return samples[sample_number].get_life_info();
  }
  
  else if (param_to_get == "humidity"){
    return samples[sample_number].get_humidity_info();
  }

  else if (param_to_get == "ph"){
    return samples[sample_number].get_ph_info();
  }

  return -2;
}
