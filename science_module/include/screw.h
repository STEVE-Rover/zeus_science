#include <Arduino.h>
#include <Servo.h>
#include <Constant.h>

class ScrewModule{
    private:
        uint8_t bottom_reed_switch_pin;
        uint8_t top_reed_switch_pin;
        uint8_t up_down_motor_pwm_pin;
        uint8_t screw_motor_pwm_pin;
        float screw_pitch = 2.0; //mm per turn
        String status = UNCONSTRUCTED;

    public:
        Servo up_down_motor;
        Servo screw_motor;

        ScrewModule();
        ~ScrewModule();

        void init();
        void change_motor_state(Servo motor_to_change, int motor_direction, float new_state);
        void set_pinout();
        void find_origin(float speed);
        void move_to_ground(float speed);
        void spin_screw(int direction, float speed);
        String get_status();
        void set_status(String new_status);
        void dispatch_functions(String function, int param);

};

ScrewModule::ScrewModule(){
    status = CONSTRUCTOR;
    bottom_reed_switch_pin = 22;
    top_reed_switch_pin = 23;
    up_down_motor_pwm_pin = 9;
    screw_motor_pwm_pin = 10;

    set_pinout();
    status = IDLE;
}

ScrewModule::~ScrewModule(){
}

void ScrewModule::init(){
    status = INIT;
    up_down_motor.attach(up_down_motor_pwm_pin);
    screw_motor.attach(screw_motor_pwm_pin);
    status = IDLE;
}

void ScrewModule::set_pinout(){
  status = BUSY;
  pinMode(top_reed_switch_pin, INPUT_PULLUP);
  pinMode(bottom_reed_switch_pin, INPUT_PULLUP);
  pinMode(up_down_motor_pwm_pin, OUTPUT);
  pinMode(screw_motor_pwm_pin, OUTPUT);
  status = IDLE;
}

void ScrewModule::change_motor_state(Servo motor_to_change, int motor_direction, float new_state){
    status = BUSY;
    int val = 90;

    if (motor_direction == CCW){
    val = (new_state * 90);
    }

    else if (motor_direction == CW){
        val = (new_state * 90) + 90;
    }

    else{
        val = 90;
    }

    motor_to_change.write(val);
    status = IDLE;
}

void ScrewModule::find_origin(float speed=0.5){
    status = BUSY;
    int origin_needed = -1;

    if(digitalRead(top_reed_switch_pin) == 0){
        origin_needed = 1;
        change_motor_state(up_down_motor, CW, speed);
    }

    if(origin_needed == 1){
        while(digitalRead(top_reed_switch_pin) == 0){
        }
    }
    change_motor_state(up_down_motor, STOP, 0.0);
    status = IDLE;
} 

void ScrewModule::move_to_ground(float speed=0.5){ // TO MODIFY WITH GROUND SENSOR
    status = BUSY;
    int bottom_needed = -1;

    if(digitalRead(bottom_reed_switch_pin) == 0){
        bottom_needed = 1;
        change_motor_state(up_down_motor, CCW, speed);
    }

    if(bottom_needed == 1){
        while(digitalRead(bottom_reed_switch_pin) == 0){
        }
    }
    change_motor_state(up_down_motor, STOP, 0.0);
    status = IDLE;
}

void ScrewModule::spin_screw(int direction, float speed=0.1){
    change_motor_state(screw_motor, direction, 0.1);
}

String ScrewModule::get_status(){
    return status;
}

void ScrewModule::set_status(String new_status){
    status = new_status;
}

void ScrewModule::dispatch_functions(String function, int param){
  if (function == "change_motor_state"){
    //change_motor_state(Servo motor_to_change, int motor_direction, float new_state);
  }

  else if (function == "find_origin"){
    find_origin();
  }

  else if (function == "move_to_ground"){
    //move_to_ground(float speed);
  }

  else if (function == "spin_screw"){
    //spin_screw(int direction, float speed);
  }

  else{}
}