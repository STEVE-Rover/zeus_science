#include <Arduino.h>
#include <Servo.h>

#define CW 1
#define CCW -1

class ScrewModule{
    private:
        uint8_t top_hardmax_pin;
        uint8_t bottom_hardmax_pin;
        uint8_t reed_switch_pin;
        uint8_t up_down_motor_pwm_pin;
        uint8_t screw_motor_pwm_pin;
        float screw_pitch = 2.0; //mm per turn

    public:
        Servo up_down_motor;
        Servo screw_motor;

        ScrewModule();
        ~ScrewModule();

        void init();
        void change_state(Servo motor_to_change, int motor_direction, float new_state);
        void set_pinout();
        void find_origin();
        void move_to_ground();

};

ScrewModule::ScrewModule(){
    top_hardmax_pin = 20;
    bottom_hardmax_pin = 21;
    reed_switch_pin = 22;
    up_down_motor_pwm_pin = 9;
    screw_motor_pwm_pin = 8;

    set_pinout();
}

ScrewModule::~ScrewModule(){
}

void ScrewModule::init(){
    up_down_motor.attach(up_down_motor_pwm_pin);
    screw_motor.attach(screw_motor_pwm_pin);
}

void ScrewModule::set_pinout(){
  pinMode(top_hardmax_pin, INPUT);
  pinMode(bottom_hardmax_pin, INPUT);
  pinMode(reed_switch_pin, INPUT);
  pinMode(up_down_motor_pwm_pin, OUTPUT);
  pinMode(screw_motor_pwm_pin, OUTPUT);
}

void ScrewModule::change_state(Servo motor_to_change, int motor_direction, float new_state){
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
    Serial.println(val);

    motor_to_change.write(val);
}

void ScrewModule::find_origin(){
    int origin_needed = -1;

    if(digitalRead(top_hardmax_pin) == 0){
        origin_needed = 1;
        change_state(up_down_motor, CCW, 1.0);
    }

    if(origin_needed == 1){
        while(digitalRead(top_hardmax_pin) == 0){
        }
        change_state(up_down_motor, CCW, 0.0);
    }
} 

void ScrewModule::move_to_ground(){
    
    if(digitalRead(top_hardmax_pin) == 0){
        //origin_needed = 1;
        change_state(up_down_motor, CCW, 1.0);
    }
}