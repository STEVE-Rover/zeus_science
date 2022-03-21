#include <Arduino.h>
#include <Servo.h>
#include <time.h>
#include <Constant.h>

class MethaneModule{
    public:
    MethaneModule();
    ~MethaneModule();

    Servo pump_motor;
    void pump_fluid(int pumping_direction, float command, float volume);
    float read_gas();
    void activate_fan(float percentage, float duration);
    float analyse_sample_gas();
    String get_status();
    void set_status(String new_status);

    private:
    void set_pinout();
    String status = UNCONSTRUCTED;

    //Methane sensor pins
    uint8_t gas_sensor_pin;
    int gas_thresh = 400; //check to change with map?

    //Fan pin
    uint8_t fan_pwm_pin;
    uint8_t fan_activation_pin;

    //pump variable
    float flow_rate = 100.0; //in ml/min (MAX)
    const static float volume_per_sample = 20.0;
    const static float nominal_pumping_speed = 20.0;

    //Other
    float actual_time; //time for delays
};

MethaneModule::MethaneModule(){
    gas_sensor_pin = A0;
    fan_pwm_pin = A6;
    fan_activation_pin = 40;

}

MethaneModule::~MethaneModule(){
}

void MethaneModule::set_pinout(){
    status = INIT;
    pinMode(gas_sensor_pin, INPUT); //faire un pulldown manuellement sur le board!
    pinMode(fan_pwm_pin, OUTPUT);
    pinMode(fan_activation_pin, OUTPUT);
    status = IDLE;
}

float MethaneModule::read_gas(){
    status = BUSY;
    if (analogRead(gas_sensor_pin) >= gas_thresh){
        status = IDLE;
        return 1.0;
        }
    else{
        status = IDLE;
        return 0.0;
    }
    
}

void MethaneModule::activate_fan(float percentage, float duration){
    status = BUSY;
    float fan_pwm = percentage * 1023.0;
    //analogWrite(fan_pwm_pin, fan_pwm);
    digitalWrite(fan_activation_pin, 1);
    delay(duration*1000);
    digitalWrite(fan_activation_pin, 0);
    //analogWrite(fan_pwm, 0.0);
    status = IDLE;
}

void MethaneModule::pump_fluid(int motor_direction, float command=nominal_pumping_speed, float volume=volume_per_sample){ //Tester combien de temps ça prend au fluide de circuler
    status = BUSY;
    int val = 90;
    float converted_pump_rate = (volume / flow_rate) * 60.0 * 1000.0;

    if (motor_direction == CCW){
    val = (command * 90);
    }

    else if (motor_direction == CW){
        val = (command * 90) + 90;
    }

    else{
        val = 90;
    }

    pump_motor.write(val);
    delay(converted_pump_rate);
    pump_motor.write(90);

    status = IDLE;
}

float MethaneModule::analyse_sample_gas(){
    pump_fluid(CW);
    delay(5000);
    float presence_of_gas = read_gas();
    activate_fan(0.2, 5.0);
    return presence_of_gas;
}

String MethaneModule::get_status(){
  return status;
}

void MethaneModule::set_status(String new_status){
  status = new_status;
}