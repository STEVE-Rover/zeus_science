#include <rotationnal_loader.h>
#include <screw.h>
#include <Arduino.h>
#include <string.h>
#include <custom_msg_test.h>
#include <Constant.h>

class ScienceModule{
private:
    String module_status = UNCONSTRUCTED;
    String module_to_poke;
    String function_to_run;
    int param;

public:
    ScienceModule();
    ~ScienceModule();

    void set_modules_stepper_speed();
    void find_origin();
    String get_status();
    void set_status(String new_status);

    String get_module_to_poke();
    void set_module_to_poke(String module);

    String get_function_to_run();
    void set_function_to_run(String function);

    int get_function_param();
    void set_function_param(int function_param);

    void update_status();
    void dispatch_modules();

    float start_cycle();

    RotationnalLoaderModule rotationnal_loader;
    ScrewModule screw_module;
};

// DEFINITION STARTS HERE

ScienceModule::ScienceModule(){
    module_status = CONSTRUCTOR;
    rotationnal_loader = RotationnalLoaderModule();
    screw_module = ScrewModule();
    module_status = IDLE;
}

ScienceModule::~ScienceModule(){}

void ScienceModule::set_modules_stepper_speed(){
    rotationnal_loader.set_speed_and_accel();
}

void ScienceModule::find_origin(){
    module_status = ORIGIN;
    rotationnal_loader.find_origin();
    screw_module.find_origin();
    module_status = IDLE;
}

String ScienceModule::get_status(){
    return module_status;
}

void ScienceModule::set_status(String new_status){
    module_status = new_status;
}

String ScienceModule::get_module_to_poke(){
    return module_to_poke;
}

void ScienceModule::set_module_to_poke(String module){
    module_to_poke = module;
}

String ScienceModule::get_function_to_run(){
    return function_to_run;
}

void ScienceModule::set_function_to_run(String function){
    function_to_run = function;
}

int ScienceModule::get_function_param(){
    return param;
}
void ScienceModule::set_function_param(int function_param){
    param = function_param;
}

void ScienceModule::update_status(){
    String new_status = BUSY;
    String screw_status = screw_module.get_status();
    String rotationnal_loader_status = rotationnal_loader.get_status();

    if(screw_status != IDLE){
        new_status = screw_status;
    }
    else if(rotationnal_loader_status != IDLE){
        new_status = rotationnal_loader_status;
    }
    else if(rotationnal_loader_status != IDLE){
        new_status = rotationnal_loader_status;
    }
}

void ScienceModule::dispatch_modules(){
    if(module_to_poke == "Screw"){
        screw_module.dispatch_functions(function_to_run, param);
    }

    if(module_to_poke == "rotationnal_loader"){
        rotationnal_loader.dispatch_functions(function_to_run, param);
    }
}

float ScienceModule::start_cycle(){
    if (rotationnal_loader.actual_sample < 8){
        rotationnal_loader.move_to_site(rotationnal_loader.actual_sample, SCREW_STATION);
        screw_module.move_to_ground();

        screw_module.spin_screw(CW, 0.1); //lifts up dirt
        delay(5000);
        screw_module.spin_screw(CCW, 0.1); //gets the old dirt back on the ground
        delay(5000);

        rotationnal_loader.move_to_site(rotationnal_loader.actual_sample, GAS_STATION);
        rotationnal_loader.methane_module.pump_fluid(CW);
        delay(5000); //Waiting time for the reaction
        rotationnal_loader.analyze_sample(rotationnal_loader.actual_sample, GAS_STATION);
        float gas_information = rotationnal_loader.get_info_on_sample(rotationnal_loader.actual_sample, "life");
        rotationnal_loader.actual_sample ++; //increment the base sample to be processed
        return gas_information;
    }
}