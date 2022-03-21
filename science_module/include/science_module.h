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