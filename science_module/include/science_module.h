#include <rotationnal_loader.h>
#include <screw.h>
#include <Arduino.h>

class ScienceModule{
private:

public:
    ScienceModule();
    ~ScienceModule();

    void set_modules_stepper_speed();

    RotationnalLoaderModule rotationnal_loader;
    ScrewModule screw_module;
};

ScienceModule::ScienceModule(){
    rotationnal_loader = RotationnalLoaderModule();
    screw_module = ScrewModule();
}

ScienceModule::~ScienceModule(){}

void ScienceModule::set_modules_stepper_speed(){
    rotationnal_loader.set_speed_and_accel();
}

//put interrupt on hard limits