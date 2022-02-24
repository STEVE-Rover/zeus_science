#include <rotationnal_loader.h>
#include <Arduino.h>

class ScienceModule{
private:

public:
    ScienceModule();
    ~ScienceModule();

    void set_modules_stepper_speed();

    RotationnalLoader rotationnal_loader;
};

ScienceModule::ScienceModule(){
    rotationnal_loader = RotationnalLoader();
}

ScienceModule::~ScienceModule(){}

void ScienceModule::set_modules_stepper_speed(){
    rotationnal_loader.set_speed_and_accel();
}