#include <Arduino.h>

class Sample
{
private:
    int life; // -1 stands for "nothing was done yet on the sample"
    float ph; //from 0 to 14
    float humidity; //in ppm

public:
    Sample();
    ~Sample();
    void set_life_info(int presence);
    int get_life_info();
    void set_humidity_info(float humidity_value);
    int get_humidity_info();
    void set_ph_info(float ph_value);
    int get_ph_info();
};

Sample::Sample()
{
    life = -1;
    ph = -1.0;
    humidity = -1.0;
}

Sample::~Sample()
{}

void Sample::set_life_info(int presence){
    life = presence;
}

int Sample::get_life_info(){
    return life;
}

void Sample::set_humidity_info(float humidity_value){
    humidity = humidity_value;
}

int Sample::get_humidity_info(){
    return humidity;
}

void Sample::set_ph_info(float ph_value){
    ph = ph_value;
}

int Sample::get_ph_info(){
    return ph;
}