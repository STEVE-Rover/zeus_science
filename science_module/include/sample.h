#include <Arduino.h>

class Sample
{
private:
    int life = -1; // -1 stands for "nothing was done yet"
public:
    Sample();
    ~Sample();
    void set_life_info(int presence);
    int get_life_info();
};

Sample::Sample()
{}

Sample::~Sample()
{}

void Sample::set_life_info(int presence){
    life = presence;
}

int Sample::get_life_info(){
    return life;
}