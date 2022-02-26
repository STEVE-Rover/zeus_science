#include <Arduino.h>
#include <Servo.h>



Servo screw;
float val = 0.5;    // variable to read the value from the analog pin

int i = 0;

void setup() {
  screw.attach(9);
}

int calculate_val(float percentage)
{
  float out;
  if (percentage<0){out = (abs(percentage)*90);}

  else{out = (percentage*90) + 90;}

  return out;
}

void change_val(float newVal)
{
   val = newVal;
}

void loop() {     
  i++;
  screw.write(calculate_val(val));
  delay(100);

  if(i > 10)
  {
    i = 0;
    change_val(-val);
  }

  
}

