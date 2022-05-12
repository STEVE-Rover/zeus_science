#include <Arduino.h>
#include <science_module.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <pub_msg.h>
#include <sub_msg.h>
#include <Servo.h>


ScienceModule science = ScienceModule();

ros::NodeHandle nh;

steve_serial::pub_msg science_msg; // change testmsg to science_msg

ros::Publisher science_output("science_state", &science_msg);


void messageCb(steve_serial::sub_msg& msg){
  if (science.get_status() == IDLE /*&& msg.start*/){ //msg.start is the bool to send the whole code running
    science.start_cycle(-1/*msg.sample_do_over*/); // if -1 : continue to increment. else: redo the sample given To add: the custom msg
  }
  else{
    ; //simple pass
  }
}

ros::Subscriber<steve_serial::sub_msg> science_input("msg_subscriber", &messageCb);

MethaneModule methane_module = MethaneModule();

void setup() {
  Serial.begin(9600);
  science.set_modules_stepper_speed();

  science.find_origin();

  nh.initNode();
  nh.subscribe(science_input);
  nh.advertise(science_output);
}

void loop() {
  delay(1000);
  science.update_status();
  science_msg.status.data = science.get_status().c_str();
  science_output.publish(&science_msg);
  //science.dispatch_modules();
  nh.spinOnce();
}