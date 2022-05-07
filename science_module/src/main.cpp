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
  if (science.get_status() == IDLE){
    science.start_cycle();
    /*science.set_module_to_poke(msg.module.data);
    science.set_function_to_run(msg.function.data);
    science.set_function_param(msg.param.data);*/
  }
  /*else{
    science.set_module_to_poke(NONE);
    science.set_function_to_run(NONE);
    science.set_function_param(0);
  }*/
}

ros::Subscriber<steve_serial::sub_msg> science_input("msg_subscriber", &messageCb);

MethaneModule methane_module = MethaneModule();

void setup() {
  
  //science.screw_module.init();
  Serial.begin(9600);
  science.set_modules_stepper_speed();

  science.find_origin();

  nh.initNode();
  nh.subscribe(science_input);
  nh.advertise(science_output);
  //science.screw_module.change_motor_state(science.screw_module.up_down_motor,CCW, 0.1);
  //science.rotationnal_loader.find_origin();
  delay(1000);
  //science.screw_module.spin_screw(CW, 0.25);
  //science.rotationnal_loader.find_origin();

  
  //delay(3000);
  //science.rotationnal_loader.move_to_site(0, 0);
  delay(1000);
  //science.rotationnal_loader.move_to_site(3,0);
  /*delay(3000);
  science.rotationnal_loader.move_to_site(7, 0);
  delay(3000);
  science.rotationnal_loader.move_to_site(0, 0);
  delay(3000);
  science.screw_module.spin_screw(STOP);

  Serial.print("this is the initial life of sample 5: ");
  Serial.println(science.rotationnal_loader.get_info_on_sample(4, "life"));
  Serial.println("WE update life to a presence on sample 5.");
  science.rotationnal_loader.set_info_on_sample(4,"life", 1.0);
  Serial.print("Sample 5 now contains: ");
  Serial.println(science.rotationnal_loader.get_info_on_sample(4, "life"));*/
  //methane_module.activate_fan(1.0, 1);
  //pinMode(A0, INPUT);
}

void loop() {
  /*methane_module.activate_fan(1.0, 1);
  delay(2000);*/
  
  science.update_status();
  science_msg.status.data = science.get_status().c_str();
  science_output.publish(&science_msg);
  science.dispatch_modules();
  nh.spinOnce();
}