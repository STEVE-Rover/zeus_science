#include <Arduino.h>
#include <science_module.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <custom_msg_test.h>

ScienceModule science = ScienceModule(); 
ros::NodeHandle nh;

zeus_serial::testmsg test; //type of message to publish
ros::Publisher test_msg("msg_publisher", &test);

void messageCb(zeus_serial::testmsg& msg){
  test.module.data = msg.module.data;
  test.function.data = msg.function.data;
  test.param.data = msg.param.data;

  std_msgs::String blip;
  blip.data = "tested";

  test.module.data = blip.data;
  test.function.data = blip.data;
  test.param.data = test.param.data + 1;
}

ros::Subscriber<zeus_serial::testmsg> sub("msg_subscriber", &messageCb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(test_msg);


  /*Serial.begin(9600);
  science.set_modules_stepper_speed();
  delay(1000);
  science.rotationnal_loader.find_origin();
  delay(1000);
  science.rotationnal_loader.move_to_site(7, 0);
  delay(2000);
  science.rotationnal_loader.move_to_site(7, 1);
  Serial.print("this is the initial life of sample 5: ");
  Serial.println(science.rotationnal_loader.get_info_on_sample(4, "life"));
  Serial.println("WE update life to a presence on sample 5.");
  science.rotationnal_loader.set_info_on_sample(4,"life", 1.0);
  Serial.print("Sample 5 now contains: ");
  Serial.println(science.rotationnal_loader.get_info_on_sample(4, "life")); */
}

void loop() {
  test_msg.publish(&test);
  nh.spinOnce();
  delay(500);
}