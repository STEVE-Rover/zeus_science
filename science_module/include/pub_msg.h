#ifndef _ROS_zeus_serial_pub_msg_h
#define _ROS_zeus_serial_pub_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace zeus_serial
{

  class pub_msg : public ros::Msg
  {
    public:
      typedef std_msgs::String _status_type;
      _status_type status;

    pub_msg():
      status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "zeus_serial/pub_msg"; };
    const char * getMD5(){ return "da740a2c07d1e6cb851fc1b477c8705a"; };

  };

}
#endif
