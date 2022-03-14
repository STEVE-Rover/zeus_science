#ifndef _ROS_steve_serial_testmsg_h
#define _ROS_steve_serial_testmsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

namespace steve_serial
{

  class testmsg : public ros::Msg
  {
    public:
      typedef std_msgs::String _module_type;
      _module_type module;
      typedef std_msgs::String _function_type;
      _function_type function;
      typedef std_msgs::Int8 _param_type;
      _param_type param;

    testmsg():
      module(),
      function(),
      param()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->module.serialize(outbuffer + offset);
      offset += this->function.serialize(outbuffer + offset);
      offset += this->param.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->module.deserialize(inbuffer + offset);
      offset += this->function.deserialize(inbuffer + offset);
      offset += this->param.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "steve_serial/testmsg"; };
    const char * getMD5(){ return "d49c21b3d51790677a6624022511985b"; };

  };

}
#endif
