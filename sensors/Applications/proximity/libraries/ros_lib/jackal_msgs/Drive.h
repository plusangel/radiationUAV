#ifndef _ROS_jackal_msgs_Drive_h
#define _ROS_jackal_msgs_Drive_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jackal_msgs
{

  class Drive : public ros::Msg
  {
    public:
      int8_t mode;
      float drivers[2];
      enum { MODE_VELOCITY = 0    };
      enum { MODE_PWM = 1         };
      enum { MODE_EFFORT = 2      };
      enum { MODE_NONE = -1       };
      enum { LEFT = 0 };
      enum { RIGHT = 1 };

    Drive():
      mode(0),
      drivers()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      for( uint8_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_driversi;
      u_driversi.real = this->drivers[i];
      *(outbuffer + offset + 0) = (u_driversi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_driversi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_driversi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_driversi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->drivers[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
      for( uint8_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_driversi;
      u_driversi.base = 0;
      u_driversi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_driversi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_driversi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_driversi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->drivers[i] = u_driversi.real;
      offset += sizeof(this->drivers[i]);
      }
     return offset;
    }

    const char * getType(){ return "jackal_msgs/Drive"; };
    const char * getMD5(){ return "601cf097cd47c174590c366c6ddd5fb3"; };

  };

}
#endif