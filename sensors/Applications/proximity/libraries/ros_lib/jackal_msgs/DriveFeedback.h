#ifndef _ROS_jackal_msgs_DriveFeedback_h
#define _ROS_jackal_msgs_DriveFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jackal_msgs
{

  class DriveFeedback : public ros::Msg
  {
    public:
      float current;
      float duty_cycle;
      float bridge_temperature;
      float motor_temperature;
      float measured_velocity;
      float measured_travel;
      bool driver_fault;

    DriveFeedback():
      current(0),
      duty_cycle(0),
      bridge_temperature(0),
      motor_temperature(0),
      measured_velocity(0),
      measured_travel(0),
      driver_fault(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_duty_cycle;
      u_duty_cycle.real = this->duty_cycle;
      *(outbuffer + offset + 0) = (u_duty_cycle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duty_cycle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duty_cycle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duty_cycle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duty_cycle);
      union {
        float real;
        uint32_t base;
      } u_bridge_temperature;
      u_bridge_temperature.real = this->bridge_temperature;
      *(outbuffer + offset + 0) = (u_bridge_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bridge_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bridge_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bridge_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bridge_temperature);
      union {
        float real;
        uint32_t base;
      } u_motor_temperature;
      u_motor_temperature.real = this->motor_temperature;
      *(outbuffer + offset + 0) = (u_motor_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_temperature);
      union {
        float real;
        uint32_t base;
      } u_measured_velocity;
      u_measured_velocity.real = this->measured_velocity;
      *(outbuffer + offset + 0) = (u_measured_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_velocity);
      union {
        float real;
        uint32_t base;
      } u_measured_travel;
      u_measured_travel.real = this->measured_travel;
      *(outbuffer + offset + 0) = (u_measured_travel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_travel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_travel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_travel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_travel);
      union {
        bool real;
        uint8_t base;
      } u_driver_fault;
      u_driver_fault.real = this->driver_fault;
      *(outbuffer + offset + 0) = (u_driver_fault.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->driver_fault);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_duty_cycle;
      u_duty_cycle.base = 0;
      u_duty_cycle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duty_cycle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duty_cycle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duty_cycle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->duty_cycle = u_duty_cycle.real;
      offset += sizeof(this->duty_cycle);
      union {
        float real;
        uint32_t base;
      } u_bridge_temperature;
      u_bridge_temperature.base = 0;
      u_bridge_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bridge_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bridge_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bridge_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bridge_temperature = u_bridge_temperature.real;
      offset += sizeof(this->bridge_temperature);
      union {
        float real;
        uint32_t base;
      } u_motor_temperature;
      u_motor_temperature.base = 0;
      u_motor_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_temperature = u_motor_temperature.real;
      offset += sizeof(this->motor_temperature);
      union {
        float real;
        uint32_t base;
      } u_measured_velocity;
      u_measured_velocity.base = 0;
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_velocity = u_measured_velocity.real;
      offset += sizeof(this->measured_velocity);
      union {
        float real;
        uint32_t base;
      } u_measured_travel;
      u_measured_travel.base = 0;
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_travel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_travel = u_measured_travel.real;
      offset += sizeof(this->measured_travel);
      union {
        bool real;
        uint8_t base;
      } u_driver_fault;
      u_driver_fault.base = 0;
      u_driver_fault.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->driver_fault = u_driver_fault.real;
      offset += sizeof(this->driver_fault);
     return offset;
    }

    const char * getType(){ return "jackal_msgs/DriveFeedback"; };
    const char * getMD5(){ return "8dd0b7a3cfa20cfc5c054ddd9763609b"; };

  };

}
#endif