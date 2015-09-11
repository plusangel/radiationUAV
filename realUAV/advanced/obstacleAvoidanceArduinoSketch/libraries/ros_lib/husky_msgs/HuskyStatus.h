#ifndef _ROS_husky_msgs_HuskyStatus_h
#define _ROS_husky_msgs_HuskyStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace husky_msgs
{

  class HuskyStatus : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t uptime;
      float ros_control_loop_freq;
      float mcu_and_user_port_current;
      float left_driver_current;
      float right_driver_current;
      float battery_voltage;
      float left_driver_voltage;
      float right_driver_voltage;
      float left_driver_temp;
      float right_driver_temp;
      float left_motor_temp;
      float right_motor_temp;
      uint16_t capacity_estimate;
      float charge_estimate;
      bool timeout;
      bool lockout;
      bool e_stop;
      bool ros_pause;
      bool no_battery;
      bool current_limit;

    HuskyStatus():
      header(),
      uptime(0),
      ros_control_loop_freq(0),
      mcu_and_user_port_current(0),
      left_driver_current(0),
      right_driver_current(0),
      battery_voltage(0),
      left_driver_voltage(0),
      right_driver_voltage(0),
      left_driver_temp(0),
      right_driver_temp(0),
      left_motor_temp(0),
      right_motor_temp(0),
      capacity_estimate(0),
      charge_estimate(0),
      timeout(0),
      lockout(0),
      e_stop(0),
      ros_pause(0),
      no_battery(0),
      current_limit(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->uptime >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uptime >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uptime >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uptime >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uptime);
      offset += serializeAvrFloat64(outbuffer + offset, this->ros_control_loop_freq);
      offset += serializeAvrFloat64(outbuffer + offset, this->mcu_and_user_port_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_driver_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_driver_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->battery_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_driver_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_driver_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_driver_temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_driver_temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_motor_temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_motor_temp);
      *(outbuffer + offset + 0) = (this->capacity_estimate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->capacity_estimate >> (8 * 1)) & 0xFF;
      offset += sizeof(this->capacity_estimate);
      offset += serializeAvrFloat64(outbuffer + offset, this->charge_estimate);
      union {
        bool real;
        uint8_t base;
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timeout);
      union {
        bool real;
        uint8_t base;
      } u_lockout;
      u_lockout.real = this->lockout;
      *(outbuffer + offset + 0) = (u_lockout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lockout);
      union {
        bool real;
        uint8_t base;
      } u_e_stop;
      u_e_stop.real = this->e_stop;
      *(outbuffer + offset + 0) = (u_e_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->e_stop);
      union {
        bool real;
        uint8_t base;
      } u_ros_pause;
      u_ros_pause.real = this->ros_pause;
      *(outbuffer + offset + 0) = (u_ros_pause.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_pause);
      union {
        bool real;
        uint8_t base;
      } u_no_battery;
      u_no_battery.real = this->no_battery;
      *(outbuffer + offset + 0) = (u_no_battery.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->no_battery);
      union {
        bool real;
        uint8_t base;
      } u_current_limit;
      u_current_limit.real = this->current_limit;
      *(outbuffer + offset + 0) = (u_current_limit.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current_limit);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->uptime =  ((uint32_t) (*(inbuffer + offset)));
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->uptime);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ros_control_loop_freq));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mcu_and_user_port_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_driver_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_driver_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_driver_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_driver_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_driver_temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_driver_temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_motor_temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_motor_temp));
      this->capacity_estimate =  ((uint16_t) (*(inbuffer + offset)));
      this->capacity_estimate |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->capacity_estimate);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->charge_estimate));
      union {
        bool real;
        uint8_t base;
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
      union {
        bool real;
        uint8_t base;
      } u_lockout;
      u_lockout.base = 0;
      u_lockout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->lockout = u_lockout.real;
      offset += sizeof(this->lockout);
      union {
        bool real;
        uint8_t base;
      } u_e_stop;
      u_e_stop.base = 0;
      u_e_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->e_stop = u_e_stop.real;
      offset += sizeof(this->e_stop);
      union {
        bool real;
        uint8_t base;
      } u_ros_pause;
      u_ros_pause.base = 0;
      u_ros_pause.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_pause = u_ros_pause.real;
      offset += sizeof(this->ros_pause);
      union {
        bool real;
        uint8_t base;
      } u_no_battery;
      u_no_battery.base = 0;
      u_no_battery.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->no_battery = u_no_battery.real;
      offset += sizeof(this->no_battery);
      union {
        bool real;
        uint8_t base;
      } u_current_limit;
      u_current_limit.base = 0;
      u_current_limit.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->current_limit = u_current_limit.real;
      offset += sizeof(this->current_limit);
     return offset;
    }

    const char * getType(){ return "husky_msgs/HuskyStatus"; };
    const char * getMD5(){ return "fd724379c53d89ec4629be3b235dc10d"; };

  };

}
#endif