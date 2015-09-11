#ifndef _ROS_jackal_msgs_Status_h
#define _ROS_jackal_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace jackal_msgs
{

  class Status : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* hardware_id;
      ros::Duration mcu_uptime;
      ros::Duration connection_uptime;
      bool drivers_active;
      bool driver_external_stop_present;
      bool driver_external_stop_stopped;
      float measured_battery;
      float measured_12v;
      float measured_5v;
      float drive_current;
      float user_current;
      float computer_current;
      float total_current;
      float total_current_peak;
      float total_power_consumed;

    Status():
      header(),
      hardware_id(""),
      mcu_uptime(),
      connection_uptime(),
      drivers_active(0),
      driver_external_stop_present(0),
      driver_external_stop_stopped(0),
      measured_battery(0),
      measured_12v(0),
      measured_5v(0),
      drive_current(0),
      user_current(0),
      computer_current(0),
      total_current(0),
      total_current_peak(0),
      total_power_consumed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_hardware_id = strlen(this->hardware_id);
      memcpy(outbuffer + offset, &length_hardware_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->hardware_id, length_hardware_id);
      offset += length_hardware_id;
      *(outbuffer + offset + 0) = (this->mcu_uptime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mcu_uptime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mcu_uptime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mcu_uptime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mcu_uptime.sec);
      *(outbuffer + offset + 0) = (this->mcu_uptime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mcu_uptime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mcu_uptime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mcu_uptime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mcu_uptime.nsec);
      *(outbuffer + offset + 0) = (this->connection_uptime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->connection_uptime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->connection_uptime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->connection_uptime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connection_uptime.sec);
      *(outbuffer + offset + 0) = (this->connection_uptime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->connection_uptime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->connection_uptime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->connection_uptime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connection_uptime.nsec);
      union {
        bool real;
        uint8_t base;
      } u_drivers_active;
      u_drivers_active.real = this->drivers_active;
      *(outbuffer + offset + 0) = (u_drivers_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->drivers_active);
      union {
        bool real;
        uint8_t base;
      } u_driver_external_stop_present;
      u_driver_external_stop_present.real = this->driver_external_stop_present;
      *(outbuffer + offset + 0) = (u_driver_external_stop_present.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->driver_external_stop_present);
      union {
        bool real;
        uint8_t base;
      } u_driver_external_stop_stopped;
      u_driver_external_stop_stopped.real = this->driver_external_stop_stopped;
      *(outbuffer + offset + 0) = (u_driver_external_stop_stopped.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->driver_external_stop_stopped);
      union {
        float real;
        uint32_t base;
      } u_measured_battery;
      u_measured_battery.real = this->measured_battery;
      *(outbuffer + offset + 0) = (u_measured_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_battery);
      union {
        float real;
        uint32_t base;
      } u_measured_12v;
      u_measured_12v.real = this->measured_12v;
      *(outbuffer + offset + 0) = (u_measured_12v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_12v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_12v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_12v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_12v);
      union {
        float real;
        uint32_t base;
      } u_measured_5v;
      u_measured_5v.real = this->measured_5v;
      *(outbuffer + offset + 0) = (u_measured_5v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_5v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_5v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_5v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_5v);
      union {
        float real;
        uint32_t base;
      } u_drive_current;
      u_drive_current.real = this->drive_current;
      *(outbuffer + offset + 0) = (u_drive_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_drive_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_drive_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_drive_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->drive_current);
      union {
        float real;
        uint32_t base;
      } u_user_current;
      u_user_current.real = this->user_current;
      *(outbuffer + offset + 0) = (u_user_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_user_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_user_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_user_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->user_current);
      union {
        float real;
        uint32_t base;
      } u_computer_current;
      u_computer_current.real = this->computer_current;
      *(outbuffer + offset + 0) = (u_computer_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_computer_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_computer_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_computer_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->computer_current);
      union {
        float real;
        uint32_t base;
      } u_total_current;
      u_total_current.real = this->total_current;
      *(outbuffer + offset + 0) = (u_total_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_total_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_total_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_total_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_current);
      union {
        float real;
        uint32_t base;
      } u_total_current_peak;
      u_total_current_peak.real = this->total_current_peak;
      *(outbuffer + offset + 0) = (u_total_current_peak.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_total_current_peak.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_total_current_peak.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_total_current_peak.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_current_peak);
      offset += serializeAvrFloat64(outbuffer + offset, this->total_power_consumed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_hardware_id;
      memcpy(&length_hardware_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hardware_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hardware_id-1]=0;
      this->hardware_id = (char *)(inbuffer + offset-1);
      offset += length_hardware_id;
      this->mcu_uptime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mcu_uptime.sec);
      this->mcu_uptime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mcu_uptime.nsec);
      this->connection_uptime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->connection_uptime.sec);
      this->connection_uptime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->connection_uptime.nsec);
      union {
        bool real;
        uint8_t base;
      } u_drivers_active;
      u_drivers_active.base = 0;
      u_drivers_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->drivers_active = u_drivers_active.real;
      offset += sizeof(this->drivers_active);
      union {
        bool real;
        uint8_t base;
      } u_driver_external_stop_present;
      u_driver_external_stop_present.base = 0;
      u_driver_external_stop_present.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->driver_external_stop_present = u_driver_external_stop_present.real;
      offset += sizeof(this->driver_external_stop_present);
      union {
        bool real;
        uint8_t base;
      } u_driver_external_stop_stopped;
      u_driver_external_stop_stopped.base = 0;
      u_driver_external_stop_stopped.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->driver_external_stop_stopped = u_driver_external_stop_stopped.real;
      offset += sizeof(this->driver_external_stop_stopped);
      union {
        float real;
        uint32_t base;
      } u_measured_battery;
      u_measured_battery.base = 0;
      u_measured_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_battery = u_measured_battery.real;
      offset += sizeof(this->measured_battery);
      union {
        float real;
        uint32_t base;
      } u_measured_12v;
      u_measured_12v.base = 0;
      u_measured_12v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_12v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_12v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_12v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_12v = u_measured_12v.real;
      offset += sizeof(this->measured_12v);
      union {
        float real;
        uint32_t base;
      } u_measured_5v;
      u_measured_5v.base = 0;
      u_measured_5v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_5v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_5v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_5v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_5v = u_measured_5v.real;
      offset += sizeof(this->measured_5v);
      union {
        float real;
        uint32_t base;
      } u_drive_current;
      u_drive_current.base = 0;
      u_drive_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_drive_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_drive_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_drive_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->drive_current = u_drive_current.real;
      offset += sizeof(this->drive_current);
      union {
        float real;
        uint32_t base;
      } u_user_current;
      u_user_current.base = 0;
      u_user_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_user_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_user_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_user_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->user_current = u_user_current.real;
      offset += sizeof(this->user_current);
      union {
        float real;
        uint32_t base;
      } u_computer_current;
      u_computer_current.base = 0;
      u_computer_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_computer_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_computer_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_computer_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->computer_current = u_computer_current.real;
      offset += sizeof(this->computer_current);
      union {
        float real;
        uint32_t base;
      } u_total_current;
      u_total_current.base = 0;
      u_total_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_total_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_total_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_total_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->total_current = u_total_current.real;
      offset += sizeof(this->total_current);
      union {
        float real;
        uint32_t base;
      } u_total_current_peak;
      u_total_current_peak.base = 0;
      u_total_current_peak.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_total_current_peak.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_total_current_peak.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_total_current_peak.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->total_current_peak = u_total_current_peak.real;
      offset += sizeof(this->total_current_peak);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->total_power_consumed));
     return offset;
    }

    const char * getType(){ return "jackal_msgs/Status"; };
    const char * getMD5(){ return "c851ebcf9a6e20b196bc7894e285b4f6"; };

  };

}
#endif