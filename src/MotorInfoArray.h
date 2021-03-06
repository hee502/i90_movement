/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/robotlab/catkin_ws/src/drrobot_I90_player/msg/MotorInfoArray.msg
 *
 */


#ifndef DRROBOT_I90_PLAYER_MESSAGE_MOTORINFOARRAY_H
#define DRROBOT_I90_PLAYER_MESSAGE_MOTORINFOARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include "MotorInfo.h"

namespace drrobot_I90_player
{
template <class ContainerAllocator>
struct MotorInfoArray_
{
  typedef MotorInfoArray_<ContainerAllocator> Type;

  MotorInfoArray_()
    : motorInfos()  {
    }
  MotorInfoArray_(const ContainerAllocator& _alloc)
    : motorInfos(_alloc)  {
    }



   typedef std::vector< ::drrobot_I90_player::MotorInfo_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::drrobot_I90_player::MotorInfo_<ContainerAllocator> >::other >  _motorInfos_type;
  _motorInfos_type motorInfos;




  typedef boost::shared_ptr< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct MotorInfoArray_

typedef ::drrobot_I90_player::MotorInfoArray_<std::allocator<void> > MotorInfoArray;

typedef boost::shared_ptr< ::drrobot_I90_player::MotorInfoArray > MotorInfoArrayPtr;
typedef boost::shared_ptr< ::drrobot_I90_player::MotorInfoArray const> MotorInfoArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace drrobot_I90_player

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/groovy/share/nav_msgs/cmake/../msg'], 'drrobot_I90_player': ['/home/robotlab/catkin_ws/src/drrobot_I90_player/msg'], 'std_msgs': ['/opt/ros/groovy/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/groovy/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/groovy/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/groovy/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "64d8eb9826ec2f78779f54df29bcc931";
  }

  static const char* value(const ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x64d8eb9826ec2f78ULL;
  static const uint64_t static_value2 = 0x779f54df29bcc931ULL;
};

template<class ContainerAllocator>
struct DataType< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "drrobot_I90_player/MotorInfoArray";
  }

  static const char* value(const ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#this message will be used for motor sensor\n\
MotorInfo[] motorInfos\n\
\n\
================================================================================\n\
MSG: drrobot_I90_player/MotorInfo\n\
# motor sensor data message from DrRobot Robot.\n\
\n\
Header header    	# timestamp in the header is the time the driver\n\
		 	# returned the battery/power reading\n\
string robot_type	# robot type, I90 series, sentinel3 or Jaguar Power/Motion\n\
\n\
uint32 encoder_pos	# encoder positon count\n\
uint32 encoder_vel	# encoder velocity value\n\
uint32 encoder_dir	# encoder direction\n\
\n\
float32 motor_current	# motor current\n\
uint32 motor_pwm	# output PWM value, only for Jaguar series robot\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.motorInfos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct MotorInfoArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::drrobot_I90_player::MotorInfoArray_<ContainerAllocator>& v)
  {
    s << indent << "motorInfos[]" << std::endl;
    for (size_t i = 0; i < v.motorInfos.size(); ++i)
    {
      s << indent << "  motorInfos[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::drrobot_I90_player::MotorInfo_<ContainerAllocator> >::stream(s, indent + "    ", v.motorInfos[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DRROBOT_I90_PLAYER_MESSAGE_MOTORINFOARRAY_H
