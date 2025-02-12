// Generated by gencpp from file ydlidar_ros_driver/LaserFan.msg
// DO NOT EDIT!


#ifndef YDLIDAR_ROS_DRIVER_MESSAGE_LASERFAN_H
#define YDLIDAR_ROS_DRIVER_MESSAGE_LASERFAN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ydlidar_ros_driver
{
template <class ContainerAllocator>
struct LaserFan_
{
  typedef LaserFan_<ContainerAllocator> Type;

  LaserFan_()
    : header()
    , angle_min(0.0)
    , angle_max(0.0)
    , time_increment(0.0)
    , scan_time(0.0)
    , range_min(0.0)
    , range_max(0.0)
    , angles()
    , ranges()
    , intensities()  {
    }
  LaserFan_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , angle_min(0.0)
    , angle_max(0.0)
    , time_increment(0.0)
    , scan_time(0.0)
    , range_min(0.0)
    , range_max(0.0)
    , angles(_alloc)
    , ranges(_alloc)
    , intensities(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _angle_min_type;
  _angle_min_type angle_min;

   typedef float _angle_max_type;
  _angle_max_type angle_max;

   typedef float _time_increment_type;
  _time_increment_type time_increment;

   typedef float _scan_time_type;
  _scan_time_type scan_time;

   typedef float _range_min_type;
  _range_min_type range_min;

   typedef float _range_max_type;
  _range_max_type range_max;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _angles_type;
  _angles_type angles;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _ranges_type;
  _ranges_type ranges;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _intensities_type;
  _intensities_type intensities;





  typedef boost::shared_ptr< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> const> ConstPtr;

}; // struct LaserFan_

typedef ::ydlidar_ros_driver::LaserFan_<std::allocator<void> > LaserFan;

typedef boost::shared_ptr< ::ydlidar_ros_driver::LaserFan > LaserFanPtr;
typedef boost::shared_ptr< ::ydlidar_ros_driver::LaserFan const> LaserFanConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ydlidar_ros_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'ydlidar_ros_driver': ['/home/nvidia/SLAM_ws/src/x2_cartographer/ydlidar_ros_driver/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "be4554a7f739c3325c744fb261ecf7eb";
  }

  static const char* value(const ::ydlidar_ros_driver::LaserFan_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbe4554a7f739c332ULL;
  static const uint64_t static_value2 = 0x5c744fb261ecf7ebULL;
};

template<class ContainerAllocator>
struct DataType< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ydlidar_ros_driver/LaserFan";
  }

  static const char* value(const ::ydlidar_ros_driver::LaserFan_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
float32 angle_min\n\
float32 angle_max\n\
float32 time_increment\n\
float32 scan_time\n\
float32 range_min\n\
float32 range_max\n\
float32[] angles\n\
float32[] ranges\n\
float32[] intensities\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::ydlidar_ros_driver::LaserFan_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.angle_min);
      stream.next(m.angle_max);
      stream.next(m.time_increment);
      stream.next(m.scan_time);
      stream.next(m.range_min);
      stream.next(m.range_max);
      stream.next(m.angles);
      stream.next(m.ranges);
      stream.next(m.intensities);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LaserFan_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ydlidar_ros_driver::LaserFan_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ydlidar_ros_driver::LaserFan_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "angle_min: ";
    Printer<float>::stream(s, indent + "  ", v.angle_min);
    s << indent << "angle_max: ";
    Printer<float>::stream(s, indent + "  ", v.angle_max);
    s << indent << "time_increment: ";
    Printer<float>::stream(s, indent + "  ", v.time_increment);
    s << indent << "scan_time: ";
    Printer<float>::stream(s, indent + "  ", v.scan_time);
    s << indent << "range_min: ";
    Printer<float>::stream(s, indent + "  ", v.range_min);
    s << indent << "range_max: ";
    Printer<float>::stream(s, indent + "  ", v.range_max);
    s << indent << "angles[]" << std::endl;
    for (size_t i = 0; i < v.angles.size(); ++i)
    {
      s << indent << "  angles[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.angles[i]);
    }
    s << indent << "ranges[]" << std::endl;
    for (size_t i = 0; i < v.ranges.size(); ++i)
    {
      s << indent << "  ranges[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.ranges[i]);
    }
    s << indent << "intensities[]" << std::endl;
    for (size_t i = 0; i < v.intensities.size(); ++i)
    {
      s << indent << "  intensities[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.intensities[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // YDLIDAR_ROS_DRIVER_MESSAGE_LASERFAN_H
