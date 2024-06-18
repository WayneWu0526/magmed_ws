// Generated by gencpp from file magmed_msgs/PoseTwist.msg
// DO NOT EDIT!


#ifndef MAGMED_MSGS_MESSAGE_POSETWIST_H
#define MAGMED_MSGS_MESSAGE_POSETWIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace magmed_msgs
{
template <class ContainerAllocator>
struct PoseTwist_
{
  typedef PoseTwist_<ContainerAllocator> Type;

  PoseTwist_()
    : header()
    , pose()
    , twist()  {
    }
  PoseTwist_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)
    , twist(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;





  typedef boost::shared_ptr< ::magmed_msgs::PoseTwist_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::magmed_msgs::PoseTwist_<ContainerAllocator> const> ConstPtr;

}; // struct PoseTwist_

typedef ::magmed_msgs::PoseTwist_<std::allocator<void> > PoseTwist;

typedef boost::shared_ptr< ::magmed_msgs::PoseTwist > PoseTwistPtr;
typedef boost::shared_ptr< ::magmed_msgs::PoseTwist const> PoseTwistConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::magmed_msgs::PoseTwist_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::magmed_msgs::PoseTwist_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::magmed_msgs::PoseTwist_<ContainerAllocator1> & lhs, const ::magmed_msgs::PoseTwist_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.pose == rhs.pose &&
    lhs.twist == rhs.twist;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::magmed_msgs::PoseTwist_<ContainerAllocator1> & lhs, const ::magmed_msgs::PoseTwist_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace magmed_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::magmed_msgs::PoseTwist_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::magmed_msgs::PoseTwist_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::magmed_msgs::PoseTwist_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::magmed_msgs::PoseTwist_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::magmed_msgs::PoseTwist_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::magmed_msgs::PoseTwist_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::magmed_msgs::PoseTwist_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a296da29623d3a182b44fee95e8415a4";
  }

  static const char* value(const ::magmed_msgs::PoseTwist_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa296da29623d3a18ULL;
  static const uint64_t static_value2 = 0x2b44fee95e8415a4ULL;
};

template<class ContainerAllocator>
struct DataType< ::magmed_msgs::PoseTwist_<ContainerAllocator> >
{
  static const char* value()
  {
    return "magmed_msgs/PoseTwist";
  }

  static const char* value(const ::magmed_msgs::PoseTwist_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::magmed_msgs::PoseTwist_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"geometry_msgs/Pose pose \n"
"geometry_msgs/Twist twist\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::magmed_msgs::PoseTwist_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::magmed_msgs::PoseTwist_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pose);
      stream.next(m.twist);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PoseTwist_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::magmed_msgs::PoseTwist_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::magmed_msgs::PoseTwist_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAGMED_MSGS_MESSAGE_POSETWIST_H