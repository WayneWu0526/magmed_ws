// Generated by gencpp from file magmed_msgs/GetRobotStateRequest.msg
// DO NOT EDIT!


#ifndef MAGMED_MSGS_MESSAGE_GETROBOTSTATEREQUEST_H
#define MAGMED_MSGS_MESSAGE_GETROBOTSTATEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace magmed_msgs
{
template <class ContainerAllocator>
struct GetRobotStateRequest_
{
  typedef GetRobotStateRequest_<ContainerAllocator> Type;

  GetRobotStateRequest_()
    : request(0)  {
    }
  GetRobotStateRequest_(const ContainerAllocator& _alloc)
    : request(0)  {
  (void)_alloc;
    }



   typedef int32_t _request_type;
  _request_type request;





  typedef boost::shared_ptr< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetRobotStateRequest_

typedef ::magmed_msgs::GetRobotStateRequest_<std::allocator<void> > GetRobotStateRequest;

typedef boost::shared_ptr< ::magmed_msgs::GetRobotStateRequest > GetRobotStateRequestPtr;
typedef boost::shared_ptr< ::magmed_msgs::GetRobotStateRequest const> GetRobotStateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator1> & lhs, const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator2> & rhs)
{
  return lhs.request == rhs.request;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator1> & lhs, const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace magmed_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "650f0ccd41c8f8d53ada80be6ddde434";
  }

  static const char* value(const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x650f0ccd41c8f8d5ULL;
  static const uint64_t static_value2 = 0x3ada80be6ddde434ULL;
};

template<class ContainerAllocator>
struct DataType< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "magmed_msgs/GetRobotStateRequest";
  }

  static const char* value(const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# GetRobotState.srv\n"
"int32 request\n"
;
  }

  static const char* value(const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.request);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetRobotStateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::magmed_msgs::GetRobotStateRequest_<ContainerAllocator>& v)
  {
    s << indent << "request: ";
    Printer<int32_t>::stream(s, indent + "  ", v.request);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAGMED_MSGS_MESSAGE_GETROBOTSTATEREQUEST_H