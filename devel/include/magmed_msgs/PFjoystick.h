// Generated by gencpp from file magmed_msgs/PFjoystick.msg
// DO NOT EDIT!


#ifndef MAGMED_MSGS_MESSAGE_PFJOYSTICK_H
#define MAGMED_MSGS_MESSAGE_PFJOYSTICK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace magmed_msgs
{
template <class ContainerAllocator>
struct PFjoystick_
{
  typedef PFjoystick_<ContainerAllocator> Type;

  PFjoystick_()
    : header()
    , nJOY1()
    , nJOY2()
    , nJOY3()
    , bJOYD(false)
    , POTA(0)
    , POTB(0)
    , BANA(0)
    , BANB(0)
    , ENCA(0)
    , ENCB(0)
    , TOG()
    , BUT()  {
      nJOY1.assign(0.0);

      nJOY2.assign(0.0);

      nJOY3.assign(0.0);

      TOG.assign(false);

      BUT.assign(false);
  }
  PFjoystick_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , nJOY1()
    , nJOY2()
    , nJOY3()
    , bJOYD(false)
    , POTA(0)
    , POTB(0)
    , BANA(0)
    , BANB(0)
    , ENCA(0)
    , ENCB(0)
    , TOG()
    , BUT()  {
  (void)_alloc;
      nJOY1.assign(0.0);

      nJOY2.assign(0.0);

      nJOY3.assign(0.0);

      TOG.assign(false);

      BUT.assign(false);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<float, 3>  _nJOY1_type;
  _nJOY1_type nJOY1;

   typedef boost::array<float, 2>  _nJOY2_type;
  _nJOY2_type nJOY2;

   typedef boost::array<float, 2>  _nJOY3_type;
  _nJOY3_type nJOY3;

   typedef uint8_t _bJOYD_type;
  _bJOYD_type bJOYD;

   typedef uint16_t _POTA_type;
  _POTA_type POTA;

   typedef uint16_t _POTB_type;
  _POTB_type POTB;

   typedef int32_t _BANA_type;
  _BANA_type BANA;

   typedef int32_t _BANB_type;
  _BANB_type BANB;

   typedef int16_t _ENCA_type;
  _ENCA_type ENCA;

   typedef int16_t _ENCB_type;
  _ENCB_type ENCB;

   typedef boost::array<uint8_t, 5>  _TOG_type;
  _TOG_type TOG;

   typedef boost::array<uint8_t, 6>  _BUT_type;
  _BUT_type BUT;





  typedef boost::shared_ptr< ::magmed_msgs::PFjoystick_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::magmed_msgs::PFjoystick_<ContainerAllocator> const> ConstPtr;

}; // struct PFjoystick_

typedef ::magmed_msgs::PFjoystick_<std::allocator<void> > PFjoystick;

typedef boost::shared_ptr< ::magmed_msgs::PFjoystick > PFjoystickPtr;
typedef boost::shared_ptr< ::magmed_msgs::PFjoystick const> PFjoystickConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::magmed_msgs::PFjoystick_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::magmed_msgs::PFjoystick_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::magmed_msgs::PFjoystick_<ContainerAllocator1> & lhs, const ::magmed_msgs::PFjoystick_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.nJOY1 == rhs.nJOY1 &&
    lhs.nJOY2 == rhs.nJOY2 &&
    lhs.nJOY3 == rhs.nJOY3 &&
    lhs.bJOYD == rhs.bJOYD &&
    lhs.POTA == rhs.POTA &&
    lhs.POTB == rhs.POTB &&
    lhs.BANA == rhs.BANA &&
    lhs.BANB == rhs.BANB &&
    lhs.ENCA == rhs.ENCA &&
    lhs.ENCB == rhs.ENCB &&
    lhs.TOG == rhs.TOG &&
    lhs.BUT == rhs.BUT;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::magmed_msgs::PFjoystick_<ContainerAllocator1> & lhs, const ::magmed_msgs::PFjoystick_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace magmed_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::magmed_msgs::PFjoystick_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::magmed_msgs::PFjoystick_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::magmed_msgs::PFjoystick_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::magmed_msgs::PFjoystick_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::magmed_msgs::PFjoystick_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::magmed_msgs::PFjoystick_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::magmed_msgs::PFjoystick_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c1b2838b51e4cc36d6636da93093d28d";
  }

  static const char* value(const ::magmed_msgs::PFjoystick_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc1b2838b51e4cc36ULL;
  static const uint64_t static_value2 = 0xd6636da93093d28dULL;
};

template<class ContainerAllocator>
struct DataType< ::magmed_msgs::PFjoystick_<ContainerAllocator> >
{
  static const char* value()
  {
    return "magmed_msgs/PFjoystick";
  }

  static const char* value(const ::magmed_msgs::PFjoystick_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::magmed_msgs::PFjoystick_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header              # ROS standard header\n"
"\n"
"float32[3] nJOY1             # Three axes of the big joystick\n"
"float32[2] nJOY2             # Two axes of the first (left) small joystick\n"
"float32[2] nJOY3             # Two axes of the second (right) small joystick\n"
"bool bJOYD                 # Big joystick button\n"
"uint16 POTA                # Potentiometer A\n"
"uint16 POTB                # Potentiometer B\n"
"int32 BANA                 # Rotary switch A\n"
"int32 BANB                 # Rotary switch B\n"
"int16 ENCA                 # Encoder A\n"
"int16 ENCB                 # Encoder B\n"
"bool[5] TOG                # Toggle switches (5 in total)\n"
"bool[6] BUT                # Push buttons (6 in total)\n"
"\n"
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
;
  }

  static const char* value(const ::magmed_msgs::PFjoystick_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::magmed_msgs::PFjoystick_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.nJOY1);
      stream.next(m.nJOY2);
      stream.next(m.nJOY3);
      stream.next(m.bJOYD);
      stream.next(m.POTA);
      stream.next(m.POTB);
      stream.next(m.BANA);
      stream.next(m.BANB);
      stream.next(m.ENCA);
      stream.next(m.ENCB);
      stream.next(m.TOG);
      stream.next(m.BUT);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PFjoystick_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::magmed_msgs::PFjoystick_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::magmed_msgs::PFjoystick_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "nJOY1[]" << std::endl;
    for (size_t i = 0; i < v.nJOY1.size(); ++i)
    {
      s << indent << "  nJOY1[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.nJOY1[i]);
    }
    s << indent << "nJOY2[]" << std::endl;
    for (size_t i = 0; i < v.nJOY2.size(); ++i)
    {
      s << indent << "  nJOY2[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.nJOY2[i]);
    }
    s << indent << "nJOY3[]" << std::endl;
    for (size_t i = 0; i < v.nJOY3.size(); ++i)
    {
      s << indent << "  nJOY3[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.nJOY3[i]);
    }
    s << indent << "bJOYD: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.bJOYD);
    s << indent << "POTA: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.POTA);
    s << indent << "POTB: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.POTB);
    s << indent << "BANA: ";
    Printer<int32_t>::stream(s, indent + "  ", v.BANA);
    s << indent << "BANB: ";
    Printer<int32_t>::stream(s, indent + "  ", v.BANB);
    s << indent << "ENCA: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ENCA);
    s << indent << "ENCB: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ENCB);
    s << indent << "TOG[]" << std::endl;
    for (size_t i = 0; i < v.TOG.size(); ++i)
    {
      s << indent << "  TOG[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.TOG[i]);
    }
    s << indent << "BUT[]" << std::endl;
    for (size_t i = 0; i < v.BUT.size(); ++i)
    {
      s << indent << "  BUT[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.BUT[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAGMED_MSGS_MESSAGE_PFJOYSTICK_H
