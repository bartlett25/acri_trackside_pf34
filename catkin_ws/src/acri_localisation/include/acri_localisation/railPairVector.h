// Generated by gencpp from file acri_localisation/railPairVector.msg
// DO NOT EDIT!


#ifndef ACRI_LOCALISATION_MESSAGE_RAILPAIRVECTOR_H
#define ACRI_LOCALISATION_MESSAGE_RAILPAIRVECTOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <acri_localisation/railPair.h>

namespace acri_localisation
{
template <class ContainerAllocator>
struct railPairVector_
{
  typedef railPairVector_<ContainerAllocator> Type;

  railPairVector_()
    : header()
    , pairs()  {
    }
  railPairVector_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pairs(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::acri_localisation::railPair_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::acri_localisation::railPair_<ContainerAllocator> >::other >  _pairs_type;
  _pairs_type pairs;





  typedef boost::shared_ptr< ::acri_localisation::railPairVector_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::acri_localisation::railPairVector_<ContainerAllocator> const> ConstPtr;

}; // struct railPairVector_

typedef ::acri_localisation::railPairVector_<std::allocator<void> > railPairVector;

typedef boost::shared_ptr< ::acri_localisation::railPairVector > railPairVectorPtr;
typedef boost::shared_ptr< ::acri_localisation::railPairVector const> railPairVectorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::acri_localisation::railPairVector_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::acri_localisation::railPairVector_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::acri_localisation::railPairVector_<ContainerAllocator1> & lhs, const ::acri_localisation::railPairVector_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.pairs == rhs.pairs;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::acri_localisation::railPairVector_<ContainerAllocator1> & lhs, const ::acri_localisation::railPairVector_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace acri_localisation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::acri_localisation::railPairVector_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::acri_localisation::railPairVector_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::acri_localisation::railPairVector_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::acri_localisation::railPairVector_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::acri_localisation::railPairVector_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::acri_localisation::railPairVector_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::acri_localisation::railPairVector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "45d6f0b4412639e96eb4e308a8b6852c";
  }

  static const char* value(const ::acri_localisation::railPairVector_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x45d6f0b4412639e9ULL;
  static const uint64_t static_value2 = 0x6eb4e308a8b6852cULL;
};

template<class ContainerAllocator>
struct DataType< ::acri_localisation::railPairVector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "acri_localisation/railPairVector";
  }

  static const char* value(const ::acri_localisation::railPairVector_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::acri_localisation::railPairVector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"acri_localisation/railPair[] pairs\n"
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
"MSG: acri_localisation/railPair\n"
"acri_localisation/railLine line1\n"
"acri_localisation/railLine line2\n"
"acri_localisation/railLine midline\n"
"================================================================================\n"
"MSG: acri_localisation/railLine\n"
"geometry_msgs/Point point1\n"
"geometry_msgs/Point point2\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::acri_localisation::railPairVector_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::acri_localisation::railPairVector_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pairs);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct railPairVector_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::acri_localisation::railPairVector_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::acri_localisation::railPairVector_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pairs[]" << std::endl;
    for (size_t i = 0; i < v.pairs.size(); ++i)
    {
      s << indent << "  pairs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::acri_localisation::railPair_<ContainerAllocator> >::stream(s, indent + "    ", v.pairs[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ACRI_LOCALISATION_MESSAGE_RAILPAIRVECTOR_H