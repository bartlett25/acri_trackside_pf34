// Generated by gencpp from file acri_localisation/railLine.msg
// DO NOT EDIT!


#ifndef ACRI_LOCALISATION_MESSAGE_RAILLINE_H
#define ACRI_LOCALISATION_MESSAGE_RAILLINE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace acri_localisation
{
template <class ContainerAllocator>
struct railLine_
{
  typedef railLine_<ContainerAllocator> Type;

  railLine_()
    : point1()
    , point2()  {
    }
  railLine_(const ContainerAllocator& _alloc)
    : point1(_alloc)
    , point2(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _point1_type;
  _point1_type point1;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _point2_type;
  _point2_type point2;





  typedef boost::shared_ptr< ::acri_localisation::railLine_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::acri_localisation::railLine_<ContainerAllocator> const> ConstPtr;

}; // struct railLine_

typedef ::acri_localisation::railLine_<std::allocator<void> > railLine;

typedef boost::shared_ptr< ::acri_localisation::railLine > railLinePtr;
typedef boost::shared_ptr< ::acri_localisation::railLine const> railLineConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::acri_localisation::railLine_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::acri_localisation::railLine_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::acri_localisation::railLine_<ContainerAllocator1> & lhs, const ::acri_localisation::railLine_<ContainerAllocator2> & rhs)
{
  return lhs.point1 == rhs.point1 &&
    lhs.point2 == rhs.point2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::acri_localisation::railLine_<ContainerAllocator1> & lhs, const ::acri_localisation::railLine_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace acri_localisation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::acri_localisation::railLine_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::acri_localisation::railLine_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::acri_localisation::railLine_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::acri_localisation::railLine_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::acri_localisation::railLine_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::acri_localisation::railLine_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::acri_localisation::railLine_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e377648347b19d625c7a86b684f82b75";
  }

  static const char* value(const ::acri_localisation::railLine_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe377648347b19d62ULL;
  static const uint64_t static_value2 = 0x5c7a86b684f82b75ULL;
};

template<class ContainerAllocator>
struct DataType< ::acri_localisation::railLine_<ContainerAllocator> >
{
  static const char* value()
  {
    return "acri_localisation/railLine";
  }

  static const char* value(const ::acri_localisation::railLine_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::acri_localisation::railLine_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point point1\n"
"geometry_msgs/Point point2\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::acri_localisation::railLine_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::acri_localisation::railLine_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point1);
      stream.next(m.point2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct railLine_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::acri_localisation::railLine_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::acri_localisation::railLine_<ContainerAllocator>& v)
  {
    s << indent << "point1: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.point1);
    s << indent << "point2: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.point2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ACRI_LOCALISATION_MESSAGE_RAILLINE_H
