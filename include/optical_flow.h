// Generated by gencpp from file kari_estimator/optical_flow.msg
// DO NOT EDIT!


#ifndef KARI_ESTIMATOR_MESSAGE_OPTICAL_FLOW_H
#define KARI_ESTIMATOR_MESSAGE_OPTICAL_FLOW_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kari_estimator
{
template <class ContainerAllocator>
struct optical_flow_
{
  typedef optical_flow_<ContainerAllocator> Type;

  optical_flow_()
    : time_usec(0)
    , sensor_id(0)
    , integration_time_us(0)
    , integrated_x(0.0)
    , integrated_y(0.0)
    , integrated_xgyro(0.0)
    , integrated_ygyro(0.0)
    , integrated_zgyro(0.0)
    , temperature(0.0)
    , quality(0)
    , time_delta_distance_us(0)
    , distance(0.0)  {
    }
  optical_flow_(const ContainerAllocator& _alloc)
    : time_usec(0)
    , sensor_id(0)
    , integration_time_us(0)
    , integrated_x(0.0)
    , integrated_y(0.0)
    , integrated_xgyro(0.0)
    , integrated_ygyro(0.0)
    , integrated_zgyro(0.0)
    , temperature(0.0)
    , quality(0)
    , time_delta_distance_us(0)
    , distance(0.0)  {
  (void)_alloc;
    }



   typedef int64_t _time_usec_type;
  _time_usec_type time_usec;

   typedef int32_t _sensor_id_type;
  _sensor_id_type sensor_id;

   typedef int32_t _integration_time_us_type;
  _integration_time_us_type integration_time_us;

   typedef float _integrated_x_type;
  _integrated_x_type integrated_x;

   typedef float _integrated_y_type;
  _integrated_y_type integrated_y;

   typedef float _integrated_xgyro_type;
  _integrated_xgyro_type integrated_xgyro;

   typedef float _integrated_ygyro_type;
  _integrated_ygyro_type integrated_ygyro;

   typedef float _integrated_zgyro_type;
  _integrated_zgyro_type integrated_zgyro;

   typedef float _temperature_type;
  _temperature_type temperature;

   typedef int32_t _quality_type;
  _quality_type quality;

   typedef int32_t _time_delta_distance_us_type;
  _time_delta_distance_us_type time_delta_distance_us;

   typedef float _distance_type;
  _distance_type distance;





  typedef boost::shared_ptr< ::kari_estimator::optical_flow_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kari_estimator::optical_flow_<ContainerAllocator> const> ConstPtr;

}; // struct optical_flow_

typedef ::kari_estimator::optical_flow_<std::allocator<void> > optical_flow;

typedef boost::shared_ptr< ::kari_estimator::optical_flow > optical_flowPtr;
typedef boost::shared_ptr< ::kari_estimator::optical_flow const> optical_flowConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kari_estimator::optical_flow_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kari_estimator::optical_flow_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kari_estimator::optical_flow_<ContainerAllocator1> & lhs, const ::kari_estimator::optical_flow_<ContainerAllocator2> & rhs)
{
  return lhs.time_usec == rhs.time_usec &&
    lhs.sensor_id == rhs.sensor_id &&
    lhs.integration_time_us == rhs.integration_time_us &&
    lhs.integrated_x == rhs.integrated_x &&
    lhs.integrated_y == rhs.integrated_y &&
    lhs.integrated_xgyro == rhs.integrated_xgyro &&
    lhs.integrated_ygyro == rhs.integrated_ygyro &&
    lhs.integrated_zgyro == rhs.integrated_zgyro &&
    lhs.temperature == rhs.temperature &&
    lhs.quality == rhs.quality &&
    lhs.time_delta_distance_us == rhs.time_delta_distance_us &&
    lhs.distance == rhs.distance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kari_estimator::optical_flow_<ContainerAllocator1> & lhs, const ::kari_estimator::optical_flow_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kari_estimator

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kari_estimator::optical_flow_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kari_estimator::optical_flow_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_estimator::optical_flow_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kari_estimator::optical_flow_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_estimator::optical_flow_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kari_estimator::optical_flow_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kari_estimator::optical_flow_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c18eece5edacfefd28f611b294cc7ce";
  }

  static const char* value(const ::kari_estimator::optical_flow_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9c18eece5edacfefULL;
  static const uint64_t static_value2 = 0xd28f611b294cc7ceULL;
};

template<class ContainerAllocator>
struct DataType< ::kari_estimator::optical_flow_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kari_estimator/optical_flow";
  }

  static const char* value(const ::kari_estimator::optical_flow_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kari_estimator::optical_flow_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 time_usec\n"
"int32 sensor_id\n"
"int32 integration_time_us\n"
"float32 integrated_x\n"
"float32 integrated_y\n"
"float32 integrated_xgyro\n"
"float32 integrated_ygyro\n"
"float32 integrated_zgyro\n"
"float32 temperature\n"
"int32 quality\n"
"int32 time_delta_distance_us\n"
"float32 distance\n"
;
  }

  static const char* value(const ::kari_estimator::optical_flow_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kari_estimator::optical_flow_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time_usec);
      stream.next(m.sensor_id);
      stream.next(m.integration_time_us);
      stream.next(m.integrated_x);
      stream.next(m.integrated_y);
      stream.next(m.integrated_xgyro);
      stream.next(m.integrated_ygyro);
      stream.next(m.integrated_zgyro);
      stream.next(m.temperature);
      stream.next(m.quality);
      stream.next(m.time_delta_distance_us);
      stream.next(m.distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct optical_flow_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kari_estimator::optical_flow_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kari_estimator::optical_flow_<ContainerAllocator>& v)
  {
    s << indent << "time_usec: ";
    Printer<int64_t>::stream(s, indent + "  ", v.time_usec);
    s << indent << "sensor_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.sensor_id);
    s << indent << "integration_time_us: ";
    Printer<int32_t>::stream(s, indent + "  ", v.integration_time_us);
    s << indent << "integrated_x: ";
    Printer<float>::stream(s, indent + "  ", v.integrated_x);
    s << indent << "integrated_y: ";
    Printer<float>::stream(s, indent + "  ", v.integrated_y);
    s << indent << "integrated_xgyro: ";
    Printer<float>::stream(s, indent + "  ", v.integrated_xgyro);
    s << indent << "integrated_ygyro: ";
    Printer<float>::stream(s, indent + "  ", v.integrated_ygyro);
    s << indent << "integrated_zgyro: ";
    Printer<float>::stream(s, indent + "  ", v.integrated_zgyro);
    s << indent << "temperature: ";
    Printer<float>::stream(s, indent + "  ", v.temperature);
    s << indent << "quality: ";
    Printer<int32_t>::stream(s, indent + "  ", v.quality);
    s << indent << "time_delta_distance_us: ";
    Printer<int32_t>::stream(s, indent + "  ", v.time_delta_distance_us);
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARI_ESTIMATOR_MESSAGE_OPTICAL_FLOW_H