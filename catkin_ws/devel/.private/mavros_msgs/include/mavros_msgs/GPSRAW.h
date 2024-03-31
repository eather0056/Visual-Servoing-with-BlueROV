// Generated by gencpp from file mavros_msgs/GPSRAW.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_GPSRAW_H
#define MAVROS_MSGS_MESSAGE_GPSRAW_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace mavros_msgs
{
template <class ContainerAllocator>
struct GPSRAW_
{
  typedef GPSRAW_<ContainerAllocator> Type;

  GPSRAW_()
    : header()
    , fix_type(0)
    , lat(0)
    , lon(0)
    , alt(0)
    , eph(0)
    , epv(0)
    , vel(0)
    , cog(0)
    , satellites_visible(0)
    , alt_ellipsoid(0)
    , h_acc(0)
    , v_acc(0)
    , vel_acc(0)
    , hdg_acc(0)
    , yaw(0)
    , dgps_numch(0)
    , dgps_age(0)  {
    }
  GPSRAW_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , fix_type(0)
    , lat(0)
    , lon(0)
    , alt(0)
    , eph(0)
    , epv(0)
    , vel(0)
    , cog(0)
    , satellites_visible(0)
    , alt_ellipsoid(0)
    , h_acc(0)
    , v_acc(0)
    , vel_acc(0)
    , hdg_acc(0)
    , yaw(0)
    , dgps_numch(0)
    , dgps_age(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _fix_type_type;
  _fix_type_type fix_type;

   typedef int32_t _lat_type;
  _lat_type lat;

   typedef int32_t _lon_type;
  _lon_type lon;

   typedef int32_t _alt_type;
  _alt_type alt;

   typedef uint16_t _eph_type;
  _eph_type eph;

   typedef uint16_t _epv_type;
  _epv_type epv;

   typedef uint16_t _vel_type;
  _vel_type vel;

   typedef uint16_t _cog_type;
  _cog_type cog;

   typedef uint8_t _satellites_visible_type;
  _satellites_visible_type satellites_visible;

   typedef int32_t _alt_ellipsoid_type;
  _alt_ellipsoid_type alt_ellipsoid;

   typedef uint32_t _h_acc_type;
  _h_acc_type h_acc;

   typedef uint32_t _v_acc_type;
  _v_acc_type v_acc;

   typedef uint32_t _vel_acc_type;
  _vel_acc_type vel_acc;

   typedef int32_t _hdg_acc_type;
  _hdg_acc_type hdg_acc;

   typedef uint16_t _yaw_type;
  _yaw_type yaw;

   typedef uint8_t _dgps_numch_type;
  _dgps_numch_type dgps_numch;

   typedef uint32_t _dgps_age_type;
  _dgps_age_type dgps_age;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(GPS_FIX_TYPE_NO_GPS)
  #undef GPS_FIX_TYPE_NO_GPS
#endif
#if defined(_WIN32) && defined(GPS_FIX_TYPE_NO_FIX)
  #undef GPS_FIX_TYPE_NO_FIX
#endif
#if defined(_WIN32) && defined(GPS_FIX_TYPE_2D_FIX)
  #undef GPS_FIX_TYPE_2D_FIX
#endif
#if defined(_WIN32) && defined(GPS_FIX_TYPE_3D_FIX)
  #undef GPS_FIX_TYPE_3D_FIX
#endif
#if defined(_WIN32) && defined(GPS_FIX_TYPE_DGPS)
  #undef GPS_FIX_TYPE_DGPS
#endif
#if defined(_WIN32) && defined(GPS_FIX_TYPE_RTK_FLOATR)
  #undef GPS_FIX_TYPE_RTK_FLOATR
#endif
#if defined(_WIN32) && defined(GPS_FIX_TYPE_RTK_FIXEDR)
  #undef GPS_FIX_TYPE_RTK_FIXEDR
#endif
#if defined(_WIN32) && defined(GPS_FIX_TYPE_STATIC)
  #undef GPS_FIX_TYPE_STATIC
#endif
#if defined(_WIN32) && defined(GPS_FIX_TYPE_PPP)
  #undef GPS_FIX_TYPE_PPP
#endif

  enum {
    GPS_FIX_TYPE_NO_GPS = 0u,
    GPS_FIX_TYPE_NO_FIX = 1u,
    GPS_FIX_TYPE_2D_FIX = 2u,
    GPS_FIX_TYPE_3D_FIX = 3u,
    GPS_FIX_TYPE_DGPS = 4u,
    GPS_FIX_TYPE_RTK_FLOATR = 5u,
    GPS_FIX_TYPE_RTK_FIXEDR = 6u,
    GPS_FIX_TYPE_STATIC = 7u,
    GPS_FIX_TYPE_PPP = 8u,
  };


  typedef boost::shared_ptr< ::mavros_msgs::GPSRAW_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::GPSRAW_<ContainerAllocator> const> ConstPtr;

}; // struct GPSRAW_

typedef ::mavros_msgs::GPSRAW_<std::allocator<void> > GPSRAW;

typedef boost::shared_ptr< ::mavros_msgs::GPSRAW > GPSRAWPtr;
typedef boost::shared_ptr< ::mavros_msgs::GPSRAW const> GPSRAWConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::GPSRAW_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::GPSRAW_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::GPSRAW_<ContainerAllocator1> & lhs, const ::mavros_msgs::GPSRAW_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.fix_type == rhs.fix_type &&
    lhs.lat == rhs.lat &&
    lhs.lon == rhs.lon &&
    lhs.alt == rhs.alt &&
    lhs.eph == rhs.eph &&
    lhs.epv == rhs.epv &&
    lhs.vel == rhs.vel &&
    lhs.cog == rhs.cog &&
    lhs.satellites_visible == rhs.satellites_visible &&
    lhs.alt_ellipsoid == rhs.alt_ellipsoid &&
    lhs.h_acc == rhs.h_acc &&
    lhs.v_acc == rhs.v_acc &&
    lhs.vel_acc == rhs.vel_acc &&
    lhs.hdg_acc == rhs.hdg_acc &&
    lhs.yaw == rhs.yaw &&
    lhs.dgps_numch == rhs.dgps_numch &&
    lhs.dgps_age == rhs.dgps_age;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::GPSRAW_<ContainerAllocator1> & lhs, const ::mavros_msgs::GPSRAW_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GPSRAW_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::GPSRAW_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GPSRAW_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::GPSRAW_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GPSRAW_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::GPSRAW_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::GPSRAW_<ContainerAllocator> >
{
  static const char* value()
  {
    return "58a85dbc1516a2d4302f256cca54bbbf";
  }

  static const char* value(const ::mavros_msgs::GPSRAW_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x58a85dbc1516a2d4ULL;
  static const uint64_t static_value2 = 0x302f256cca54bbbfULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::GPSRAW_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/GPSRAW";
  }

  static const char* value(const ::mavros_msgs::GPSRAW_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::GPSRAW_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# FCU GPS RAW message for the gps_status plugin\n"
"# A merge of <a href=\"https://mavlink.io/en/messages/common.html#GPS_RAW_INT\">mavlink GPS_RAW_INT</a> and \n"
"# <a href=\"https://mavlink.io/en/messages/common.html#GPS2_RAW\">mavlink GPS2_RAW</a> messages.\n"
"\n"
"std_msgs/Header header\n"
"## GPS_FIX_TYPE enum\n"
"uint8 GPS_FIX_TYPE_NO_GPS     = 0    # No GPS connected\n"
"uint8 GPS_FIX_TYPE_NO_FIX     = 1    # No position information, GPS is connected\n"
"uint8 GPS_FIX_TYPE_2D_FIX     = 2    # 2D position\n"
"uint8 GPS_FIX_TYPE_3D_FIX     = 3    # 3D position\n"
"uint8 GPS_FIX_TYPE_DGPS       = 4    # DGPS/SBAS aided 3D position\n"
"uint8 GPS_FIX_TYPE_RTK_FLOATR = 5    # TK float, 3D position\n"
"uint8 GPS_FIX_TYPE_RTK_FIXEDR = 6    # TK Fixed, 3D position\n"
"uint8 GPS_FIX_TYPE_STATIC     = 7    # Static fixed, typically used for base stations\n"
"uint8 GPS_FIX_TYPE_PPP        = 8    # PPP, 3D position\n"
"uint8 fix_type      # [GPS_FIX_TYPE] GPS fix type\n"
"\n"
"int32 lat           # [degE7] Latitude (WGS84, EGM96 ellipsoid)\n"
"int32 lon           # [degE7] Longitude (WGS84, EGM96 ellipsoid)\n"
"int32 alt           # [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.\n"
"uint16 eph          # GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX\n"
"uint16 epv          # GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX\n"
"uint16 vel          # [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX\n"
"uint16 cog          # [cdeg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX\n"
"uint8 satellites_visible # Number of satellites visible. If unknown, set to 255\n"
"\n"
"# -*- only available with MAVLink v2.0 and GPS_RAW_INT messages -*-\n"
"int32 alt_ellipsoid # [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.\n"
"uint32 h_acc        # [mm] Position uncertainty. Positive for up.\n"
"uint32 v_acc        # [mm] Altitude uncertainty. Positive for up.\n"
"uint32 vel_acc      # [mm] Speed uncertainty. Positive for up.\n"
"int32  hdg_acc      # [degE5] Heading / track uncertainty\n"
"uint16 yaw          # [cdeg] Yaw in earth frame from north.\n"
"\n"
"# -*- only available with MAVLink v2.0 and GPS2_RAW messages -*-\n"
"uint8 dgps_numch    # Number of DGPS satellites\n"
"uint32 dgps_age     # [ms] Age of DGPS info\n"
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

  static const char* value(const ::mavros_msgs::GPSRAW_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::GPSRAW_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.fix_type);
      stream.next(m.lat);
      stream.next(m.lon);
      stream.next(m.alt);
      stream.next(m.eph);
      stream.next(m.epv);
      stream.next(m.vel);
      stream.next(m.cog);
      stream.next(m.satellites_visible);
      stream.next(m.alt_ellipsoid);
      stream.next(m.h_acc);
      stream.next(m.v_acc);
      stream.next(m.vel_acc);
      stream.next(m.hdg_acc);
      stream.next(m.yaw);
      stream.next(m.dgps_numch);
      stream.next(m.dgps_age);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GPSRAW_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::GPSRAW_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::GPSRAW_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "fix_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fix_type);
    s << indent << "lat: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lat);
    s << indent << "lon: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lon);
    s << indent << "alt: ";
    Printer<int32_t>::stream(s, indent + "  ", v.alt);
    s << indent << "eph: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.eph);
    s << indent << "epv: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.epv);
    s << indent << "vel: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.vel);
    s << indent << "cog: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.cog);
    s << indent << "satellites_visible: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.satellites_visible);
    s << indent << "alt_ellipsoid: ";
    Printer<int32_t>::stream(s, indent + "  ", v.alt_ellipsoid);
    s << indent << "h_acc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.h_acc);
    s << indent << "v_acc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.v_acc);
    s << indent << "vel_acc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.vel_acc);
    s << indent << "hdg_acc: ";
    Printer<int32_t>::stream(s, indent + "  ", v.hdg_acc);
    s << indent << "yaw: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.yaw);
    s << indent << "dgps_numch: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dgps_numch);
    s << indent << "dgps_age: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.dgps_age);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_GPSRAW_H
