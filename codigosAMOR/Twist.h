// This is an automatically generated file.
// Generated from this Twist.msg definition:
//   [geometry_msgs/Twist]:
//   # This expresses velocity in free space broken into its linear and angular parts.
//   Vector3  linear
//   Vector3  angular
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_Twist
#define YARPMSG_TYPE_Twist

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include "Vector3.h"

class Twist : public yarp::os::idl::WirePortable {
public:
  Vector3 linear;
  Vector3 angular;

  Twist() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** linear ***
    if (!linear.read(connection)) return false;

    // *** angular ***
    if (!angular.read(connection)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(2)) return false;

    // *** linear ***
    if (!linear.read(connection)) return false;

    // *** angular ***
    if (!angular.read(connection)) return false;
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** linear ***
    if (!linear.write(connection)) return false;

    // *** angular ***
    if (!angular.write(connection)) return false;
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(2);

    // *** linear ***
    if (!linear.write(connection)) return false;

    // *** angular ***
    if (!angular.write(connection)) return false;
    connection.convertTextMode();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::write;
  bool write(yarp::os::ConnectionWriter& connection) {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<Twist> rosStyle;
  typedef yarp::os::idl::BottleStyle<Twist> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "[geometry_msgs/Twist]:\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n================================================================================\n\
MSG: Vector3\n\
[geometry_msgs/Vector3]:\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("Twist","Twist");
    typ.addProperty("md5sum",yarp::os::Value("9f195f881246fdfa2798d1d3eebca84a"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
