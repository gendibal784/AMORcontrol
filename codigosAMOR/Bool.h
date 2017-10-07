// This is an automatically generated file.
// Generated from this Bool.msg definition:
//   [std_msgs/Bool]:
//   bool data
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_Bool
#define YARPMSG_TYPE_Bool

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class Bool : public yarp::os::idl::WirePortable {
public:
  bool data;

  Bool() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** data ***
    if (!connection.expectBlock((char*)&data,1)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(1)) return false;

    // *** data ***
    data = reader.expectInt();
    return !connection.isError();
  }

  using yarp::os::idl::WirePortable::read;
  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** data ***
    connection.appendBlock((char*)&data,1);
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(1);

    // *** data ***
    connection.appendInt(BOTTLE_TAG_INT);
    connection.appendInt((int)data);
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
  typedef yarp::os::idl::BareStyle<Bool> rosStyle;
  typedef yarp::os::idl::BottleStyle<Bool> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "[std_msgs/Bool]:\n\
bool data";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("Bool","Bool");
    typ.addProperty("md5sum",yarp::os::Value("8b94c1b53db61fb6aed406028ad6332a"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
