// Generated by gencpp from file dynamixel_sdk_examples/SyncGetPosition.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_SDK_EXAMPLES_MESSAGE_SYNCGETPOSITION_H
#define DYNAMIXEL_SDK_EXAMPLES_MESSAGE_SYNCGETPOSITION_H

#include <ros/service_traits.h>


#include <dynamixel_sdk_examples/SyncGetPositionRequest.h>
#include <dynamixel_sdk_examples/SyncGetPositionResponse.h>


namespace dynamixel_sdk_examples
{

struct SyncGetPosition
{

typedef SyncGetPositionRequest Request;
typedef SyncGetPositionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SyncGetPosition
} // namespace dynamixel_sdk_examples


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dynamixel_sdk_examples::SyncGetPosition > {
  static const char* value()
  {
    return "4a5a90af26ce6360c45e1980f134941b";
  }

  static const char* value(const ::dynamixel_sdk_examples::SyncGetPosition&) { return value(); }
};

template<>
struct DataType< ::dynamixel_sdk_examples::SyncGetPosition > {
  static const char* value()
  {
    return "dynamixel_sdk_examples/SyncGetPosition";
  }

  static const char* value(const ::dynamixel_sdk_examples::SyncGetPosition&) { return value(); }
};


// service_traits::MD5Sum< ::dynamixel_sdk_examples::SyncGetPositionRequest> should match
// service_traits::MD5Sum< ::dynamixel_sdk_examples::SyncGetPosition >
template<>
struct MD5Sum< ::dynamixel_sdk_examples::SyncGetPositionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_sdk_examples::SyncGetPosition >::value();
  }
  static const char* value(const ::dynamixel_sdk_examples::SyncGetPositionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_sdk_examples::SyncGetPositionRequest> should match
// service_traits::DataType< ::dynamixel_sdk_examples::SyncGetPosition >
template<>
struct DataType< ::dynamixel_sdk_examples::SyncGetPositionRequest>
{
  static const char* value()
  {
    return DataType< ::dynamixel_sdk_examples::SyncGetPosition >::value();
  }
  static const char* value(const ::dynamixel_sdk_examples::SyncGetPositionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dynamixel_sdk_examples::SyncGetPositionResponse> should match
// service_traits::MD5Sum< ::dynamixel_sdk_examples::SyncGetPosition >
template<>
struct MD5Sum< ::dynamixel_sdk_examples::SyncGetPositionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_sdk_examples::SyncGetPosition >::value();
  }
  static const char* value(const ::dynamixel_sdk_examples::SyncGetPositionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_sdk_examples::SyncGetPositionResponse> should match
// service_traits::DataType< ::dynamixel_sdk_examples::SyncGetPosition >
template<>
struct DataType< ::dynamixel_sdk_examples::SyncGetPositionResponse>
{
  static const char* value()
  {
    return DataType< ::dynamixel_sdk_examples::SyncGetPosition >::value();
  }
  static const char* value(const ::dynamixel_sdk_examples::SyncGetPositionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DYNAMIXEL_SDK_EXAMPLES_MESSAGE_SYNCGETPOSITION_H