/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_ROS_CLIENT_NODE_H
#define QUANERGY_CLIENT_ROS_CLIENT_NODE_H

#include <string>
#include <cstdint>

#include <quanergy_client_ros/simple_publisher.h>

// client; failover adds support for old M8 data
#include <quanergy/client/failover_client.h>

// parsers for the data packets we want to support
#include <quanergy/parsers/data_packet_parser_00.h>
#include <quanergy/parsers/data_packet_parser_01.h>
#include <quanergy/parsers/data_packet_parser_failover.h>

// filters
#include <quanergy/modules/distance_filter.h>
#include <quanergy/modules/ring_intensity_filter.h>

// conversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>

struct ClientNode
{
  /// FailoverClient allows packets to pass through that don't have a header (for old M8 data)
  typedef quanergy::client::FailoverClient ClientType;
  typedef quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr,   // return type
                                                 quanergy::client::DataPacketParserFailover, // following are data packet types
                                                 quanergy::client::DataPacketParser00,
                                                 quanergy::client::DataPacketParser01> ParserType;
  typedef quanergy::client::PacketParserModule<ParserType> ParserModuleType;
  typedef quanergy::client::DistanceFilter DistanceFilter;
  typedef quanergy::client::RingIntensityFilter RingIntensityFilter;
  typedef quanergy::client::PolarToCartConverter ConverterType;

  ClientNode(int argc, char** argv);

  void run();

private:

  void loadSettings(int argc, char ** argv);
  void parseArgs(int argc, char ** argv);

  // settings
  // Defaults overridded by settings file which is then
  // overridden by command-line options.
  struct
  {
    // Distance filter
    float min = 0.5f;
    float max = 500.f;

    // Ring filter
    // Only set by config file
    // only relevant for M8
    float default_ring_range = 0.f;
    std::uint16_t default_ring_intensity = 0;

    float ring_range[quanergy::client::M8_NUM_LASERS];
    std::uint16_t ring_intensity[quanergy::client::M8_NUM_LASERS];

    // Client
    std::string host = "10.0.0.3";

    std::string topic = "points";
    std::string frame = "sensor";

    bool useRosTime = false;
  } settings_;
};


#endif
