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

#include <quanergy/client/version.h>

#if (QUANERGY_CLIENT_VERSION/100000 != 3)
  #error Incompatible Quanergy Client Version; looking for v3.x.x
#endif

#include <quanergy/client/sensor_client.h>

// parsers for the data packets we want to support
#include <quanergy/parsers/variadic_packet_parser.h>
#include <quanergy/parsers/data_packet_parser_00.h>
#include <quanergy/parsers/data_packet_parser_01.h>
#include <quanergy/parsers/data_packet_parser_04.h>

// filters
#include <quanergy/modules/distance_filter.h>
#include <quanergy/modules/ring_intensity_filter.h>

// conversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>

// module to apply encoder correction
#include <quanergy/modules/encoder_angle_calibration.h>

struct ClientNode
{
  typedef quanergy::client::SensorClient ClientType;
  typedef quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr,                      // return type
                                                 quanergy::client::DataPacketParser00,              // PARSER_00_INDEX
                                                 quanergy::client::DataPacketParser01,              // PARSER_01_INDEX
                                                 quanergy::client::DataPacketParser04> ParserType;  // PARSER_04_INDEX
  enum
  {
    PARSER_00_INDEX = 0,
    PARSER_01_INDEX = 1,
    PARSER_04_INDEX = 2
  };

  typedef quanergy::client::PacketParserModule<ParserType> ParserModuleType;
  typedef quanergy::calibration::EncoderAngleCalibration EncoderAngleCalibrationType;
  typedef quanergy::client::DistanceFilter DistanceFilter;
  typedef quanergy::client::RingIntensityFilter RingIntensityFilter;
  typedef quanergy::client::PolarToCartConverter ConverterType;
  
  ClientNode(int argc, char** argv);
  
  /// check whether valid arguments are provided and print usage if not
  static bool checkArgs(int argc, char** argv);

  void run();

private:
  // settings
  // Defaults overridded by settings file which is then
  // overridden by command-line options.
  struct Settings
  {
    // Distance filter
    float min = 0.5f;
    float max = 500.f;

    // Ring filter
    // Only set by config file
    // only relevant for M8
    const float default_ring_range = 0.f; // not exposed to user
    const std::uint16_t default_ring_intensity = 0; // not exposed to user

    float ring_range[quanergy::client::M8_NUM_LASERS];
    std::uint16_t ring_intensity[quanergy::client::M8_NUM_LASERS];
    
    // return selection
    // only applies to DataPacket00 (newer M8 data)
    // setting and argument both look for strings: all or an integer between 0 and quanergy::client::M8_NUM_RETURNS-1
    int return_selection = quanergy::client::ALL_RETURNS;

    // Encoder correction terms:
    float amplitude = 0.f;
    float phase = 0.f; // in radians

    // Client
    std::string host = "10.0.0.3";
    const std::string port = "4141"; // not exposed to user

    std::string topic = "points";
    std::string frame = "sensor";

    bool useRosTime = false;

    std::int32_t minCloudSize = 1500*quanergy::client::M8_NUM_LASERS;
    std::int32_t maxCloudSize = 11000*quanergy::client::M8_NUM_LASERS;

  } settings_;

  void loadSettings(int argc, char ** argv);
  void parseArgs(int argc, char ** argv);
  int returnFromString(const std::string& r);

  // Mutex for accessing the client
  std::mutex client_mutex_;
  // Whether or not to publish each return on a separate topic
  bool separate_return_topics_ = false;
  // The various steps of reading data from a parser bundled into a
  // structure
  struct SensorPipelineModules
  {
    using Ptr = std::unique_ptr<SensorPipelineModules>;

    ParserModuleType parser;
    EncoderAngleCalibrationType encoder_corrector;
    DistanceFilter distance_filter;
    RingIntensityFilter ring_intensity_filter;
    ConverterType cartesian_converter;
    SimplePublisher<quanergy::PointXYZIR> publisher;

    std::vector<boost::signals2::connection> connections;

    SensorPipelineModules(
      const Settings &settings,
      int return_selection,
      ClientType &client,
      bool add_return_number_to_topic = false
    );
    ~SensorPipelineModules();

    void run();
    
  private:  
    std::string ros_topic_name_;
  };
 
  // Block until publishers in all pipelines are ready 
  void waitForPublisherStartup(
    const std::vector<SensorPipelineModules::Ptr>& pipelines
  );
};


#endif
