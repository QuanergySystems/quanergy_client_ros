/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_ROS_SENSOR_CLIENT_H
#define QUANERGY_CLIENT_ROS_SENSOR_CLIENT_H

#include <quanergy/parsers/failover_client.h>
#include <quanergy/parsers/pointcloud_generator_01.h>
#include <quanergy/parsers/pointcloud_generator_00.h>

#include <quanergy/modules/polar_to_cart_converter.h>
#include <quanergy/modules/distance_filter.h>
#include <quanergy/modules/ring_intensity_filter.h>


struct SensorClient {

  typedef std::shared_ptr<SensorClient> Ptr;

  /// FailoverClient adds a failover to old M8 data
  typedef quanergy::client::FailoverClient<quanergy::client::DataPacket01, quanergy::client::DataPacket00> ClientType;

  typedef quanergy::client::DistanceFilter DistanceFilter;
  typedef quanergy::client::RingIntensityFilter RingIntensityFilter;
  typedef quanergy::client::PolarToCartConverter PolarToCartConverter;

  typedef boost::signals2::signal<void (quanergy::PointCloudXYZIRPtr const &)> Signal;

  SensorClient(std::string const & host,
                 std::string const & port, 
                 std::string const & frame_id = std::string());

  ~SensorClient();

  boost::signals2::connection connect(const typename Signal::slot_type & subscriber);

  void run();
  void stop();

  void setMaximumDistanceThreshold(float maxThreshold);
  float getMaximumDistanceThreshold() const;

  void setMinimumDistanceThreshold(float minThreshold);
  float getMinimumDistanceThreshold() const;


  void setRingFilterMinimumRangeThreshold(const std::uint16_t laser_beam, const float min_threshold);
  float getRingFilterMinimumRangeThreshold(const std::uint16_t laser_beam) const;

  void setRingFilterMinimumIntensityThreshold(const uint16_t laser_beam, const uint8_t min_threshold);
  uint8_t getRingFilterMinimumIntensityThreshold(const std::uint16_t laser_beam) const;


private:

  ClientType client_;
  DistanceFilter distance_filter_;
  /// ring filter only relevant for M8
  RingIntensityFilter ring_intensity_filter_;
  PolarToCartConverter polar_to_cart_converter_;

  boost::signals2::connection client_connection_;
  boost::signals2::connection distance_filter_connection_;
  boost::signals2::connection ring_intensity_filter_connection_;
};


#endif
