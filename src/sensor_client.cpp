/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include "m8_sensor_client.h"


M8SensorClient::M8SensorClient(std::string const & host, 
                               std::string const & port, 
                               std::string const & frame_id)
  : client_(host, port, frame_id) 
{
  client_connection_ = client_.connect([this](ClientType::Result const & pc)
                                       { this->distance_filter_.slot(pc); } );

  distance_filter_connection_ = distance_filter_.connect([this](DistanceFilter::Result const & pc)
                                                         {this->ring_intensity_filter_.slot(pc); } );

  ring_intensity_filter_connection_ = ring_intensity_filter_.connect([this](RingIntensityFilter::Result const & pc)
                                                                     {this->polar_to_cart_converter_.slot(pc); } );
}


M8SensorClient::~M8SensorClient()
{
  client_connection_.disconnect();
  distance_filter_connection_.disconnect();
  ring_intensity_filter_connection_.disconnect();
}


boost::signals2::connection M8SensorClient::connect(const typename Signal::slot_type & subscriber) 
{
  return polar_to_cart_converter_.connect(subscriber);
}


void M8SensorClient::run() 
{
  client_.run();
}


void M8SensorClient::stop() 
{
  client_.stop();
}


void M8SensorClient::setMaximumDistanceThreshold(float maxThreshold)
{
  distance_filter_.setMaximumDistanceThreshold(maxThreshold);
}


float M8SensorClient::getMaximumDistanceThreshold() const
{
  return distance_filter_.getMaximumDistanceThreshold();
}


void M8SensorClient::setMinimumDistanceThreshold(float minThreshold)
{
  distance_filter_.setMinimumDistanceThreshold(minThreshold);
}


float M8SensorClient::getMinimumDistanceThreshold() const
{
  return distance_filter_.getMinimumDistanceThreshold();
}


void M8SensorClient::setRingFilterMinimumRangeThreshold(const std::uint16_t laser_beam, const float min_threshold)
{
  ring_intensity_filter_.setRingFilterMinimumRangeThreshold(laser_beam, min_threshold);
}

  
float M8SensorClient::getRingFilterMinimumRangeThreshold(const std::uint16_t laser_beam) const
{
  return ring_intensity_filter_.getRingFilterMinimumRangeThreshold(laser_beam);
}


void M8SensorClient::setRingFilterMinimumIntensityThreshold(const uint16_t laser_beam, const uint8_t min_threshold)
{
  ring_intensity_filter_.setRingFilterMinimumIntensityThreshold(laser_beam, min_threshold);
}


uint8_t M8SensorClient::getRingFilterMinimumIntensityThreshold(const std::uint16_t laser_beam) const
{
  return ring_intensity_filter_.getRingFilterMinimumIntensityThreshold(laser_beam);
}

