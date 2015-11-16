/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <iostream>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/failover_client.h>
#include <quanergy/parsers/pointcloud_generator_01.h>
#include <quanergy/parsers/pointcloud_generator_00.h>

#include <quanergy/modules/polar_to_cart_converter.h>
#include <quanergy/modules/distance_filter.h>
#include <quanergy/modules/ring_intensity_filter.h>

// Simple application settings wrapper around boost property tree.
#include "settings.h"

struct M8SensorClient {

  typedef std::shared_ptr<M8SensorClient> Ptr;

  /// FailoverClient adds a failover to old M8 data
  typedef quanergy::client::FailoverClient<quanergy::client::DataPacket01, quanergy::client::DataPacket00> ClientType;

  typedef quanergy::client::DistanceFilter DistanceFilter;
  typedef quanergy::client::RingIntensityFilter RingIntensityFilter;
  typedef quanergy::client::PolarToCartConverter PolarToCartConverter;

  typedef boost::signals2::signal<void (quanergy::PointCloudXYZIRPtr const &)> Signal;

  M8SensorClient(std::string const & host, std::string const & port, std::string const & frame_id = std::string())
    : client_(host, port, frame_id) 
  {
    client_connection_ = client_.connect([this](ClientType::Result const & pc)
                                        { this->distance_filter_.slot(pc); } );

    distance_filter_connection_ = distance_filter_.connect([this](DistanceFilter::Result const & pc)
                                                           {this->ring_intensity_filter_.slot(pc); } );

    ring_intensity_filter_connection_ = ring_intensity_filter_.connect([this](RingIntensityFilter::Result const & pc)
                                                                      {this->polar_to_cart_converter_.slot(pc); } );
  }

  ~M8SensorClient()
  {
    client_connection_.disconnect();
    distance_filter_connection_.disconnect();
    ring_intensity_filter_connection_.disconnect();
  }

  boost::signals2::connection connect(const typename Signal::slot_type & subscriber) 
  {
    return polar_to_cart_converter_.connect(subscriber);
  }

  void run() 
  {
    client_.run();
  }

  void stop() 
  {
    client_.stop();
  }


  void setMaximumDistanceThreshold(float maxThreshold)
  {
    distance_filter_.setMaximumDistanceThreshold(maxThreshold);
  }


  float getMaximumDistanceThreshold() const
  {
    return distance_filter_.getMaximumDistanceThreshold();
  }


  void setMinimumDistanceThreshold(float minThreshold)
  {
    distance_filter_.setMinimumDistanceThreshold(minThreshold);
  }


  float getMinimumDistanceThreshold() const
  {
    return distance_filter_.getMinimumDistanceThreshold();
  }


  void setRingFilterMinimumRangeThreshold(const std::uint16_t laser_beam, const float min_threshold)
  {
    ring_intensity_filter_.setRingFilterMinimumRangeThreshold(laser_beam, min_threshold);
  }

  
  float getRingFilterMinimumRangeThreshold(const std::uint16_t laser_beam) const
  {
    return ring_intensity_filter_.getRingFilterMinimumRangeThreshold(laser_beam);
  }


  void setRingFilterMinimumIntensityThreshold(const uint16_t laser_beam, const uint8_t min_threshold)
  {
    ring_intensity_filter_.setRingFilterMinimumIntensityThreshold(laser_beam, min_threshold);
  }


  uint8_t getRingFilterMinimumIntensityThreshold(const std::uint16_t laser_beam) const
  {
    return ring_intensity_filter_.getRingFilterMinimumIntensityThreshold(laser_beam);
  }


private:

  ClientType client_;
  DistanceFilter distance_filter_;
  RingIntensityFilter ring_intensity_filter_;
  PolarToCartConverter polar_to_cart_converter_;

  boost::signals2::connection client_connection_;
  boost::signals2::connection distance_filter_connection_;
  boost::signals2::connection ring_intensity_filter_connection_;
};



template <typename PointT>
class SimpleM8Publisher
{
public:

  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  SimpleM8Publisher (M8SensorClient::Ptr const & grabber,
                     const std::string& topic,
                     const std::string& frame,
                     bool useRosTime)
    : grabber_ (grabber)
    , topic_ (topic)
    , frame_ (frame)
    , useRosTime_ (useRosTime)
  {}

  void cloud_callback (const CloudConstPtr& cloud)
  {
    if(!ros::ok() || !cloud) return;

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header.frame_id = frame_;
    if (useRosTime_)
      ros_cloud.header.stamp = ros::Time::now();
    else
    {
      ros_cloud.header.stamp.sec = (cloud->header.stamp / 1000000000);
      ros_cloud.header.stamp.nsec = (cloud->header.stamp % 1000000000);
    }

    if (cloud_publisher_mutex_.try_lock ())
    {
      publisher_.publish(ros_cloud);
      cloud_publisher_mutex_.unlock ();
    }
  }

  void run ()
  {
    ros::NodeHandle n;
    std::string topic = n.resolveName(topic_);
    // Don't advertise too many packets. 
    // If you do, you'll create a memory leak, as these things are *huge*.
    publisher_ = n.advertise<pcl::PointCloud<PointT> >(topic, 10);
    boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
      &SimpleM8Publisher::cloud_callback, this, _1);
    cloud_connection_ = grabber_->connect(cloud_cb);

    std::exception_ptr e_ptr;
    std::thread grabber_thread([this, &e_ptr]
                               {
                                 try
                                 {
                                   grabber_->run();
                                 }
                                 catch (std::exception& e)
                                 {
                                   std::cerr << "Terminating after catching exception: " << e.what() << std::endl;
                                   stop();
                                   e_ptr = std::current_exception();
                                 }
                               });
    ros::spin();
    grabber_->stop();
    grabber_thread.join();

    cloud_connection_.disconnect();
    publisher_.shutdown();

    if (e_ptr)
      std::rethrow_exception(e_ptr);
  }

  void stop ()
  {
    ros::shutdown();
  }

private:

  M8SensorClient::Ptr grabber_;
  boost::mutex cloud_publisher_mutex_;
  std::string topic_;
  CloudConstPtr cloud_;
  std::string frame_;
  bool useRosTime_;
  boost::signals2::connection cloud_connection_;
  ros::Publisher publisher_;
};

struct M8ClientNode 
{

  M8ClientNode(int argc, char** argv)
    : settings_file("")
    , min(1.29)         // Distance filter
    , max(200.0)        // Distance filter
    , default_ring_range(0.0f)
    , default_ring_intensity(0)
    , ip("10.0.0.3")
    , port("4141")
    , topic("m8_points")
    , frame("m8")
    , organize(true)
    , useRosTime(false)
  {
    for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
    {
      ring_range[i] = default_ring_range;
      ring_intensity[i] = default_ring_intensity;
    }

    ros::init(argc, argv, "M8Client");

    loadSettings(argc, argv);
    parseArgs(argc, argv);
  }

  void publish()
  {
    M8SensorClient::Ptr grabber = M8SensorClient::Ptr(new M8SensorClient(ip, port));

    grabber->setMinimumDistanceThreshold(min);
    grabber->setMaximumDistanceThreshold(max);

    for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
    {
      grabber->setRingFilterMinimumRangeThreshold(i, ring_range[i]);
      grabber->setRingFilterMinimumIntensityThreshold(i, ring_intensity[i]);
    }
  
    SimpleM8Publisher<quanergy::PointXYZIR> p(grabber, topic, frame, useRosTime);
    try
    {
      p.run();
    }
    catch (quanergy::client::SocketBindError& e)
    {
      std::cout << "Unable to bind to socket; shutting down" << std::endl;
    }
    catch (quanergy::client::SocketReadError& e)
    {
      std::cout << "Socket connection dropped; shutting down" << std::endl;
    }
  }


private:

  void loadSettings(int argc, char ** argv)
  {
    // Is there a settings file specified?

    pcl::console::parse_argument (argc, argv, "-settings", settings_file);

    if (!settings_file.empty())
    {
      quanergy::Settings settings;

      settings.loadXML(settings_file);

      min = settings.get("DistanceFilter.min", min);
      max = settings.get("DistanceFilter.max", max);

      for (int i = 0; i < quanergy::client::M8_NUM_LASERS; i++)
      {
        const std::string num = boost::lexical_cast<std::string>(i);

        std::string range_param = std::string("RingFilter.Range").append(num);
        ring_range[i] = settings.get(range_param, ring_range[i]);

        std::string intensity_param = std::string("RingFilter.Intensity").append(num);
        ring_intensity[i] = settings.get(intensity_param, ring_intensity[i]);
      }

      ip = settings.get("M8ClientRos.ip", ip);
      port = settings.get("M8ClientRos.port", port);

      topic = settings.get("M8ClientRos.topic", topic);
      frame = settings.get("M8ClientRos.frame", frame);

      organize = settings.get("M8ClientRos.organize", organize);
      useRosTime = settings.get("M8ClientRos.useRosTime", useRosTime);
    }
  }


  void parseArgs(int argc, char ** argv)
  {
    pcl::console::parse_argument (argc, argv, "-min", min);
    pcl::console::parse_argument (argc, argv, "-max", max);

    pcl::console::parse_argument (argc, argv, "-ip", ip);
    pcl::console::parse_argument (argc, argv, "-port", port);

    pcl::console::parse_argument (argc, argv, "-topic", topic);
    pcl::console::parse_argument (argc, argv, "-frame", frame);

    pcl::console::parse_argument (argc, argv, "-organize", organize);
    pcl::console::parse_argument (argc, argv, "-useRosTime", useRosTime);
  }


private:


  // Defaults, overridded by settings file, which is in turn
  // overridden by command-line options.

  std::string settings_file;

  // Distance filter

  float min;
  float max;

  // Ring filter
  // Only set by config file

  float default_ring_range;
  std::uint16_t default_ring_intensity;

  float ring_range[quanergy::client::M8_NUM_LASERS];
  std::uint16_t ring_intensity[quanergy::client::M8_NUM_LASERS];
  
  // M8 Client

  std::string ip;
  std::string port;

  std::string topic;
  std::string frame;

  bool organize;
  bool useRosTime;
};


int main (int argc, char ** argv)
{

  M8ClientNode node (argc, argv);

  node.publish();

  return (0);
}
