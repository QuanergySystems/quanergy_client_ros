/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_ROS_SIMPLE_PUBLISHER_H
#define QUANERGY_CLIENT_ROS_SIMPLE_PUBLISHER_H


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


#include <quanergy_client_ros/sensor_client.h>


template <typename PointT>
class SimplePublisher
{
public:

  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  SimplePublisher(SensorClient::Ptr const & grabber,
                    const std::string& topic,
                    const std::string& frame,
                    bool useRosTime)
    : grabber_ (grabber)
    , topic_ (topic)
    , frame_ (frame)
    , useRosTime_ (useRosTime)
  {}


  void cloud_callback(const CloudConstPtr& cloud)

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


  void run()
  {
    ros::NodeHandle n;
    std::string topic = n.resolveName(topic_);
    // Don't advertise too many packets. 
    // If you do, you'll create a memory leak, as these things are *huge*.
    publisher_ = n.advertise<pcl::PointCloud<PointT> >(topic, 10);
    boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
      &SimplePublisher::cloud_callback, this, _1);
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


  void stop()
  {
    ros::shutdown();
  }


private:

  SensorClient::Ptr grabber_;
  boost::mutex cloud_publisher_mutex_;
  std::string topic_;
  CloudConstPtr cloud_;
  std::string frame_;
  bool useRosTime_;
  boost::signals2::connection cloud_connection_;
  ros::Publisher publisher_;
};


#endif
