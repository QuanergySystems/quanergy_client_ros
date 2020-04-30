/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_ROS_SIMPLE_PUBLISHER_H
#define QUANERGY_CLIENT_ROS_SIMPLE_PUBLISHER_H

#include <mutex>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/** \brief SimplePublisher publishes point clouds of type PointT */
template <typename PointT>
class SimplePublisher
{
public:
  using Cloud = pcl::PointCloud<PointT>;
  using CloudConstPtr = typename Cloud::ConstPtr;

  SimplePublisher(ros::NodeHandle& nh, std::string topic, bool use_ros_time = false)
    : use_ros_time_(use_ros_time)
  {
    topic = nh.resolveName(topic);
    // Don't advertise too many packets. 
    // If you do, you'll create a memory leak, as these things are *huge*.
    publisher_ = nh.advertise<pcl::PointCloud<PointT>>(topic, 10);
  }

  void slot(const CloudConstPtr& cloud)
  {
    if(!ros::ok() || !cloud) return;

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    if (use_ros_time_)
    {
      ros_cloud.header.stamp = ros::Time::now();
    }

    // don't block if publisher isn't available
    std::unique_lock<std::timed_mutex> lock(cloud_publisher_mutex_, std::chrono::milliseconds(10));
    if (lock)
    {
      publisher_.publish(ros_cloud);
    }
  }

  /** \brief run the publisher; blocks until done */
  void run(double frequency = 50.)
  {
    ros::Rate r(frequency);
    while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  }

  void stop()
  {
    ros::shutdown();
  }

private:
  std::timed_mutex cloud_publisher_mutex_;
  bool use_ros_time_;
  ros::Publisher publisher_;
};


#endif
