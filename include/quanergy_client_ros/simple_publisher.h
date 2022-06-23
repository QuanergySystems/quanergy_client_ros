/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_ROS_SIMPLE_PUBLISHER_H
#define QUANERGY_CLIENT_ROS_SIMPLE_PUBLISHER_H

#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#else
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#endif

/** \brief SimplePublisher publishes point clouds of type PointT */
template <typename PointT>
class SimplePublisher
{
public:
  using Cloud = pcl::PointCloud<PointT>;
  using CloudConstPtr = typename Cloud::ConstPtr;

#ifdef ROS2_FOUND
  SimplePublisher(rclcpp::Node::SharedPtr& node, std::string topic, bool use_ros_time = false)
    : node_(node)
    , use_ros_time_(use_ros_time)
  {
    // Don't advertise too many packets. 
    // If you do, you'll create a memory leak, as these things are *huge*.
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);
  }
#else
  SimplePublisher(ros::NodeHandle& nh, std::string topic, bool use_ros_time = false)
      : use_ros_time_(use_ros_time)
  {
    topic = nh.resolveName(topic);
    // Don't advertise too many packets.
    // If you do, you'll create a memory leak, as these things are *huge*.
    publisher_.reset(new ros::Publisher);
    *publisher_ = nh.advertise<pcl::PointCloud<PointT>>(topic, 10);
  }
#endif

  void slot(const CloudConstPtr& cloud)
  {
#ifdef ROS2_FOUND
    if (!rclcpp::ok() || !cloud)
      return;

    sensor_msgs::msg::PointCloud2 ros_cloud;
#else
    if (!ros::ok() || !cloud)
      return;

    sensor_msgs::PointCloud2 ros_cloud;
#endif

    pcl::toROSMsg(*cloud, ros_cloud);
    if (use_ros_time_)
    {
#ifdef ROS2_FOUND
      ros_cloud.header.stamp = node_->now();
#else
      ros_cloud.header.stamp = ros::Time::now();
#endif
    }

    // don't block if publisher isn't available
    std::unique_lock<std::timed_mutex> lock(cloud_publisher_mutex_, std::chrono::milliseconds(10));
    if (lock)
    {
      publisher_->publish(ros_cloud);
    }
  }

  /** \brief run the publisher; blocks until done */
  void run(double frequency = 50.)
  {
#ifdef ROS2_FOUND
    rclcpp::Rate r(frequency);
    while (rclcpp::ok())
    {
      rclcpp::spin_some(node_);
#else
    ros::Rate r(frequency);
    while (ros::ok())
    {
      ros::spinOnce();
#endif

      r.sleep();
    }
  }

  void stop()
  {
#ifdef ROS2_FOUND
    rclcpp::shutdown();
#else
    ros::shutdown();
#endif
  }

private:
  std::timed_mutex cloud_publisher_mutex_;
  bool use_ros_time_;

#ifdef ROS2_FOUND
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
#else
  std::shared_ptr<ros::Publisher> publisher_;
#endif
};


#endif
