/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
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

template <typename PointT>
class SimplePublisher
{
public:

  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  SimplePublisher(const std::string& topic,
                  bool useRosTime = false)
    : topic_(topic)
    , useRosTime_(useRosTime)
  {
  }

  void slot(const CloudConstPtr& cloud)
  {
    if(!ros::ok() || !cloud) return;

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    if (useRosTime_)
      ros_cloud.header.stamp = ros::Time::now();

    if (cloud_publisher_mutex_.try_lock())
    {
      publisher_.publish(ros_cloud);
      cloud_publisher_mutex_.unlock ();
    }
  }

  /** \brief run the application */
  void run(double frequency = 50.)
  {
    ros::NodeHandle n;
    std::string topic = n.resolveName(topic_);
    // Don't advertise too many packets. 
    // If you do, you'll create a memory leak, as these things are *huge*.
    publisher_ = n.advertise<pcl::PointCloud<PointT> >(topic, 10);

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
  std::mutex cloud_publisher_mutex_;
  std::string topic_;
  bool useRosTime_;
  ros::Publisher publisher_;
};


#endif
