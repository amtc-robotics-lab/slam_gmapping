/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITparticlefilterNEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Brian Gerkey */

#ifndef SLAM_GMAPPING_SLAM_GMAPPING_H_
#define SLAM_GMAPPING_SLAM_GMAPPING_H_

#include <mutex>
#include <thread>
#include <memory>
#include <iostream>
#include <time.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "message_filters/subscriber.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

class SlamGMapping : public rclcpp::Node
{
public:
    SlamGMapping();
    ~SlamGMapping() override;

    void init();
    void startLiveSlam();
    void publishTransform();

    void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    // bool mapCallback(nav_msgs::GetMap::Request  &req,
    //                  nav_msgs::GetMap::Response &res);
    void publishLoop(double transform_publish_period);

private:
    rclcpp::Node::SharedPtr node_;
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr entropy_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr sst_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr sstm_;
    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_filter_sub_;
    // TF Listener / Broadcaster
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_list_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bcast_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan >> scan_filter_;

    GMapping::RangeSensor* gsp_laser_;
    GMapping::GridSlamProcessor* gsp_;
    // The angles in the laser, going from -x to x (adjustment is made to get the laser between
    // symmetrical bounds as that's what gmapping expects)
    std::vector<double> laser_angles_;
    // The pose, in the original laser frame, of the corresponding centered laser with z facing up
    geometry_msgs::msg::PoseStamped centered_laser_pose_;
    // Depending on the order of the elements in the scan and the orientation of the scan frame,
    // We might need to change the order of the scan
    bool do_reverse_range_;
    unsigned int gsp_laser_beam_count_;
    GMapping::OdometrySensor* gsp_odom_;

    bool got_first_scan_;

    bool got_map_;
    nav_msgs::msg::OccupancyGrid map_;

    tf2::Duration map_update_interval_;
    tf2::Transform map_to_odom_;
    std::mutex map_to_odom_mutex_;
    std::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    std::shared_ptr<std::thread> transform_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t);
    bool initMapper(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    bool addScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    double lstep_;
    double astep_;
    double lsigma_;
    double ogain_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;
    double map_interval_float_;
    double transform_publish_period_;
    double tf_delay_;

    int particles_;
    int lskip_;
    int iterations_;
    int kernelSize_;

    unsigned long int seed_;

};

#endif //SLAM_GMAPPING_SLAM_GMAPPING_H_
