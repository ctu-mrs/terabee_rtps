#include "ros/ros.h"
#include "std_msgs/String.h"
#include "positioning_systems_ros/RtlsTrackerFrame.h"
#include "positioning_systems_ros/RtlsAnchorData.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>

ros::Publisher pub;
ros::Publisher pub_odom;

/* transformation handler */
mrs_lib::Transformer      transformer_;
mrs_lib::TransformStamped tfko;

std::string from_frame_;
std::string to_frame_;

float total_counter_ = 0;
float bad_counter_   = 0;


void rtpsCallback(const positioning_systems_ros::RtlsTrackerFrame& msg) {

  visualization_msgs::Marker      marker;
  visualization_msgs::MarkerArray marker_array;
  int                             anchor_num = 0;

  for (auto anchor : msg.anchors) {
    marker.header.frame_id    = "terabee_rtps";
    marker.header.stamp       = ros::Time();
    marker.ns                 = "anchor"+anchor.id;
    marker.id                 = anchor.id;
    marker.type               = visualization_msgs::Marker::SPHERE;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.position.x    = anchor.position.x;
    marker.pose.position.y    = anchor.position.y;
    marker.pose.position.z    = anchor.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.5;
    marker.scale.y            = 0.5;
    marker.scale.z            = 0.5;
    marker.color.a            = 1;
    marker.color.r            = 0;
    marker.color.g            = 0;
    marker.color.b            = 1;
    marker.lifetime           = ros::Duration(0.1);

    marker_array.markers.push_back(marker);
    anchor_num++;
  }
  marker.header.stamp       = ros::Time();
  marker.ns                 = "tracker";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::CUBE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = msg.position.x;
  marker.pose.position.y    = msg.position.y;
  marker.pose.position.z    = msg.position.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.3;
  marker.scale.y            = 0.3;
  marker.scale.z            = 0.1;
  marker.color.a            = 1;
  marker.color.r            = 0;
  marker.color.g            = 1;
  marker.color.b            = 0;

  marker_array.markers.push_back(marker);

  pub.publish(marker_array);


  nav_msgs::Odometry odom, odom_tf;

  odom.header.frame_id         = "terabee_rtps";
  odom.header.stamp            = ros::Time();
  odom.pose.pose.position.x    = msg.position.x;
  odom.pose.pose.position.y    = msg.position.y;
  odom.pose.pose.position.z    = msg.position.z;
  odom.pose.pose.orientation.w = 1;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;

  {
    auto res = transformer_.getTransform(from_frame_, to_frame_, ros::Time());

    if (!res) {
      ROS_WARN_THROTTLE(1.0, "[%s]: could not find transform from '%s' to '%s' at time '%f'", ros::this_node::getName().c_str(), from_frame_.c_str(),
                        to_frame_.c_str(), ros::Time().toSec());
      return;
    }

    tfko = res.value();
  }

  geometry_msgs::PoseStamped rtps_pose, rtps_pose_tf;
  rtps_pose.header = odom.header;
  rtps_pose.pose   = mrs_lib::getPose(odom);

  {
    auto res = transformer_.transform(tfko, rtps_pose);

    if (!res) {
      ROS_WARN_THROTTLE(1.0, "[%s]: could not transform vins pose to '%s'", ros::this_node::getName().c_str(), to_frame_.c_str());
      return;
    }

    rtps_pose_tf = res.value();
  }

  odom_tf.header.frame_id         = "uav6/rtk_origin";
  odom_tf.header.stamp            = ros::Time();
  odom_tf.pose.pose.position.x    = rtps_pose_tf.pose.position.x;
  odom_tf.pose.pose.position.y    = rtps_pose_tf.pose.position.y;
  odom_tf.pose.pose.position.z    = rtps_pose_tf.pose.position.z;
  odom_tf.pose.pose.orientation.w = 1;
  odom_tf.pose.pose.orientation.x = 0;
  odom_tf.pose.pose.orientation.y = 0;
  odom_tf.pose.pose.orientation.z = 0;


  ROS_INFO_THROTTLE(1.0, "[%s]: Marker published", ros::this_node::getName().c_str());


  /* if (!msg.is_valid_position) { */
  /*   bad_counter_++; */
  /* } else { */
  /*   pub_odom.publish(odom_tf); */
  /* } */

  /* total_counter_++; */

  /* float num = (bad_counter_ / total_counter_) * 100; */
  /* ROS_INFO("[%s]: Total #%f, bad #%f, bad prg #%f", ros::this_node::getName().c_str(), total_counter_, bad_counter_, num); */
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rtps_republisher");

  ros::NodeHandle nh("~");

  pub                 = nh.advertise<visualization_msgs::MarkerArray>("marker_out", 1000);
  pub_odom            = nh.advertise<nav_msgs::Odometry>("odom_out", 1000);
  ros::Subscriber sub = nh.subscribe("/rtls_tracker_node/rtls_tracker_frame/", 10, rtpsCallback);

  /* transformation handler */
  transformer_ = mrs_lib::Transformer("rtps_republisher");
  from_frame_  = "terabee_rtps";
  to_frame_    = "uav6/rtk_origin";

  ros::spin();

  return 0;
}
