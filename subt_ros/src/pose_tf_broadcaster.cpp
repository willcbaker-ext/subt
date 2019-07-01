// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

void sendTransform(const geometry_msgs::TransformStamped& msg)
{
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(msg);
}

// Odom broadcaster
void odomCallback(const nav_msgs::Odometry& msg)
{
  geometry_msgs::TransformStamped geomMsg;
  geomMsg.header.stamp = msg.header.stamp;
  geomMsg.header.frame_id = msg.header.frame_id;
  geomMsg.child_frame_id = msg.child_frame_id;
  geomMsg.transform.translation.x = msg.pose.pose.position.x;
  geomMsg.transform.translation.y = msg.pose.pose.position.y;
  geomMsg.transform.translation.z = msg.pose.pose.position.z;
  geomMsg.transform.rotation = msg.pose.pose.orientation;

  sendTransform(geomMsg);
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "pose_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("pose", 10, &sendTransform);
  ros::Subscriber odomSub = node.subscribe("odom", 10, &odomCallback);

  ros::spin();
  return 0;
}
