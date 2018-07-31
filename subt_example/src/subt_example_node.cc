/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <chrono>
#include <iostream>
#include <thread>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <subt_gazebo/CommsClient.hh>

using namespace std::chrono_literals;

class Controller
{
    /// \brief Constructor.
    public: Controller(const std::string &_name, const std::string &_address);

    /// \brief Callback function for selection command.
    public: void teleopSelectCallback(const std_msgs::Bool::ConstPtr& _select);

    /// \brief Callback function for velocity command.
    public: void teleopVelCallback(const geometry_msgs::Twist::ConstPtr& _vel);

    /// \brief Callback function for light command.
    public: void teleopLightCallback(const std_msgs::Bool::ConstPtr& _switch);

    /// \brief Callback function for communication command.
    public: void teleopCommCallback(const std_msgs::String::ConstPtr& _dest);

    /// \brief Callback function for message from other comm clients.
    public: void commClientCallback(const std::string &_srcAddress,
                                    const std::string &_dstAddress,
                                    const uint32_t _dstPort,
                                    const std::string &_data);

    /// \brief Name of the robot.
    private: std::string name;

    /// \brief ROS node handler.
    private: ros::NodeHandle n;

    /// \brief subscriber for selection command from teleop.
    private: ros::Subscriber teleopSelectSub;

    /// \brief subscriber for velocity command from teleop.
    private: ros::Subscriber teleopVelSub;

    /// \brief subscriber for light command from teleop.
    private: ros::Subscriber teleopLightSub;

    /// \brief subscriber for communication command from teleop.
    private: ros::Subscriber teleopCommSub;

    /// \brief publisher to send cmd_vel
    private: ros::Publisher velPub;

    /// \brief List of service clients to control flashlight(s).
    private: std::vector<ros::ServiceClient> flashlightSrvList;

    /// \brief List of service clients to control selection LED(s).
    private: std::vector<ros::ServiceClient> selectLedSrvList;

    /// \brief List of service clients to control communciation LED(s).
    private: std::vector<ros::ServiceClient> commLedSrvList;

    /// \brief Communication client.
    private: subt::CommsClient client;
};

/////////////////////////////////////////////////
Controller::Controller(const std::string &_name, const std::string &_address):
  client(_address)
{
  this->name = _name;
  this->teleopSelectSub
    = this->n.subscribe<std_msgs::Bool>(
      _name + "/select", 1, &Controller::teleopSelectCallback, this);
  this->teleopVelSub
    = this->n.subscribe<geometry_msgs::Twist>(
      _name + "/cmd_vel_relay", 1, &Controller::teleopVelCallback, this);
  this->teleopLightSub
    = this->n.subscribe<std_msgs::Bool>(
      _name + "/light", 1, &Controller::teleopLightCallback, this);
  this->teleopCommSub
    = this->n.subscribe<std_msgs::String>(
      _name + "/comm", 1, &Controller::teleopCommCallback, this);

  this->client.Bind(&Controller::commClientCallback, this);

  this->velPub
    = this->n.advertise<geometry_msgs::Twist>(_name + "/cmd_vel", 1);

  std::vector<std::string> flashlightSrvSuffixList;
  this->n.getParam(
    "flashlight_service_suffixes", flashlightSrvSuffixList);
  std::vector<std::string> selectLedSrvSuffixList;
  this->n.getParam(
    "select_led_service_suffixes", selectLedSrvSuffixList);
  std::vector<std::string> commLedSrvSuffixList;
  this->n.getParam(
    "comm_led_service_suffixes", commLedSrvSuffixList);

  // Create service clients to control flashlights, and associate them
  // to the corresponding service names.
  for (auto suffix: flashlightSrvSuffixList)
  {
    // Note: a service name is formatted like, "/<robot name><suffix>"
    std::string serviceName = "/" + this->name + suffix;
    this->flashlightSrvList.push_back(
      this->n.serviceClient<std_srvs::SetBool>(serviceName));
  }

  // Create service clients to control flashlights, and associate them
  // to the corresponding service names.
  for (auto suffix: selectLedSrvSuffixList)
  {
    // Note: a service name is formatted like, "/<robot name><suffix>"
    std::string serviceName = "/" + this->name + suffix;
    this->selectLedSrvList.push_back(
      this->n.serviceClient<std_srvs::SetBool>(serviceName));
  }
  // Create service clients to control flashlights, and associate them
  // to the corresponding service names.
  for (auto suffix: commLedSrvSuffixList)
  {
    // Note: a service name is formatted like, "/<robot name><suffix>"
    std::string serviceName = "/" + this->name + suffix;
    this->commLedSrvList.push_back(
      this->n.serviceClient<std_srvs::SetBool>(serviceName));
  }

}

/////////////////////////////////////////////////
void Controller::teleopSelectCallback(const std_msgs::Bool::ConstPtr& _select)
{
  ROS_INFO("teleopSelectCallback");

  std_srvs::SetBool srv;
  srv.request.data = _select->data;
  for (auto service: this->selectLedSrvList)
  {
    service.call(srv);
  }
}

/////////////////////////////////////////////////
void Controller::teleopVelCallback(const geometry_msgs::Twist::ConstPtr& _vel)
{
  ROS_INFO("teleopVelCallback");

  geometry_msgs::Twist vel = *_vel;
  this->velPub.publish(vel);
}

/////////////////////////////////////////////////
void Controller::teleopLightCallback(const std_msgs::Bool::ConstPtr& _switch)
{
  ROS_INFO_STREAM("teleopLightCallback" << this->flashlightSrvList.size());

  std_srvs::SetBool srv;
  srv.request.data = _switch->data;
  for (auto service: this->flashlightSrvList)
  {
    service.call(srv);
  }
}

/////////////////////////////////////////////////
void Controller::teleopCommCallback(const std_msgs::String::ConstPtr& _dest)
{
  ROS_INFO("teleopCommCallback");
  this->client.SendTo("_data_", _dest->data);
}

/////////////////////////////////////////////////
void Controller::commClientCallback(const std::string &_srcAddress,
                                const std::string &_dstAddress,
                                const uint32_t _dstPort,
                                const std::string &_data)
{
  ROS_INFO("commClientCallback");

  std_srvs::SetBool srv;
  srv.request.data = true;
  for (auto service: this->commLedSrvList)
  {
    service.call(srv);
  }

  ros::Duration(3.0).sleep();

  srv.request.data = false;
  for (auto service: this->commLedSrvList)
  {
    service.call(srv);
  }
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_ERROR_STREAM(
      "Needs an argument for the competitor's name.");
    return -1;
  }

  ros::init(argc, argv, argv[1]);

  ros::NodeHandle n;
  std::map<std::string, std::string> robotAddressMap;
  n.getParam("robot_address_map", robotAddressMap);

  // Instantiate a communication handler for sending and receiving data.
  Controller controller(argv[1], robotAddressMap[argv[1]]);

  ROS_INFO("Starting competitor\n");

  ros::spin();
}
