/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <subt_msgs/Bind.h>
#include <subt_msgs/DatagramRos.h>
#include <subt_msgs/PoseFromArtifact.h>
#include <subt_msgs/Register.h>
#include <subt_msgs/Unregister.h>
#include <subt_communication_broker/protobuf/datagram.pb.h>
#include <subt_communication_broker/common_types.h>

#include <ignition/transport/Node.hh>

class SubtRosRelay
{
  /// \brief Constructor
  public: SubtRosRelay();

  /// \brief Destructor
  public: ~SubtRosRelay();

  /// \brief Ign callback for score topic and the data is republished to ROS
  /// \param[in] _msg Score msg
  public: void OnScore(const ignition::msgs::Float &_msg);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing a flag telling if the game
  /// is to start.
  /// \param[out] _res The response message.
  public: bool OnStartCall(std_srvs::SetBool::Request &_req,
                           std_srvs::SetBool::Response &_res);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing a flag telling if the game is to
  /// be finished.
  /// \param[out] _res The response message.
  public: bool OnFinishCall(std_srvs::SetBool::Request &_req,
                            std_srvs::SetBool::Response &_res);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing the robot name.
  /// \param[out] _res The response message.
  public: bool OnPoseFromArtifact(
               subt_msgs::PoseFromArtifact::Request &_req,
               subt_msgs::PoseFromArtifact::Response &_res);

  /// \brief ROS service callback triggered when a comms client binds to an
  /// address.
  /// \param[in] _req The requested address and endpoint.
  /// \param[out] _res True if the bind was successful, false otherwise.
  public: bool OnBind(subt_msgs::Bind::Request &_req,
                      subt_msgs::Bind::Response &_res);

  /// \brief ROS service callback triggerered when a comms client sends a
  /// message to an address.
  /// \param[in] _req The requested message to send.
  /// \param[out] _res The response.
  public: bool OnSendTo(subt_msgs::DatagramRos::Request &_req,
                        subt_msgs::DatagramRos::Response &_res);

  /// \brief ROS service callback triggerered when a comms client is
  /// registered.
  /// \param[in] _req The request.
  /// \param[out] _res The response.
  public: bool OnRegister(subt_msgs::Register::Request &_req,
                          subt_msgs::Register::Response &_res);

  /// \brief ROS service callback triggerered when a comms client is
  /// unregistered.
  /// \param[in] _req The request.
  /// \param[out] _res The response.
  public: bool OnUnregister(subt_msgs::Unregister::Request &_req,
                            subt_msgs::Unregister::Response &_res);

  /// \brief Ignition service callback triggerered when a message is received.
  /// Inside the callback, the message is forwarded via a ROS service call.
  /// \param[in] _msg The message.
  public: bool OnMessage(const subt::msgs::Datagram &_msg,
                         ignition::msgs::Boolean &_res);

  /// \brief Ignition Transport node.
  public: ignition::transport::Node node;

  /// \brief The ROS node handler used for communications.
  public: std::unique_ptr<ros::NodeHandle> rosnode;

  /// \brief ROS service to receive a call to finish the game.
  public: ros::ServiceServer finishService;

  /// \brief ROS service to receive a call to start the game.
  public: ros::ServiceServer startService;

  /// \brief ROS service to receive a bind request.
  public: ros::ServiceServer commsModelBindService;

  /// \brief ROS service to receive a send to request.
  public: ros::ServiceServer commsModelSendToService;

  /// \brief ROS service to receive a register request.
  public: ros::ServiceServer commsModelRegisterService;

  /// \brief ROS service to receive a unregister request.
  public: ros::ServiceServer commsModelUnregisterService;


  /// \brief ROS publisher to publish score data
  public: ros::Publisher rosScorePub;

  /// \brief ROS service server to receive the location of a robot relative to
  /// the origin artifact.
  public: ros::ServiceServer poseFromArtifactService;

  /// \brief A ROS asynchronous spinner.
  public: std::unique_ptr<ros::AsyncSpinner> spinner;
};

//////////////////////////////////////////////////
SubtRosRelay::SubtRosRelay()
{
  // Initialize the ROS node.
  this->rosnode.reset(new ros::NodeHandle("subt"));

  // ROS service to receive a command to finish the game.
  ros::NodeHandle n;
  this->finishService = n.advertiseService(
      "/subt/finish", &SubtRosRelay::OnFinishCall, this);

  this->startService = n.advertiseService(
      "/subt/start", &SubtRosRelay::OnStartCall, this);

  // ROS service to request the robot pose relative to the origin artifact.
  // Note that this service is only available for robots in the staging area.
  this->poseFromArtifactService = n.advertiseService(
      "/subt/pose_from_artifact_origin",
      &SubtRosRelay::OnPoseFromArtifact, this);

  this->commsModelBindService = n.advertiseService(
      subt::communication_broker::kEndPointRegistrationSrv,
      &SubtRosRelay::OnBind, this);

  this->commsModelSendToService = n.advertiseService(
      subt::communication_broker::kBrokerSrv,
      &SubtRosRelay::OnSendTo, this);

  this->commsModelRegisterService = n.advertiseService(
      subt::communication_broker::kAddrRegistrationSrv,
      &SubtRosRelay::OnRegister, this);

  this->commsModelUnregisterService = n.advertiseService(
      subt::communication_broker::kAddrUnregistrationSrv,
      &SubtRosRelay::OnUnregister, this);

  this->node.Subscribe("/subt/score", &SubtRosRelay::OnScore, this);

  this->rosScorePub =
    this->rosnode->advertise<std_msgs::Int32>("score", 1000);
}

//////////////////////////////////////////////////
SubtRosRelay::~SubtRosRelay()
{
}

/////////////////////////////////////////////////
void SubtRosRelay::OnScore(const ignition::msgs::Float &_msg)
{
  std_msgs::Int32 rosMsg;
  rosMsg.data = _msg.data();
  this->rosScorePub.publish(rosMsg);
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnFinishCall(std_srvs::SetBool::Request &_req,
  std_srvs::SetBool::Response &_res)
{
  ignition::msgs::Boolean req;
  ignition::msgs::Boolean rep;
  unsigned int timeout = 5000;
  bool result;

  req.set_data(_req.data);

  // Pass the request onto ignition transport
  this->node.Request("/subt/finish", req, timeout, rep, result);
  _res.success = rep.data();

  return result;
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnStartCall(std_srvs::SetBool::Request &_req,
  std_srvs::SetBool::Response &_res)
{
  ignition::msgs::Boolean req;
  ignition::msgs::Boolean rep;
  unsigned int timeout = 5000;
  bool result;

  req.set_data(_req.data);

  // Pass the request onto ignition transport
  this->node.Request("/subt/start", req, timeout, rep, result);
  _res.success = rep.data();

  return result;
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnPoseFromArtifact(
  subt_msgs::PoseFromArtifact::Request &_req,
  subt_msgs::PoseFromArtifact::Response &_res)
{
  ignition::msgs::StringMsg req;
  ignition::msgs::Pose rep;
  unsigned int timeout = 5000;
  bool result;

  req.set_data(_req.robot_name.data);

  // Pass the request onto ignition transport
  _res.success = this->node.Request("/subt/pose_from_artifact_origin",
      req, timeout, rep, result);

  // Construct the ROS response
  _res.pose.pose.position.x = rep.position().x();
  _res.pose.pose.position.y = rep.position().y();
  _res.pose.pose.position.z = rep.position().z();
  _res.pose.pose.orientation.x = rep.orientation().x();
  _res.pose.pose.orientation.y = rep.orientation().y();
  _res.pose.pose.orientation.z = rep.orientation().z();
  _res.pose.pose.orientation.w = rep.orientation().w();

  // Header.
  _res.pose.header.stamp = ros::Time(
    rep.header().stamp().sec(), rep.header().stamp().nsec());
  _res.pose.header.frame_id = rep.header().data(0).value(0);

  return result;
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnBind(subt_msgs::Bind::Request &_req,
                          subt_msgs::Bind::Response &_res)
{
  ignition::msgs::StringMsg_V req;
  req.add_data(_req.address);
  req.add_data(_req.endpoint);

  const unsigned int timeout = 3000u;
  ignition::msgs::Boolean rep;
  bool result;
  bool executed = this->node.Request(
      subt::communication_broker::kEndPointRegistrationSrv,
      req, timeout, rep, result);

  _res.success = result;

  if (executed && result)
  {
    if (!this->node.Advertise(_req.address, &SubtRosRelay::OnMessage, this))
    {
      std::cerr << "Bind Error: could not advertise "
                << _req.address << std::endl;
      return false;
    }
  }

  return executed;
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnSendTo(subt_msgs::DatagramRos::Request &_req,
                            subt_msgs::DatagramRos::Response &_res)
{
  subt::msgs::Datagram msg;
  msg.set_src_address(_req.src_address);
  msg.set_dst_address(_req.dst_address);
  msg.set_dst_port(_req.dst_port);
  msg.set_rssi(_req.rssi);
  msg.set_data(_req.data);

  return this->node.Request(subt::communication_broker::kBrokerSrv, msg);
}

//////////////////////////////////////////////////
bool SubtRosRelay::OnRegister(subt_msgs::Register::Request &_req,
                              subt_msgs::Register::Response &_res)
{
  ignition::msgs::StringMsg req;
  req.set_data(_req.local_address);

  ignition::msgs::Boolean rep;
  bool result;
  const unsigned int timeout = 3000u;

  bool executed = this->node.Request(
    subt::communication_broker::kAddrRegistrationSrv,
    req, timeout, rep, result);

  _res.success = result;
  return executed;
}

//////////////////////////////////////////////////
bool SubtRosRelay::OnUnregister(subt_msgs::Unregister::Request &_req,
                                subt_msgs::Unregister::Response &_res)
{
  ignition::msgs::StringMsg req;
  req.set_data(_req.local_address);

  ignition::msgs::Boolean rep;
  bool result;
  const unsigned int timeout = 3000u;

  bool executed = this->node.Request(
    subt::communication_broker::kAddrUnregistrationSrv,
    req, timeout, rep, result);

  _res.success = result;

  return executed;
}

//////////////////////////////////////////////////
bool SubtRosRelay::OnMessage(const subt::msgs::Datagram &_req,
    ignition::msgs::Boolean &_res)
{
  subt_msgs::DatagramRos::Request req;
  subt_msgs::DatagramRos::Response res;
  req.src_address = _req.src_address();
  req.dst_address = _req.dst_address();
  req.dst_port = _req.dst_port();
  req.data = _req.data();
  req.rssi = _req.rssi();

  ros::service::call(_req.dst_address(), req, res);
  _res.set_data(true);

  return true;
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "subt_ros_relay");

  SubtRosRelay relay;

  ros::spin();
  return 0;
}

