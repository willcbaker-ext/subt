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
#include <ignition/common/Console.hh>
#include "subt_ign/FlashlightPlugin.hh"

using namespace ignition;
using namespace subt;

//////////////////////////////////////////////////
void FlashlightPlugin::Load(const tinyxml2::XMLElement *_elem)
{
  // === must call this ===
  // LedPlugin::Load(_parent, _sdf);

  // Service name is renamed if an alternative one is given in SDF.
  std::string serviceName = "light_control";
  const tinyxml2::XMLElement *elem = _elem->FirstChildElement("service_name");
  if (elem)
    serviceName = elem->GetText();

  // if (serviceName[0] != '/')
  // {
  //   std::string prefix = _parent->GetScopedName();
  //   int pos = prefix.find("::");
  //   while (pos > 0)
  //   {
  //     prefix.replace(pos, 2, "/");
  //     pos = prefix.find("::");
  //   }
  //   serviceName = "/" + prefix + "/" + serviceName;
  // }
  ignmsg << "Flashlight service name: " << serviceName << std::endl;

  this->node.Advertise(serviceName, &FlashlightPlugin::Control, this);

  // Make sure the ROS node for Gazebo has already been initialized
  // if (!ros::isInitialized())
  // {
  //   ROS_FATAL_STREAM(
  //     "A ROS node for Gazebo has not been initialized, unable to load plugin. "
  //     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so'"
  //     << " in the gazebo_ros package)");
  //   return;
  // }

  // // ROS service to receive a command to control the light
  // ros::NodeHandle n;
  // this->rosService
  //   = n.advertiseService(serviceName, &RosFlashLightPlugin::Control, this);
}

//////////////////////////////////////////////////
bool FlashlightPlugin::Control(
    const ignition::msgs::Boolean & /*_req*/, ignition::msgs::Boolean & /*_res*/)
{
  std::cout << "On flashlight\n";
  return true;
}

//////////////////////////////////////////////////
/*bool FlashlightPlugin::ControlRos(std_srvs::SetBool::Request &_req,
                                  std_srvs::SetBool::Response &_res)
{
  if (_req.data)
    _res.success = this->TurnOnAll();
  else
    _res.success = this->TurnOffAll();

  return _res.success;
}*/
