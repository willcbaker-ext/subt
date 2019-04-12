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

#include <ros/ros.h>

#include <functional>
#include <mutex>

#include <ignition/common/Console.hh>

#include "subt_ign/CommsBrokerPlugin.hh"

using namespace gazebo;
using namespace subt;
using namespace subt::communication_model;
using namespace subt::rf_interface;
using namespace subt::rf_interface::range_model;
using namespace subt::communication_broker;
using namespace subt::rf_interface::visibility_model;

GZ_REGISTER_WORLD_PLUGIN(CommsBrokerPlugin)

/////////////////////////////////////////////////
void CommsBrokerPlugin::Load(const tinyxml2::XMLElement *_elem)
{
  // Build RF Function
  struct range_model::rf_configuration range_config;
  struct visibility_model::rf_configuration visibility_config;
  // Build radio configuration (which includes RF pathloss function)
  struct radio_configuration radio;

  const tinyxml2::XMLElement *commsModelElem =
    _elem->FirstChildElement("comms_model");

  if (commsModelElem)
  {
    const tinyxml2::XMLElement *rfConfigElem = 
          commsModelElem->FirstChildElement("visibility_config");
    if (rfConfigElem)
    {
      const tinyxml2::XMLElement *elem;
      elem = rfConfigElem->FirstChildElement(
        "visibility_cost_to_fading_exponent");
      if (elem)
      {
        double value;
        auto res = elem->QueryDoubleText(&value);
        if (res = EXML_SUCCESS)
          visibility_config.visibility_cost_to_fading_exponent = value;
      }
      // if (rfConfigElem->HasElement("visibility_cost_to_fading_exponent"))
      // {
      //   visibility_config.visibility_cost_to_fading_exponent =
      //       rfConfigElem->Get<double>("visibility_cost_to_fading_exponent");
      // }
      // if (rfConfigElem->HasElement("comms_cost_max"))
      // {
      //   visibility_config.comms_cost_max =
      //       rfConfigElem->Get<double>("comms_cost_max");
      // }

      // ROS_INFO_STREAM("Loading visibility_config from SDF: \n" <<
      //                 visibility_config);
    }

    // if (commsModelElem->HasElement("range_config"))
    // {
    //   auto const &rfConfigElem = commsModelElem->GetElement("range_config");

    //   if (rfConfigElem->HasElement("max_range"))
    //   {
    //     range_config.max_range = rfConfigElem->Get<double>("max_range");
    //   }
    //   if (rfConfigElem->HasElement("fading_exponent"))
    //   {
    //     range_config.fading_exponent =
    //         rfConfigElem->Get<double>("fading_exponent");
    //   }
    //   if (rfConfigElem->HasElement("L0"))
    //   {
    //     range_config.L0 = rfConfigElem->Get<double>("L0");
    //   }
    //   if (rfConfigElem->HasElement("sigma"))
    //   {
    //     range_config.sigma = rfConfigElem->Get<double>("sigma");
    //   }

    //   ROS_INFO_STREAM("Loading range_config from SDF: \n" << range_config);
    // }

    // if (commsModelElem->HasElement("radio_config"))
    // {
    //   auto const &radioConfigElem = commsModelElem->GetElement("radio_config");

    //   if (radioConfigElem->HasElement("capacity"))
    //   {
    //     radio.capacity = radioConfigElem->Get<double>("capacity");
    //   }
    //   if (radioConfigElem->HasElement("tx_power"))
    //   {
    //     radio.default_tx_power = radioConfigElem->Get<double>("tx_power");
    //   }
    //   if (radioConfigElem->HasElement("modulation"))
    //   {
    //     radio.modulation = radioConfigElem->Get<std::string>("modulation");
    //   }
    //   if (radioConfigElem->HasElement("noise_floor"))
    //   {
    //     radio.noise_floor = radioConfigElem->Get<double>("noise_floor");
    //   }

    //   ROS_INFO_STREAM("Loading radio_config from SDF: \n" << radio);
    // }
  }

  const tinyxml2::XMLElement *worldNameElem = 
    _elem->FirstChildElement("world_name");
  std::string worldName = "default";
  if (worldNameElem)
  {
    worldName = worldNameElem->GetText();
  }
  else
  {
    ignerr << "Missing <world_name>, the GameLogicPlugin will assume a "
      << " world name of 'default'. This could lead to incorrect scoring\n";
  }

  // TODO: Maybe only try to instantiate if visibility type is selected
  this->visibilityModel = std::make_unique<VisibilityModel>(visibility_config,
                                                            range_config);

  // Build RF propagation function options
  std::map<std::string, pathloss_function> pathloss_functions;

  pathloss_functions["log_normal_range"] =
      std::bind(&log_normal_received_power,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3,
                range_config);

  pathloss_functions["visibility_range"] =
      std::bind(&VisibilityModel::compute_received_power,
                this->visibilityModel.get(),
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3);

  // Default comms model type is log_normal_range (will always work)
  std::string comms_model_type = "log_normal_range";
  if (_sdf->HasElement("comms_model")) {
    auto const &commsModelElem = _sdf->GetElement("comms_model");

    if (commsModelElem->HasElement("comms_model_type")) {
      std::string comms_model_type_tmp =
          commsModelElem->Get<std::string>("comms_model_type");

      // Only allow the specified comms_model_type if it is in our map
      // of available functions
      if (pathloss_functions.find(comms_model_type_tmp) ==
         pathloss_functions.end()) {
        ROS_WARN("comms_model_type: %s is not available, falling back to %s",
                 comms_model_type_tmp.c_str(),
                 comms_model_type.c_str());
      }
      else
      {
        comms_model_type = comms_model_type_tmp;
        ROS_INFO("Using comms_model_type: %s", comms_model_type.c_str());
      }
    }
    else
    {
      ROS_WARN("comms_model_type not specified, using: %s",
               comms_model_type.c_str());
    }
  }

  radio.pathloss_f = pathloss_functions[comms_model_type];
  broker.SetDefaultRadioConfiguration(radio);

  // Set communication function (i.e., the attempt_send function) to
  // use for the broker
  broker.SetCommunicationFunction(&subt::communication_model::attempt_send);

  // Build function to get pose from gazebo
  auto update_pose_func = [_world](const std::string& name) {
    auto const &model = _world->ModelByName(name);

    if (!model)
      return std::make_tuple(false,
                             ignition::math::Pose3<double>(),
                             ros::Time());

    return std::make_tuple(true,
                           model->WorldPose(),
                           ros::Time(_world->SimTime().Double()));
  };
  broker.SetPoseUpdateFunction(update_pose_func);

  broker.Start();

  // Subscribe to pose messages. We will pull out model and artifact
  // information from the published message.
  this->node.Subscribe("/world/" + worldName + "/pose/info",
      &CommsBrokerPlugin::OnClock, this);

  ignmsg << "Starting SubT comms broker" << std::endl;
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnClock(const ignition::msgs::Clock &_msg)
{
  ignition::common::Time now(_msg.sim().sec(), _msg.sim().nsec());
  double dt = (now.Double() - this->lastROSParameterCheckTime;

  // It's time to query the ROS parameter server. We do it every second.
  if (dt >= 1.0 )
  {
    bool value = false;
    ros::param::get("/subt/comms/simple_mode", value);

    if (value)
    {
      igndbg << "Enabling simple mode comms" << std::endl;

      auto f = [](const radio_configuration&,
                  const rf_interface::radio_state&,
                  const rf_interface::radio_state&,
                  const uint64_t&
                  ) { return true; };

      broker.SetCommunicationFunction(f);
    }
    else
    {
      igndbg << "Disabling simple mode comms" << std::endl;
      broker.SetCommunicationFunction(&subt::communication_model::attempt_send);
    }

    this->lastROSParameterCheckTime = now.Double();
  }

  // Send a message to each team member with its updated neighbors list.
  this->broker.NotifyNeighbors();

  // Dispatch all the incoming messages, deciding whether the destination gets
  // the message according to the communication model.
  this->broker.DispatchMessages();
}
