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

#include <ignition/common/Console.hh>

#include "subt_gazebo/BaseStationPlugin.hh"
#include "subt_gazebo/CommsClient.hh"
#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/protobuf/artifact.pb.h"

using namespace subt;

GZ_REGISTER_MODEL_PLUGIN(BaseStationPlugin)

//////////////////////////////////////////////////
BaseStationPlugin::BaseStationPlugin()
{
  ignmsg << "Base station plugin loaded" << std::endl;
}

//////////////////////////////////////////////////
void BaseStationPlugin::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent, "BaseStationPlugin() error: _parent pointer is NULL");
  GZ_ASSERT(_sdf,    "BaseStationPlugin() error: _sdf pointer is NULL");

  // We pass true to flag this client as private. The goal is to restrict the
  // ignition transport communication to only other nodes running within the
  // same process (e.g.: the GameLogicPlugin).
  this->client.reset(new subt::CommsClient(_parent->GetName(), true));
  this->client->Bind(&BaseStationPlugin::OnArtifact, this);

  this->worldUpdateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BaseStationPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////
void BaseStationPlugin::OnArtifact(const std::string &_srcAddress,
  const std::string &/*_dstAddress*/, const uint32_t _dstPort,
  const std::string &_data)
{
  subt::msgs::Artifact artifact;
  if (!artifact.ParseFromString(_data))
  {
    ignerr << "Error parsing artifact" << std::endl;
    return;
  }

  // Report this artifact to the scoring plugin.
  this->node.Request(kNewArtifactSrv, artifact);

  // Save the <address, port> of the sender for future ACK.
  std::lock_guard<std::mutex> lock(this->mutex);
  this->acks.push_back({_srcAddress, _dstPort});
}

/////////////////////////////////////////////
void BaseStationPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // We send an ACKs to the senders.
  for (const auto &client : this->acks)
  {
    std::string addr = std::get<0>(client);
    uint32_t port = std::get<1>(client);
    if (!this->client->SendTo("ACK", addr, port))
      std::cerr << "Error sending ACK to [" << addr << "]" << std::endl;
  }

  this->acks.clear();
}
