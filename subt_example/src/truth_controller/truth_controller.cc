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

#include "EnvironmentGraph.hh"

#include <ignition/fuel_tools/Interface.hh>
#include <ignition/fuel_tools/ClientConfig.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>


#include <ros/ros.h>

#include <sdf/sdf.hh>
#include <sdf/Root.hh>
#include <sdf/Error.hh>

#include <string>

using namespace ignition;

class SubtTruthController
{
  public: SubtTruthController();

  private: ros::NodeHandle nh;
  private: ros::NodeHandle pnh {"~"};

  private: std::string dotFilename;
  private: std::string sdfFilename;

  private: std::unique_ptr<subt_example::Graph> graph;
};

/////////////////////////////////////////////////
void PopulateFromSdf(const std::string &_sdfFilename, subt_example::Graph *_g)
{

  auto fetch = [](const std::string &_uri) {
    fuel_tools::ClientConfig config;
    auto fuelClient = std::make_unique<fuel_tools::FuelClient>(config);
    return fuel_tools::fetchResourceWithClient(_uri, *fuelClient.get());
  };

  // Configure SDF to fetch assets from ignition fuel.
  sdf::setFindCallback(fetch);
  sdf::Root sdfRoot;
  auto errors = sdfRoot.Load(_sdfFilename);

  const sdf::World *sdfWorld = sdfRoot.WorldByIndex(0);
  sdf::ElementPtr worldElem = sdfWorld->Element();

  auto vertices = _g->Vertices();

  for (uint64_t modelIndex = 0; modelIndex < sdfWorld->ModelCount();
        ++modelIndex)
  {
    auto model = sdfWorld->ModelByIndex(modelIndex);
    auto modelName = model->Name();

    for (const auto & vertex: vertices)
    {
      const auto v = vertex.second.get();

      if (v.Data().tileName == modelName)
      {
        _g->VertexFromId(v.Id()).Data().tileModel =
          const_cast<sdf::Model*>(model);
      }
    }
  }
}

/////////////////////////////////////////////////
SubtTruthController::SubtTruthController()
{
  if (!this->pnh.getParam("dot_fname", this->dotFilename) ||
      !this->pnh.getParam("sdf_fname", this->sdfFilename))
  {
    std::cerr << "Didn't get parameters" << std::endl;
    return;
  }

  std::cout << "Got dot filename: " << this->dotFilename << std::endl;
  std::cout << "Got sdf filename: " << this->sdfFilename << std::endl;

  this->graph = std::make_unique<subt_example::Graph>();
  subt_example::ParseGraph(this->dotFilename, this->graph.get());

  PopulateFromSdf(this->sdfFilename, this->graph.get());

  auto vertices = this->graph->Vertices();
  for (const auto & vertex: vertices)
  {
    const auto v = vertex.second.get();

    if  (v.Data().tileModel == nullptr)
    {
      std::cout << v.Id() << " "
                << v.Data().tileName << " "
                << v.Data().tileType << std::endl;
    }
    else
    {
      std::cout << v.Id() << " "
                << v.Data().tileName << " "
                << v.Data().tileType << " "
                << v.Data().tileModel->Pose() << std::endl;
    }
  }
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "truth_controller_node");

  SubtTruthController truthController;

  ros::spin();
}


