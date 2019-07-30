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

#ifndef SUBT_EXAMPLE__ENVIRONMENT_GRAPH_HH_
#define SUBT_EXAMPLE__ENVIRONMENT_GRAPH_HH_

#include <ignition/math/graph/Graph.hh>
#include <ignition/math/graph/Vertex.hh>

#include <string>
#include <sdf/sdf.hh>
#include <sdf/Model.hh>


namespace subt_example
{
  struct VertexData
  {
    std::string tileName;
    std::string tileType;
    sdf::Model* tileModel {nullptr};
  };

  using Vertex = ignition::math::graph::Vertex<VertexData>;
  using Graph = ignition::math::graph::UndirectedGraph<VertexData, int>;

  /// \brief Parse dot file into in-memory graph structure.
  void ParseGraph(const std::string &_dotFilename, Graph *_g);

}  // namespace subt_example

#endif  // SUBT_EXAMPLE__ENVIRONMENT_GRAPH_HH_

